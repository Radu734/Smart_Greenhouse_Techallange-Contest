#include <Arduino.h>
#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <DHT.h>
#include <Servo.h>
#include <Stepper.h>

/* /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Components to be added:
// - Battery Voltage Monitor (1 analog pin)
// - Light Level Sensor (I2C)
// - MQ-135 CO2 sensor (1 analog pin)
// - Relay Heater Light (1 digital pin)
// - Relay Cooling Fan (1 digital pin)
// - Servo Motor for Cooling Trap (1 digital pin)
// - Stepper Motor for Shade Control (4 digital pins) 
// - DHT sensor for Humidity and Temperature (1 digital pin)
*/ ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Connections (pins):
// - Battery Voltage Monitor:
//     - VOLTAGE_MONITOR_PIN       -> A0
// - Light Level Sensor:
//     - LIGHT_LEVEL_SENSOR_PIN    -> I2C
// - MQ-135 CO2 Sensor:
//     - MQ135_PIN                 -> A2
// - Heater Light Relay:
//     - HEATER_RELAY_PIN          -> Pin 2
// - Cooling Fan Relay:
//     - COOLING_FAN_RELAY_PIN     -> Pin 3
// - Servo Motor for Cooling Trap:
//     - VENTILATION_SERVO_PIN     -> Pin 4
// - DHT Sensor for Humidity and Temperature:
//     - DHT_SENSOR_PIN            -> Pin 5
// - Stepper Motor Shade Control:
//     - StepperMotor_PIN_1        -> Pin 8
//     - StepperMotor_PIN_2        -> Pin 9
//     - StepperMotor_PIN_3        -> Pin 10
//     - StepperMotor_PIN_4        -> Pin 11
*/ ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Stepper Motor Control Notes:
// - 2048 steps per revolution (for 28BYJ-48 stepper motor)
// - 8 revolutions for passing from one shade level to another
// - 4 levels of shade (-1 = no shade and heater on, 0 = no shade and heater off, 1 = medium shade, 2 = full shade)
*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control logic (Action and Reaction):
// - Temperature Control:
//     - If temperature < minTemperature_Celsius: Activate Heater Relay
//     - If temperature > maxTemperature_Celsius: Activate Cooling Fan Relay and Open Cooling Trap
// - CO2 above maxCO2_ppm -> !!!DANGER!!! -> Activate Ventilation System (Open Cooling Trap and turn on Cooling Fan)
// - Light Level Control:
//     - iterate trough upper lumen thresholds for each shade level array until finding the first threshold that is higher than the current light level
//     - set the shade level to the index of that threshold minus one
//     - 4 levels of shade (-1 = no shade and heater on, 0 = no shade and heater off, 1 = medium shade, 2 = full shade)
// - DHT Humidity & Temperature Sensor:
//     - humidity too low -> increase ventilation (open cooling trap more)
//     - humidity too high -> decrease ventilation (close cooling trap more), turn on cooling fan
//     - temperature too low -> turn on heater, turn off cooling fan & close cooling trap
//     - temperature too high -> turn on cooling fan & open cooling trap, turn off heater

// - Function priority order (from lowest to highest):
//     - temperature control
//     - light level control
//     - CO2 control
*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// pin declarations
#define VOLTAGE_MONITOR_PIN        A0
#define LIGHT_LEVEL_SENSOR_PIN     A1
#define MQ135_PIN                  A2
#define HEATER_RELAY_PIN           2
#define COOLING_FAN_RELAY_PIN      3
#define VENTILATION_SERVO_PIN      4

#define DHT_SENSOR_PIN             5
#define DHT_TYPE                   DHT22

#define STEPPER_MOTOR_PIN_1       8
#define STEPPER_MOTOR_PIN_2       9
#define STEPPER_MOTOR_PIN_3      10
#define STEPPER_MOTOR_PIN_4      11

// MQ-135 CO2 Sensor constants
#define RL_VALUE 10.0         // Load resistance (kΩ)
#define R0 76.63              // Sensor resistance in clean air (calibrate!)
#define VOLTAGE_RESOLUTION 5.0
#define ADC_RESOLUTION 4095.0 // 12-bit ADC on UNO R4 WIFI

#define CO2_A 110.47
#define CO2_B -2.862

#pragma region Variable_Definitions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compile-Time Constants ////////////////////////////////////////////////////////////////////////////////////
constexpr int CO2_Reads_delayTime = 2000;
constexpr float MIN_CO2_THRESHOLD = 1000.0; // ppm
constexpr float MAX_CO2_THRESHOLD = 5000.0; // ppm
constexpr float CO2_Hysteresis_ppm = 200.0; // CO2 difference before switching states to avoid oscillation

constexpr int maxTemperature_Celsius = 50;
constexpr int minTemperature_Celsius = 0;  
constexpr int temperatureHysteresis_Celsius = 2; // Temperature difference before switching states to avoid oscillation

constexpr int maxHumidity_Percent = 80;
constexpr int minHumidity_Percent = 40;

constexpr int closed_VentilationServoAngle = 40;   // degrees
constexpr int open_VentilationServoAngle = 180;    // degrees

constexpr int stepsPerRevolution_StepperMotor = 2048; // for 28BYJ-48 stepper motor
constexpr int spaceBetweenShadeLevels_Steps = 2048 * 8; // 8 revolutions for passing from one shade level to another
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino Cloud variables ////////////////////////////////////////////////////////////////////////////////////
// Needs to be Synced between the Display arduino and this arduino ////////////////////////////////////////////////////////////////////////////////////
volatile int currentTemperature_Celsius = 25;
volatile int currentHumidity_Percent = 50;
volatile int currentAQI_Index = 1; // dummy value OPTIONAL - can be calculated on the Display Arduino
volatile int current_WaterLevel_TopTank_Percent = 100;
volatile float current_CO2_ppm = 1200; // dummy value
volatile float current_BatteryVoltage_V = 4.2; // dummy value
volatile float currentLightLevel_Lux = 1500; // dummy value
volatile int targetShadeLevel = 0; // desired shade level
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables ////////////////////////////////////////////////////////////////////////////////////
int currentShadeLevel = 0; // initial shade level
int Relay_HeaterLight_TargetState = 0;
int Relay_ColingFan_TargetState = 0;
int Servo_CoolingVentilator_TargetAngle = closed_VentilationServoAngle;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Screen "Old" variables for screen flickering workaround ////////////////////////////////////////////////////////////////////////////////////
float old_CO2_ppm = currentTemperature_Celsius;
float old_BatteryVoltage_V = current_BatteryVoltage_V;
int old_Temperature_Celsius = currentTemperature_Celsius;
int old_WaterLevel_TopTank_Percent = current_WaterLevel_TopTank_Percent;
float old_LightLevel_Lux = currentLightLevel_Lux;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#pragma endregion

DHT dht(DHT_SENSOR_PIN, DHT_TYPE);
Servo coolingVentilatorServo;
Stepper stepperMotor_shadeLevel(stepsPerRevolution_StepperMotor, 8, 9, 10, 11);

// recorded lux values per light intensity: 1200, 6000, 15000, 55000
// upper lux shade thresholds
constexpr unsigned int shadeLevel_LuxThresholds[] = {1200, 6000, 15000, 0xFFFFFFFF}; // -1, 0, 1, 2

#pragma region Update_Global_Variables_Functions
void updateLightLevel() {
  currentLightLevel_Lux = TSL2561.readVisibleLux();
}

void updateBatteryVoltage() {
    current_BatteryVoltage_V = (analogRead(VOLTAGE_MONITOR_PIN) * VOLTAGE_RESOLUTION) / ADC_RESOLUTION;
}

void updateCO2_ppm() {
    
    static int lastReadTime = 0;

    int currentTime;
    currentTime = millis();

    if (currentTime - lastReadTime < CO2_Reads_delayTime) {
        return; // Return last read value if delay time not met
    }

    int adcValue = analogRead(MQ135_PIN);
    float voltage = (adcValue * VOLTAGE_RESOLUTION) / ADC_RESOLUTION;
    float rs = ((VOLTAGE_RESOLUTION * RL_VALUE) / voltage) - RL_VALUE;
    float ratio = rs / R0;
    current_CO2_ppm = CO2_A * pow(ratio, CO2_B);

    // Serial.print("CO2 PPM: ");
    // Serial.println(current_CO2_ppm, 1);

    lastReadTime = currentTime;
}

void updateHumidityAndTemperature() {
    currentHumidity_Percent = dht.readHumidity();
    currentTemperature_Celsius = dht.readTemperature();
    if (isnan(currentHumidity_Percent) || isnan(currentTemperature_Celsius)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }
    currentAQI_Index = dht.computeHeatIndex(currentTemperature_Celsius, currentHumidity_Percent, false);

    // Serial.print("Humidity: ");
    // Serial.print(currentHumidity_Percent);
    // Serial.print(" %  |  Temperature: ");
    // Serial.print(currentTemperature_Celsius);
    // Serial.println(" C");
}

void updateAllSensors() {
    old_BatteryVoltage_V = current_BatteryVoltage_V;
    old_CO2_ppm = current_CO2_ppm;
    old_Temperature_Celsius = currentTemperature_Celsius;
    old_WaterLevel_TopTank_Percent = current_WaterLevel_TopTank_Percent;

    updateLightLevel();
    updateBatteryVoltage();
    updateCO2_ppm();
    updateHumidityAndTemperature();

    // handle ARDUINO CLOUD sync here
}
#pragma endregion   

#pragma region Control_Functions
void controlTemperature() {
    if (currentTemperature_Celsius <= minTemperature_Celsius - temperatureHysteresis_Celsius) {
        digitalWrite(HEATER_RELAY_PIN, HIGH);
        digitalWrite(COOLING_FAN_RELAY_PIN, LOW);
        coolingVentilatorServo.write(closed_VentilationServoAngle);
        return;
    }
    if (currentTemperature_Celsius >= maxTemperature_Celsius + temperatureHysteresis_Celsius) {
        digitalWrite(HEATER_RELAY_PIN, LOW);
        digitalWrite(COOLING_FAN_RELAY_PIN, HIGH);
        ;
        return;
    }

    // Within acceptable range
    digitalWrite(HEATER_RELAY_PIN, LOW);
    digitalWrite(COOLING_FAN_RELAY_PIN, LOW);

    // We will progressively open / close the ventilation servo based on temperature
    int servoAngle = map(currentTemperature_Celsius, minTemperature_Celsius, maxTemperature_Celsius, closed_VentilationServoAngle, open_VentilationServoAngle);
}

void controlLightLevel() {

    for (int i = 0; i < sizeof(shadeLevel_LuxThresholds) / sizeof(shadeLevel_LuxThresholds[0]); i++) {
        if (currentLightLevel_Lux < shadeLevel_LuxThresholds[i]) {
            targetShadeLevel = i - 1;
            break;
        }
    }

    if (targetShadeLevel != currentShadeLevel) {
        int stepsToMove = (targetShadeLevel - currentShadeLevel) * spaceBetweenShadeLevels_Steps;
        stepperMotor_shadeLevel.step(stepsToMove); // Move stepper motor to target shade level
        currentShadeLevel = targetShadeLevel;
    }

    if (currentShadeLevel == -1) {
        digitalWrite(HEATER_RELAY_PIN, HIGH);
    } else {
        digitalWrite(HEATER_RELAY_PIN, LOW);
    }
}

void controlCO2() {
    // CO2 to HIGH - danger to people -> decrease by Hysteresis instead of adding to max to avoid "killing" people with the Hysteresis surpassi
    if (current_CO2_ppm >= MAX_CO2_THRESHOLD - CO2_Hysteresis_ppm) {
        digitalWrite(COOLING_FAN_RELAY_PIN, HIGH);
        coolingVentilatorServo.write(open_VentilationServoAngle);
        return;
    }
    if (current_CO2_ppm <= MIN_CO2_THRESHOLD - CO2_Hysteresis_ppm) {
        digitalWrite(COOLING_FAN_RELAY_PIN, LOW);
        return;
    }
}

#pragma endregion

void setup() {
    Serial.begin(9600);

    // Initialize pins
    pinMode(HEATER_RELAY_PIN, OUTPUT);
    pinMode(COOLING_FAN_RELAY_PIN, OUTPUT);
    pinMode(VENTILATION_SERVO_PIN, OUTPUT);

    TSL2561.init();

    coolingVentilatorServo.attach(VENTILATION_SERVO_PIN);
}

void loop() {
    updateAllSensors();

    if (currentTemperature_Celsius != old_Temperature_Celsius) {
        controlTemperature();
    }
    if (currentLightLevel_Lux != old_LightLevel_Lux) {
        controlLightLevel();
    }
    if (current_CO2_ppm != old_CO2_ppm) {
        controlCO2();
    }

    delay(3000);
}