#include <Arduino.h>
#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <DHT.h>

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
// 
// - Array of pointers to functions that influence each component (Heater, Fan, Ventilation Servo, Shade Stepper Motor):
//     - iterate through each function pointer (they are ordered from lowest priority to highest) and call the function to update the component state 
//     - higher priority functions will override lower priority ones's actions
// 
// - Function priority order (from lowest to highest):
//     - humidity control
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

constexpr int maxTemperature_Celsius = 50;
constexpr int minTemperature_Celsius = 0;  
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
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables ////////////////////////////////////////////////////////////////////////////////////
int currentShadeLevel = 0; // initial shade level
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma endregion

DHT dht(DHT_SENSOR_PIN, DHT_TYPE);

// upper lux shade thresholds
constexpr unsigned int shadeLevel_LuxThresholds[] = {500, 1000, 2000, 0xFFFFFFFF}; // -1, 0, 1, 2

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
#pragma endregion   

#pragma region Control_Functions

#pragma endregion

// priority function pointers array
void (*controlFunctions[])() = {
    // humidity control function pointer (to be implemented)
    // temperature control function pointer (to be implemented)
    // light level control function pointer (to be implemented)
    // CO2 control function pointer (to be implemented)
};

void setup() {
    Serial.begin(9600);

    // Initialize pins
    pinMode(HEATER_RELAY_PIN, OUTPUT);
    pinMode(COOLING_FAN_RELAY_PIN, OUTPUT);
    pinMode(VENTILATION_SERVO_PIN, OUTPUT);

    TSL2561.init();
}

void loop() {

    delay(3000);
}