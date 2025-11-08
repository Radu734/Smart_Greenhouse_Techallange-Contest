#include <Arduino.h>

/* /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Components to be added:
// - Battery Voltage Monitor (1 analog pin)
// - Light Level Sensor (1 analog pin)
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
//     - LIGHT_LEVEL_SENSOR_PIN    -> A1
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

#define STEPPER_MOTOR_PIN_1       8
#define STEPPER_MOTOR_PIN_2       9
#define STEPPER_MOTOR_PIN_3      10
#define STEPPER_MOTOR_PIN_4      11

// upper lux shade thresholds
constexpr unsigned int shadeLevel_LuxThresholds[] = {500, 1000, 2000, 0xFFFFFFFF}; // -1, 0, 1, 2

void setup() {
    Serial.begin(9600);

    // Initialize pins
    pinMode(HEATER_RELAY_PIN, OUTPUT);
    pinMode(COOLING_FAN_RELAY_PIN, OUTPUT);
    pinMode(VENTILATION_SERVO_PIN, OUTPUT);
}