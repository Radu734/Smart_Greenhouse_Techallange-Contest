#include <Arduino.h>

/* /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Components to be added:
// - Relay Water Pump (1 digital pin)
// - Relay Watering Valve (1 digital pin)
// - (acts like pull down button) Water Level - Top Tank (1 digital pin)
// - (acts like pull down button) Water Level - Bottom Tank (1 digital pin) 
// - Sensor Ground Water Level (1 analog pin and 1 digital pin)
*/ ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Connections (pins):
// - Water Level - Top Tank:
//     - WATER_LEVEL_TOP_TANK_PIN     -> Pin 2
// - Water Level - Bottom Tank:
//     - WATER_LEVEL_BOTTOM_TANK_PIN  -> Pin 3
// - Relay Water Pump:
//     - WATER_PUMP_RELAY_PIN         -> Pin 4
// - Relay Watering Valve:
//     - WATERING_VALVE_RELAY_PIN     -> Pin 5
// - Sensor Ground Water Level:
//     - GROUND_WATER_LEVEL_Power_PIN-> Pin 6
//     - GROUND_WATER_LEVEL_ANALOG_PIN-> A3
*/ ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control logic (Action and Reaction):
- Water Tank Control Logic:
    - If Bottom Tank Water Sensor is HIGH (is full) -> Turn HIGH Water Pump Relay for PumpActivatedTime_Seconds -> Fill Top Tank 
- Irigation System Control Logic:
    - If ground water level analog reading < GroundWaterLevel_LowThreshold  -> Turn HIGH Watering Valve Relay
    - If ground water level analog reading > GroundWaterLevel_HighThreshold -> Turn LOW Watering Valve Relay 
*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// pin declarations
#define WATER_LEVEL_TOP_TANK_PIN        2
#define WATER_LEVEL_BOTTOM_TANK_PIN     3

#define WATER_PUMP_RELAY_PIN            4
#define WATERING_VALVE_RELAY_PIN        5

#define GROUND_WATER_LEVEL_Power_PIN    6
#define GROUND_WATER_LEVEL_ANALOG_PIN   A3

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compile-Time Constants ////////////////////////////////////////////////////////////////////////////////////
constexpr int PumpActivatedTime_Milliseconds = 10000; // 10 Seconds
constexpr int GroundWaterLevel_LowThreshold_Percentage = 20;  // under which we start watering
constexpr int GroundWaterLevel_HighThreshold_Percentage = 80; // over which we stop watering
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino Cloud variables ////////////////////////////////////////////////////////////////////////////////////
// Needs to be Synced between the display and the InsideGreenhouse arduino ////////////////////////////////////////////////////////////////////////////////////
volatile bool current_WaterLevel_TopTank_HalfFull = true; // dummy value
volatile long soilMoistureLevel_Percent = 100; // dummy value 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables ////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void handleBottomWaterTank_Pump() {
    static bool isPumping = false;
    static int pumpStartTime_Milliseconds = 0;

    if (digitalRead(WATER_LEVEL_BOTTOM_TANK_PIN) == HIGH && !isPumping) {
        digitalWrite(WATER_PUMP_RELAY_PIN, HIGH); // Start Pump if Bottom Tank is full
        isPumping = true;
        pumpStartTime_Milliseconds = millis();
    }

    if (isPumping && (millis() - pumpStartTime_Milliseconds >= PumpActivatedTime_Milliseconds)) {
        digitalWrite(WATER_PUMP_RELAY_PIN, LOW); // Stop Pump if time exceeded
        isPumping = false;
    }
}

void handleIrigationSystem() {
    static int sureCounter = 0;

    // Irrigation System Control Logic
    digitalWrite(GROUND_WATER_LEVEL_Power_PIN, HIGH); // Power the ground water level sensor
    delay(100); // Wait for sensor to stabilize

    int groundWaterLevelReading = analogRead(GROUND_WATER_LEVEL_ANALOG_PIN); // 0 - 4095 for 12-bit ADC

    digitalWrite(GROUND_WATER_LEVEL_Power_PIN, LOW); // Turn off sensor power to improve life cycle and save energy

    soilMoistureLevel_Percent = map(groundWaterLevelReading, 0, 4095, 0, 100); // Map to percentage

    if (soilMoistureLevel_Percent < GroundWaterLevel_LowThreshold_Percentage) {
        (sureCounter > 0) ? sureCounter++ : sureCounter = 1; // Increase counter towards opening valve
    } else if (soilMoistureLevel_Percent > GroundWaterLevel_HighThreshold_Percentage) {
        (sureCounter < 0) ? sureCounter-- : sureCounter = -1; // Decrease counter towards closing valve
    } else {
        // In between thresholds, reset counter
        sureCounter = 0;
    }

    if (sureCounter >= 10) {
        digitalWrite(WATERING_VALVE_RELAY_PIN, HIGH); // Open Watering Valve
        sureCounter = 0; // Reset counter to avoid overflow
    } else if (sureCounter <= -10) {
        digitalWrite(WATERING_VALVE_RELAY_PIN, LOW); // Close Watering Valve
        sureCounter = 0; // Reset counter to avoid overflow
    }

}

void setup() {
    Serial.begin(9600);

    // Initialize pins
    pinMode(WATER_LEVEL_TOP_TANK_PIN, INPUT_PULLDOWN);      // Active High
    pinMode(WATER_LEVEL_BOTTOM_TANK_PIN, INPUT_PULLDOWN);   // Active High

    pinMode(WATER_PUMP_RELAY_PIN, OUTPUT);
    pinMode(WATERING_VALVE_RELAY_PIN, OUTPUT);

    pinMode(GROUND_WATER_LEVEL_Power_PIN, OUTPUT);

    digitalWrite(WATER_PUMP_RELAY_PIN, LOW);        // Ensure pump is off
    digitalWrite(WATERING_VALVE_RELAY_PIN, LOW);    // Ensure watering valve is closed
}

void loop() {

    handleBottomWaterTank_Pump();
    handleIrigationSystem();

    delay(1000); // Main loop delay
}