#include <Arduino.h>

/* /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Components to be added:
// - display (6 digital pins)
// - rotary encoder (3 digital pins)
// - battery voltage monitor (1 analog pin)
// - MQ-135 CO2 sensor (1 analog pin)
// - 4 led's (4 digital pins):
//     - power on
//     - CO2 alert
//     - water too low
//     - temperature alert
*/ ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Connections (pins):
- LEDs:
    - TEMP_ALERT_LED-> Pin 2
    - LOW_BATTERY_LED     -> Pin 3
    - CO2_ALERT_LED -> Pin 4
    - WATER_LOW_LED -> Pin 5
- Rotary Encoder:
    - CLK -> Pin A0 (will be used with "digitalRead()")
    - DT  -> Pin 6
    - SW  -> Pin 7
- Display:
    - DC  -> Pin 8
    - RST -> Pin 9
    - CS  -> Pin 10
    - MOSI-> Pin 11
    - MISO-> Pin 12
    - SCK -> Pin 13
- Battery Voltage Monitor:
    - BAT_MON -> A1
- MQ-135 CO2 Sensor:
    - MQ135_PIN -> A2
*/ ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// pin declarations
#define TFT_CS   10
#define TFT_DC    8
#define TFT_RST   9

#define TEMP_ALERT_LED        2
#define LOW_BATTERY_LED       3
#define CO2_ALERT_LED         4
#define WATER_LOW_LED         5

#define ROTARY_DT            6
#define ROTARY_SW            7
#define ROTARY_CLK           A0

#define BATTERY_MONITOR      A1

#define MQ135_PIN            A2

// MQ-135 CO2 Sensor constants
#define RL_VALUE 10.0         // Load resistance (kΩ)
#define R0 76.63              // Sensor resistance in clean air (calibrate!)
#define VOLTAGE_RESOLUTION 5.0
#define ADC_RESOLUTION 4095.0 // 12-bit ADC on UNO R4 WIFI

#define CO2_A 110.47
#define CO2_B -2.862

// Compile-Time Constants
constexpr int CO2_Reads_delayTime = 2000;
constexpr float MIN_CO2_THRESHOLD = 1000.0; // ppm
constexpr float MAX_CO2_THRESHOLD = 5000.0; // ppm

constexpr float LOW_BATTERY_THRESHOLD = 3.3; // volts

void setup() {
    Serial.begin(9600);

    analogReadResolution(12); // UNO R4 has 12-bit ADC for better accuracy with MQ-135

    pinMode(TEMP_ALERT_LED, OUTPUT);
    pinMode(LOW_BATTERY_LED, OUTPUT);
    pinMode(CO2_ALERT_LED, OUTPUT);
    pinMode(WATER_LOW_LED, OUTPUT);

    pinMode(ROTARY_DT, INPUT);
    pinMode(ROTARY_SW, INPUT);
    pinMode(ROTARY_CLK, INPUT);

    // Attach interrupts to detect rotary encoder changes
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), rotaryUpdate, CHANGE);  // Detect rotary movement
    attachInterrupt(digitalPinToInterrupt(ROTARY_SW), buttonPressOnRotaryEncoder, FALLING);  // Detect button press (active low)
}

float getCO2_ppm() {
    static float co2ppm = 0.0; 
    static int lastReadTime = 0;

    int currentTime;
    currentTime = millis();

    if (currentTime - lastReadTime < CO2_Reads_delayTime) {
        return co2ppm; // Return last read value if delay time not met
    }

    int adcValue = analogRead(MQ135_PIN);
    float voltage = (adcValue * VOLTAGE_RESOLUTION) / ADC_RESOLUTION;
    float rs = ((VOLTAGE_RESOLUTION * RL_VALUE) / voltage) - RL_VALUE;
    float ratio = rs / R0;
    co2ppm = CO2_A * pow(ratio, CO2_B);

    // Serial.print("CO2 PPM: ");
    // Serial.println(co2ppm, 1);

    lastReadTime = currentTime;

    return co2ppm;
}

void loop() {

    // CO2 LOGIC
    float co2ppm = getCO2_ppm();

    if (co2ppm <= MIN_CO2_THRESHOLD || MAX_CO2_THRESHOLD <= co2ppm) {
        digitalWrite(CO2_ALERT_LED, HIGH);
    } else {
        digitalWrite(CO2_ALERT_LED, LOW);
    }

    // BATTERY MONITOR LOGIC
    float batteryVoltage = (analogRead(BATTERY_MONITOR) * VOLTAGE_RESOLUTION) / ADC_RESOLUTION;
    if (batteryVoltage <= LOW_BATTERY_THRESHOLD) {
        digitalWrite(LOW_BATTERY_LED, HIGH);
    } else {
        digitalWrite(LOW_BATTERY_LED, LOW);
    }

    delay(500); // Main loop delay
}

void buttonPressOnRotaryEncoder() {
  if (digitalRead(ROTARY_SW) == LOW) {  // Button pressed (active low)
    Serial.println("Button pressed!");
    // do something...
  }
}

// Function to handle rotary encoder
void rotaryUpdate() {
  static int last_clk = LOW;  // Store last clock pin state
  static int count = 0;

  int clk_state = digitalRead(ROTARY_CLK);
  int dt_state = digitalRead(ROTARY_DT);

  if (clk_state != last_clk) {
    if (dt_state != clk_state) {
      count--;  // Clockwise rotation
    } else {
      count++;  // Counter-clockwise rotation
    }
    // currentMode = abs((count / 2) % ModeCnt); // updates every 2 turns for better user feel (it has stoppers every 2 readings)

  }

  last_clk = clk_state;
}