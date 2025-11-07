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

#define MQ135_PIN           A2

// MQ-135 CO2 Sensor constants
#define RL_VALUE 10.0         // Load resistance (kΩ)
#define R0 76.63              // Sensor resistance in clean air (calibrate!)
#define VOLTAGE_RESOLUTION 5.0
#define ADC_RESOLUTION 4095.0 // 12-bit ADC on UNO R4

#define CO2_A 110.47
#define CO2_B -2.862

void setup() {
    pinMode(TEMP_ALERT_LED, OUTPUT);
    pinMode(LOW_BATTERY_LED, OUTPUT);
    pinMode(CO2_ALERT_LED, OUTPUT);
    pinMode(WATER_LOW_LED, OUTPUT);

    pinMode(ROTARY_DT, INPUT);
    pinMode(ROTARY_SW, INPUT);
    pinMode(ROTARY_CLK, INPUT);
}

void loop() {

}