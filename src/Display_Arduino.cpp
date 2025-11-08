#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

/* /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Components to be added:
// - display (6 digital pins)
// - rotary encoder (3 digital pins)
// - 4 led's (4 digital pins):
//     - battery low
//     - CO2 alert
//     - water too low
//     - temperature alert
*/ ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Connections (pins):
- LEDs:
    - TEMP_ALERT_LED    -> Pin 2
    - LOW_BATTERY_LED   -> Pin 3
    - CO2_ALERT_LED     -> Pin 4
    - WATER_LOW_LED     -> Pin 5
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
*/ ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// pin declarations
#define TFT_CS   10
#define TFT_DC    8
#define TFT_RST   9

#define TEMP_ALERT_LED        4
#define LOW_BATTERY_LED       5
#define CO2_ALERT_LED         6
#define WATER_LOW_LED         7

#define ROTARY_DT            2
#define ROTARY_CLK           3
#define ROTARY_SW            A0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compile-Time Constants ////////////////////////////////////////////////////////////////////////////////////
constexpr int CO2_Reads_delayTime = 2000;
constexpr float MIN_CO2_THRESHOLD = 1000.0; // ppm
constexpr float MAX_CO2_THRESHOLD = 5000.0; // ppm

constexpr float LOW_BATTERY_THRESHOLD = 3.3; // volts

constexpr int maxTemperature_Celsius = 50;
constexpr int minTemperature_Celsius = 0;  

constexpr int minWaterLevel_TopTank_Percent = 20;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino Cloud variables ////////////////////////////////////////////////////////////////////////////////////
// Needs to be Synced between the display and the InsideGreenhouse arduino ////////////////////////////////////////////////////////////////////////////////////
volatile int currentTemperature_Celsius = 25;
volatile int currentHumidity_Percent = 50;
volatile int currentAQI_Index = 1; // dummy value OPTIONAL - can be calculated on the Display Arduino
volatile int current_WaterLevel_TopTank_Percent = 100;
volatile float current_CO2_ppm = 1200; // dummy value
volatile float current_BatteryVoltage_V = 4.2; // dummy value
volatile float currentLightLevel_Lux = 1500; // dummy value // NEEDS TO BE IMPLEMENTED ON THE SCREEN
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

enum MenuMode {
    MODE_OVERVIEW,
    MODE_TEMPERATURE,
    MODE_WATER_LEVEL,
    MODE_CO2_LEVEL,
    MODE_BATTERY_STATUS,
    MODE_LIGHT_LEVEL,
    ModeCnt
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables ////////////////////////////////////////////////////////////////////////////////////
int currentMode = MODE_OVERVIEW;
int lastMode = ModeCnt; // different from current to force initial update
volatile int rotaryEncoderTurnCount = 4;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Screen "Old" variables for screen flickering workaround ////////////////////////////////////////////////////////////////////////////////////
float old_CO2_ppm = currentTemperature_Celsius;
float old_BatteryVoltage_V = current_BatteryVoltage_V;
int old_Temperature_Celsius = currentTemperature_Celsius;
int old_WaterLevel_TopTank_Percent = current_WaterLevel_TopTank_Percent;
float old_LightLevel_Lux = currentLightLevel_Lux;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void buttonPressOnRotaryEncoder();
void rotaryUpdate();

void setup() {
    Serial.begin(9600);

    analogReadResolution(12); // UNO R4 has 12-bit ADC for better accuracy with MQ-135

    pinMode(TEMP_ALERT_LED, OUTPUT);
    pinMode(LOW_BATTERY_LED, OUTPUT);
    pinMode(CO2_ALERT_LED, OUTPUT);
    pinMode(WATER_LOW_LED, OUTPUT);

    pinMode(ROTARY_DT, INPUT_PULLUP);
    pinMode(ROTARY_CLK, INPUT_PULLUP);
    pinMode(ROTARY_SW, INPUT_PULLUP);

    tft.begin();
    SPI.beginTransaction(SPISettings(1000000000, MSBFIRST, SPI_MODE0)); // 4 MHz
    tft.setRotation(1);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);

    // Attach interrupts to detect rotary encoder changes
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), rotaryUpdate, CHANGE);  // Detect rotary movement
    attachInterrupt(digitalPinToInterrupt(ROTARY_SW), buttonPressOnRotaryEncoder, FALLING);  // Detect button press (active low)
}

#pragma region Menu Mode Functions

void Mode_Overview_Display() {
    if (currentMode != lastMode) {
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_WHITE);
    }

    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.println("Overview Mode");

    tft.setTextSize(1);

    tft.setCursor(0, 30);
    tft.setTextColor(ILI9341_BLACK);
    tft.print("Temp: ");
    tft.print(old_Temperature_Celsius);
    tft.println(" C");
    tft.setCursor(0, 30);
    tft.setTextColor(ILI9341_WHITE);
    tft.print("Temp: ");
    tft.print(currentTemperature_Celsius);
    tft.println(" C");

    tft.setCursor(0, 50);
    tft.setTextColor(ILI9341_BLACK);
    tft.print("Water Level: ");
    tft.print(old_WaterLevel_TopTank_Percent);
    tft.println(" %");
    tft.setCursor(0, 50);
    tft.setTextColor(ILI9341_WHITE);
    tft.print("Water Level: ");
    tft.print(current_WaterLevel_TopTank_Percent);
    tft.println(" %");

    tft.setCursor(0, 70);
    tft.setTextColor(ILI9341_BLACK);
    tft.print("CO2: ");
    tft.print(old_CO2_ppm, 1);
    tft.println(" ppm");
    tft.setCursor(0, 70);
    tft.setTextColor(ILI9341_WHITE);
    tft.print("CO2: ");
    tft.print(current_CO2_ppm, 1);
    tft.println(" ppm");

    tft.setCursor(0, 90);
    tft.setTextColor(ILI9341_BLACK);
    tft.print("Battery: ");
    tft.print(old_BatteryVoltage_V, 2);
    tft.println(" V");
    tft.setCursor(0, 90);
    tft.setTextColor(ILI9341_WHITE);
    tft.print("Battery: ");
    tft.print(current_BatteryVoltage_V, 2);
    tft.println(" V");
    // Add more display elements as needed
}

void Mode_Temperature_Display() {
    if (currentMode != lastMode) {
        tft.fillScreen(ILI9341_RED);
        tft.setTextColor(ILI9341_CYAN);
        tft.setTextSize(3);
    }

    // --- Text header ---
    tft.setCursor(60, 80);
    tft.setTextColor(ILI9341_RED);
    tft.println("Temperature:");

    tft.setCursor(60, 80);
    tft.setTextColor(ILI9341_CYAN);
    tft.println("Temperature:");

#pragma region DegreeSymbol
    // --- Numeric display ---
    int temp = currentTemperature_Celsius;
    uint8_t ts = 3;  // text size
    tft.setTextSize(ts);

    int x = 130;
    int y = 115;

    char buf[16];
    snprintf(buf, sizeof(buf), "%d", temp);  // e.g. "23"

    // === Red shadow layer ===
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(x, y);
    tft.print(buf);

    // Calculate text width to position ° symbol & 'C'
    int charWidth = 6 * ts;                  // 6px base width per char
    int textWidth = strlen(buf) * charWidth;
    int cx = x + textWidth + 4;              // horizontal position for °
    int cy = y - (8 * ts) / 2 + 15;               // move circle ABOVE the text

    // Draw red ° symbol
    tft.fillCircle(cx, cy, ts, ILI9341_RED);
#pragma endregion

    // Draw 'C' shadow
    tft.setCursor(cx + ts * 3, y);
    tft.print("C");

    // === Cyan foreground layer ===
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(x, y);
    tft.print(buf);

#pragma region DegreeSymbol
    // Draw cyan ° symbol slightly above 'C'
    tft.fillCircle(cx, cy, ts, ILI9341_CYAN);
#pragma endregion

    // Draw 'C'
    tft.setCursor(cx + ts * 3, y);
    tft.print("C");
}

void Mode_WaterLevel_Display() {

    if (currentMode != lastMode) {
        tft.fillScreen(ILI9341_BLUE);
        tft.setTextColor(ILI9341_WHITE);
        tft.setTextSize(3);
    }

    tft.setTextColor(ILI9341_BLUE);
    tft.setCursor(60, 80);
    tft.println("Water Level:");
    tft.setCursor(130, 115);
    tft.print(old_WaterLevel_TopTank_Percent);
    tft.println("%");
    
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(60, 80);
    tft.println("Water Level:");
    tft.setCursor(130, 115);
    tft.print(current_WaterLevel_TopTank_Percent);
    tft.println("%");
}

void Mode_CO2Level_Display() {
    // float current_CO2_ppm = getCO2_ppm();

    if (currentMode != lastMode) {
        tft.fillScreen(ILI9341_GREEN);
        tft.setTextColor(ILI9341_BLACK);
        tft.setTextSize(3);
    }

    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(60, 80);
    tft.println("CO2 Density:");
    tft.setCursor(90, 115);
    tft.print(old_CO2_ppm, 0);
    tft.println(" ppm");

    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(60, 80);
    tft.println("CO2 Density:");
    tft.setCursor(90, 115);
    tft.print(current_CO2_ppm, 0);
    tft.println(" ppm");
}

void Mode_BatteryStatus_Display() {

    if (currentMode != lastMode) {
        tft.fillScreen(ILI9341_YELLOW);
        tft.setTextColor(ILI9341_PURPLE);
        tft.setTextSize(3);
    }
        
    tft.setCursor(80, 90); // Center of the screen
    tft.fillScreen(ILI9341_YELLOW);
    tft.println("Battery V:");
    tft.setCursor(130, 130);
    tft.println(old_BatteryVoltage_V, 2);

    tft.setCursor(80, 90); // Center of the screen
    tft.fillScreen(ILI9341_YELLOW);
    tft.println("Battery V:");
    tft.setCursor(130, 130);
    tft.println(current_BatteryVoltage_V, 2);
}

void Mode_LightLevel_Display() {
    if (currentMode != lastMode) {
        tft.fillScreen(ILI9341_DARKGREY);
        tft.setTextColor(ILI9341_WHITE);
        tft.setTextSize(3);
    }

    tft.setCursor(60, 80);
    tft.setTextColor(ILI9341_DARKGREY);
    tft.println("Light Level:");
    tft.setCursor(90, 115);
    tft.print(old_LightLevel_Lux, 0);
    tft.println(" Lux");
    
    tft.setCursor(60, 80);
    tft.setTextColor(ILI9341_WHITE);
    tft.println("Light Level:");
    tft.setCursor(90, 115);
    tft.print(currentLightLevel_Lux, 0);
    tft.println(" Lux");
}

#pragma endregion

void loop() {

#pragma region Alerts Logic
    // // BATTERY MONITOR LOGIC
    if (current_BatteryVoltage_V <= LOW_BATTERY_THRESHOLD) {
        digitalWrite(LOW_BATTERY_LED, HIGH);
    } else {
        digitalWrite(LOW_BATTERY_LED, LOW);
    }

    // // CO2 LOGIC
    // float current_CO2_ppm = getCO2_ppm();

    if (current_CO2_ppm <= MIN_CO2_THRESHOLD || MAX_CO2_THRESHOLD <= current_CO2_ppm) {
        digitalWrite(CO2_ALERT_LED, HIGH);
    } else {
        digitalWrite(CO2_ALERT_LED, LOW);
    }

    // WATER LEVEL LOGIC
    if (current_WaterLevel_TopTank_Percent <= minWaterLevel_TopTank_Percent) { 
        digitalWrite(WATER_LOW_LED, HIGH);
    } else {
        digitalWrite(WATER_LOW_LED, LOW);
    }

    // TEMPERATURE LOGIC
    if (currentTemperature_Celsius >= maxTemperature_Celsius || currentTemperature_Celsius <= minTemperature_Celsius) {
        digitalWrite(TEMP_ALERT_LED, HIGH);
    } else {
        digitalWrite(TEMP_ALERT_LED, LOW);
    }
#pragma endregion

#pragma region Display Update Logic

currentMode = abs((rotaryEncoderTurnCount / 4) % ModeCnt); // updates every 4 turns for better user feel (it has stoppers every 2 readings)

    switch (currentMode) {
        case MODE_OVERVIEW:
            Mode_Overview_Display();
            break;
        case MODE_TEMPERATURE:
            Mode_Temperature_Display();
            break;
        case MODE_WATER_LEVEL:
            Mode_WaterLevel_Display();
            break;
        case MODE_CO2_LEVEL:
            Mode_CO2Level_Display();
            break;
        case MODE_BATTERY_STATUS:
            Mode_BatteryStatus_Display();
            break;
        // Add cases for other modes as needed
        case MODE_LIGHT_LEVEL:
            Mode_LightLevel_Display();
            break;
        default:
            Mode_Overview_Display();
            break;
    }

    lastMode = currentMode; // Update last mode

    old_BatteryVoltage_V = current_BatteryVoltage_V;
    old_CO2_ppm = current_CO2_ppm;
    old_Temperature_Celsius = currentTemperature_Celsius;
    old_WaterLevel_TopTank_Percent = current_WaterLevel_TopTank_Percent;

#pragma endregion

    delay(1000); // Main loop delay
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

  int clk_state = digitalRead(ROTARY_CLK);
  int dt_state = digitalRead(ROTARY_DT);

  if (clk_state != last_clk) {
    if (dt_state != clk_state) {
      rotaryEncoderTurnCount--;  // Clockwise rotation
    } else {
      rotaryEncoderTurnCount++;  // Counter-clockwise rotation
    }
    // currentMode = abs((count / 2) % ModeCnt); // updates every 2 turns for better user feel (it has stoppers every 2 readings)

  }

  last_clk = clk_state;
}