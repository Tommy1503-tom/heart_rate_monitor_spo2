/*
  ================================================================
  ESP32-S3 + ST7789 + XPT2046 + MAX30102
  ADVANCED HEART RATE + SpO2 MONITOR
  ================================================================

  FEATURES
  ------------------------------------------------
  1. MAX30102 continuous FIFO sampling
  2. BPM scan every 1 second
  3. Moving Average BPM = 10 seconds
  4. BPM history graph = 60 seconds
  5. SpO2 (blood oxygen) using SparkFun maxim algorithm
  6. ST7789 320x240 Landscape
  7. XPT2046 touch page navigation
  8. WiFi + HTTP POST to Node.js server every 1 second
  9. Finger detection
  10. Heart beat animation
  11. Serial Monitor diagnostics

  ================================================================
  PIN CONFIGURATION
  ================================================================

  ST7789
  ------------------------------------------------
  SCLK = GPIO12
  MOSI = GPIO11
  CS   = GPIO8
  DC   = GPIO10
  RST  = GPIO9
  BL   = GPIO7

  XPT2046
  ------------------------------------------------
  SCLK = GPIO12
  MOSI = GPIO11
  MISO = GPIO13
  CS   = GPIO14
  IRQ  = GPIO6

  MAX30102
  ------------------------------------------------
  SDA  = GPIO4
  SCL  = GPIO5

  ================================================================
*/

#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#include <XPT2046_Touchscreen.h>

#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"


// ================================================================
// ST7789 PINS
// ================================================================

#define TFT_SCLK 12
#define TFT_MOSI 11
#define TFT_RST   9
#define TFT_DC   10
#define TFT_CS    8
#define TFT_BL    7


// ================================================================
// XPT2046 TOUCH PINS
// ================================================================

#define TOUCH_SCLK 12
#define TOUCH_MOSI 11
#define TOUCH_MISO 13
#define TOUCH_CS   14
#define TOUCH_IRQ   6


// ================================================================
// MAX30102 I2C
// ================================================================

#define I2C_SDA 4
#define I2C_SCL 5


// ================================================================
// DISPLAY
// ================================================================

#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240

const char* WIFI_SSID = "EE_144EE";
const char* WIFI_PASSWORD = "Tommy1503fun";

const char* WEB_SERVER =
    "http://192.168.1.107:3000/api/heart";

Adafruit_ST7789 tft(
  TFT_CS,
  TFT_DC,
  TFT_RST
);


// ================================================================
// TOUCH
// ================================================================

XPT2046_Touchscreen touch(
  TOUCH_CS,
  TOUCH_IRQ
);


// ================================================================
// SPI
// ================================================================

SPIClass *spi = &SPI;


// ================================================================
// MAX30102
// ================================================================

MAX30105 particleSensor;

bool sensorOK = false;


// ================================================================
// BPM
// ================================================================

float beatsPerMinute = 0.0;

int beatAvg = 0;


// ================================================================
// BPM BUFFER
// Moving Average 10 seconds
// ================================================================

#define MOVING_AVG_SIZE 10

float bpmBuffer[MOVING_AVG_SIZE];

uint8_t bpmBufferIndex = 0;

uint8_t bpmBufferCount = 0;

float movingAverage10s = 0.0;


// ================================================================
// BPM HISTORY
// 60 seconds
// ================================================================

#define HISTORY_SIZE 60

float bpmHistory[HISTORY_SIZE];

uint8_t historyIndex = 0;

uint8_t historyCount = 0;


// ================================================================
// SpO2
// ================================================================

#define SPO2_BUFFER_SIZE 100

uint32_t irBuffer[SPO2_BUFFER_SIZE];

uint32_t redBuffer[SPO2_BUFFER_SIZE];

int spo2BufferIndex = 0;

int32_t currentSpO2 = 0;


// ================================================================
// HEART RATE INTERNAL
// ================================================================

const byte RATE_SIZE = 4;

byte rates[RATE_SIZE];

byte rateSpot = 0;

unsigned long lastBeat = 0;


// ================================================================
// FINGER
// ================================================================

bool fingerPresent = false;

bool previousFingerState = false;


// ================================================================
// IR DC FILTER
// ================================================================

long irDC = 0;


// ================================================================
// HEART ICON
// ================================================================

bool heartIconOn = false;

unsigned long beatFlashUntil = 0;

const unsigned long BEAT_FLASH_DURATION = 150;


// ================================================================
// PAGE
// ================================================================

enum PageType
{
  PAGE_NUMERIC = 0,
  PAGE_GRAPH
};

uint8_t currentPage = PAGE_NUMERIC;

#define TOTAL_PAGES 2

bool pageNeedsRedraw = true;


// ================================================================
// GRAPH
// ================================================================

const int GX = 35;
const int GY = 60;
const int GW = 250;
const int GH = 125;

const int G_MID =
  GY + GH / 2;


// ================================================================
// TOUCH CALIBRATION
// ================================================================

#define TOUCH_X_MIN 200
#define TOUCH_X_MAX 3900

#define TOUCH_Y_MIN 200
#define TOUCH_Y_MAX 3900

#define TOUCH_LEFT_ZONE 106
#define TOUCH_RIGHT_ZONE 214


unsigned long lastTouchTime = 0;

const unsigned long TOUCH_DEBOUNCE = 350;

bool touchWasPressed = false;


// ================================================================
// SCAN
// ================================================================

#define SCAN_INTERVAL 1000

unsigned long lastScanTime = 0;


// ================================================================
// COLORS
// ================================================================

#define COLOR_BG       ST77XX_BLACK
#define COLOR_TEXT     ST77XX_WHITE
#define COLOR_TITLE    ST77XX_CYAN
#define COLOR_HR       ST77XX_RED
#define COLOR_TRACE    ST77XX_GREEN
#define COLOR_GRID     0x4208
#define COLOR_BAR      0x18E3
#define COLOR_AVG      ST77XX_YELLOW
#define COLOR_SPO2     ST77XX_CYAN


// ================================================================
// FORWARD DECLARATIONS
// ================================================================

void initHeartRateSensor();

void readHeartRate();

void processHeartbeat(
  unsigned long now
);

void performOneSecondScan();

void addBPMToMovingAverage(
  float bpm
);

void addBPMToHistory(
  float bpm
);

void resetHeartRate();

void handleTouch();

void nextPage();

void previousPage();

void drawCurrentPage();

void drawNumericPage();

void updateNumericValue();

void drawGraphPage();

void drawGraphValues();

void drawBPMGraph();

void drawHeader(
  const char *title
);

void drawTouchNavigation();

void drawHeartIcon(
  bool filled
);

void printCentered(
  const char *text,
  int16_t y,
  uint16_t color,
  uint8_t size
);

void sendHeartRateToWeb();


// ================================================================
// SEND TO WEB SERVER
// ================================================================
void sendHeartRateToWeb()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        return;
    }

    if (!fingerPresent)
    {
        return;
    }

    HTTPClient http;

    http.begin(WEB_SERVER);

    http.addHeader(
        "Content-Type",
        "application/json"
    );

    String json = "{";

    json += "\"bpm\":";
    json += String(beatAvg);

    json += ",";

    json += "\"avg10\":";
    json += String(movingAverage10s, 1);

    json += ",";

    json += "\"spo2\":";
    json += String(currentSpO2);

    json += ",";

    json += "\"finger\":true";

    json += "}";

    int httpCode =
        http.POST(json);

    Serial.print(
        "Web HTTP = "
    );

    Serial.println(
        httpCode
    );

    http.end();
}

void setup()
{
  Serial.begin(115200);

  delay(300);

  Serial.println();
  Serial.println();

  Serial.println(
    "================================================"
  );

  Serial.println(
    " ESP32-S3 HEART RATE + SpO2 MONITOR"
  );

  Serial.println(
    " MAX30102 + ST7789 + XPT2046"
  );

  Serial.println(
    " SCAN 1 SECOND"
  );

  Serial.println(
    " MOVING AVERAGE 10 SECONDS"
  );

  Serial.println(
    " BPM GRAPH 60 SECONDS"
  );

  Serial.println(
    "================================================"
  );


  // ============================================================
  // BACKLIGHT
  // ============================================================

  pinMode(
    TFT_BL,
    OUTPUT
  );

  digitalWrite(
    TFT_BL,
    HIGH
  );


  // ============================================================
  // SPI
  // ============================================================

  spi->begin(
    TFT_SCLK,
    TOUCH_MISO,
    TFT_MOSI,
    TFT_CS
  );


  // ============================================================
  // TFT
  // ============================================================

  Serial.println(
    "Initializing ST7789..."
  );

  tft.init(
    240,
    320
  );

  tft.setRotation(1);

  tft.fillScreen(
    COLOR_BG
  );


  // ============================================================
  // TOUCH
  // ============================================================

  Serial.println(
    "Initializing XPT2046..."
  );

  touch.begin(
    *spi
  );

  touch.setRotation(1);


  // ============================================================
  // SPLASH
  // ============================================================

  tft.fillScreen(
    COLOR_BG
  );

  tft.setTextColor(
    COLOR_TITLE
  );

  tft.setTextSize(2);

  tft.setCursor(
    30,
    60
  );

  tft.println(
    "ESP32-S3"
  );

  tft.setCursor(
    30,
    90
  );

  tft.println(
    "HEART RATE"
  );

  tft.setCursor(
    30,
    115
  );

  tft.println(
    "+ SpO2"
  );

  tft.setTextSize(1);

  tft.setTextColor(
    COLOR_TEXT
  );

  tft.setCursor(
    30,
    145
  );

  tft.println(
    "MAX30102"
  );

  tft.setCursor(
    30,
    160
  );

  tft.println(
    "10s Moving Average"
  );

  tft.setCursor(
    30,
    175
  );

  tft.println(
    "60s BPM Graph"
  );

  delay(1200);


  // ============================================================
  // MAX30102
  // ============================================================

  initHeartRateSensor();


  // ============================================================
  // CLEAR BUFFERS
  // ============================================================

  resetHeartRate();


  // ============================================================
  // TIMER
  // ============================================================

  lastScanTime =
    millis();


  // ============================================================
  // WIFI
  // ============================================================

  WiFi.mode(WIFI_STA);

  WiFi.begin(
      WIFI_SSID,
      WIFI_PASSWORD
  );

  Serial.print(
      "Connecting WiFi"
  );

  unsigned long startWiFi =
      millis();

  while (
      WiFi.status() != WL_CONNECTED &&
      millis() - startWiFi < 15000
  )
  {
      delay(300);

      Serial.print(".");
  }

  Serial.println();

  if (
      WiFi.status() == WL_CONNECTED
  )
  {
      Serial.println(
          "WiFi connected"
      );

      Serial.print(
          "ESP32 IP: "
      );

      Serial.println(
          WiFi.localIP()
      );
  }
  else
  {
      Serial.println(
          "WiFi connection failed"
      );
  }


  // ============================================================
  // FIRST PAGE
  // ============================================================

  drawCurrentPage();

  pageNeedsRedraw = false;


  Serial.println();
  Serial.println(
    "SYSTEM READY"
  );

  Serial.println();
}


// ================================================================
// LOOP
// ================================================================

void loop()
{
  // ============================================================
  // TOUCH
  // ============================================================

  handleTouch();


  // ============================================================
  // MAX30102
  // Continuous sampling
  // ============================================================

  if (sensorOK)
  {
    readHeartRate();
  }


  // ============================================================
  // PAGE REDRAW
  // ============================================================

  if (pageNeedsRedraw)
  {
    drawCurrentPage();

    pageNeedsRedraw = false;
  }


  // ============================================================
  // HEART FLASH
  // ============================================================

  if (
    currentPage == PAGE_NUMERIC &&
    heartIconOn
  )
  {
    if (
      millis() >= beatFlashUntil
    )
    {
      heartIconOn = false;

      drawHeartIcon(false);
    }
  }


  delay(1);
}


// ================================================================
// INITIALIZE MAX30102
// ================================================================

void initHeartRateSensor()
{
  Serial.println();

  Serial.println(
    "Initializing MAX30102..."
  );


  // ============================================================
  // I2C
  // ============================================================

  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );

  Wire.setClock(
    400000
  );


  // ============================================================
  // SENSOR
  // ============================================================

  if (
    !particleSensor.begin(
      Wire,
      I2C_SPEED_FAST
    )
  )
  {
    Serial.println(
      "ERROR: MAX30102 NOT FOUND"
    );

    sensorOK = false;

    tft.fillScreen(
      COLOR_BG
    );

    tft.setTextColor(
      COLOR_HR
    );

    tft.setTextSize(2);

    tft.setCursor(
      35,
      75
    );

    tft.println(
      "MAX30102 ERROR"
    );

    tft.setTextColor(
      COLOR_TEXT
    );

    tft.setTextSize(1);

    tft.setCursor(
      35,
      110
    );

    tft.println(
      "Check wiring:"
    );

    tft.setCursor(
      35,
      130
    );

    tft.println(
      "SDA = GPIO4"
    );

    tft.setCursor(
      35,
      145
    );

    tft.println(
      "SCL = GPIO5"
    );

    delay(2500);

    return;
  }


  // ============================================================
  // SENSOR CONFIG
  // ============================================================

  byte ledBrightness = 0x3F;

  byte sampleAverage = 2;

  byte ledMode = 2;

  int sampleRate = 200;

  int pulseWidth = 215;

  int adcRange = 4096;


  particleSensor.setup(
    ledBrightness,
    sampleAverage,
    ledMode,
    sampleRate,
    pulseWidth,
    adcRange
  );


  sensorOK = true;


  Serial.println(
    "MAX30102 OK"
  );

  Serial.println(
    "Sample rate : 200 SPS"
  );

  Serial.println(
    "LED mode    : RED + IR"
  );
}


// ================================================================
// RESET
// ================================================================

void resetHeartRate()
{
  beatsPerMinute = 0;

  beatAvg = 0;

  movingAverage10s = 0;

  rateSpot = 0;

  bpmBufferIndex = 0;

  bpmBufferCount = 0;

  historyIndex = 0;

  historyCount = 0;

  lastBeat = 0;

  irDC = 0;


  spo2BufferIndex = 0;

  currentSpO2 = 0;


  heartIconOn = false;


  for (
    byte i = 0;
    i < RATE_SIZE;
    i++
  )
  {
    rates[i] = 0;
  }


  for (
    byte i = 0;
    i < MOVING_AVG_SIZE;
    i++
  )
  {
    bpmBuffer[i] = 0;
  }


  for (
    byte i = 0;
    i < HISTORY_SIZE;
    i++
  )
  {
    bpmHistory[i] = 0;
  }
}


// ================================================================
// HEARTBEAT PROCESS
// ================================================================

void processHeartbeat(
  unsigned long now
)
{
  // ============================================================
  // First heartbeat
  // ============================================================

  if (
    lastBeat == 0
  )
  {
    lastBeat = now;

    return;
  }


  unsigned long delta =
    now - lastBeat;


  // ============================================================
  // Valid heartbeat range
  // ============================================================

  if (
    delta < 250 ||
    delta > 3000
  )
  {
    lastBeat = now;

    return;
  }


  lastBeat = now;


  beatsPerMinute =
    60.0 /
    (delta / 1000.0);


  // ============================================================
  // Valid BPM
  // ============================================================

  if (
    beatsPerMinute >= 20 &&
    beatsPerMinute <= 240
  )
  {
    rates[rateSpot] =
      (byte)beatsPerMinute;

    rateSpot++;


    if (
      rateSpot >= RATE_SIZE
    )
    {
      rateSpot = 0;
    }


    // ==========================================================
    // Short average
    // ==========================================================

    int sum = 0;

    int count = 0;


    for (
      byte i = 0;
      i < RATE_SIZE;
      i++
    )
    {
      if (
        rates[i] > 0
      )
      {
        sum += rates[i];

        count++;
      }
    }


    if (
      count > 0
    )
    {
      beatAvg =
        sum / count;
    }


    // ==========================================================
    // Heart animation
    // ==========================================================

    if (
      currentPage ==
      PAGE_NUMERIC
    )
    {
      heartIconOn = true;

      beatFlashUntil =
        millis() +
        BEAT_FLASH_DURATION;

      drawHeartIcon(true);
    }
  }
}


// ================================================================
// READ MAX30102
// ================================================================

void readHeartRate()
{
  // ============================================================
  // Read FIFO
  // ============================================================

  particleSensor.check();


  // ============================================================
  // Drain FIFO
  // ============================================================

  while (
    particleSensor.available()
  )
  {
    long irValue =
      particleSensor.getFIFOIR();

    long redValue =
      particleSensor.getFIFORed();


    // ==========================================================
    // Finger detect
    // ==========================================================

    bool newFingerState =
      (irValue > 50000);


    // ==========================================================
    // Finger state changed
    // ==========================================================

    if (
      newFingerState !=
      fingerPresent
    )
    {
      fingerPresent =
        newFingerState;


      if (
        fingerPresent
      )
      {
        Serial.println(
          "Finger DETECTED"
        );

        lastBeat = 0;

        irDC = 0;

        spo2BufferIndex = 0;
      }
      else
      {
        Serial.println(
          "Finger REMOVED"
        );

        resetHeartRate();


        if (
          currentPage ==
          PAGE_NUMERIC
        )
        {
          drawHeartIcon(false);

          updateNumericValue();
        }
      }
    }


    // ==========================================================
    // NO FINGER
    // ==========================================================

    if (
      !fingerPresent
    )
    {
      irDC = 0;
    }


    // ==========================================================
    // FINGER
    // ==========================================================

    else
    {
      if (
        checkForBeat(irValue)
      )
      {
        processHeartbeat(
          millis()
        );
      }


      // ========================================================
      // SpO2 buffering
      //
      // Fill a 100-sample window of IR + RED, then run the
      // SparkFun maxim algorithm once the window is full.
      // At ~100 SPS effective output this refreshes ~once/sec.
      // ========================================================

      irBuffer[spo2BufferIndex] =
        (uint32_t)irValue;

      redBuffer[spo2BufferIndex] =
        (uint32_t)redValue;

      spo2BufferIndex++;


      if (
        spo2BufferIndex >=
        SPO2_BUFFER_SIZE
      )
      {
        spo2BufferIndex = 0;


        int32_t spo2Value;

        int8_t validSPO2;

        int32_t heartRateAlgo;

        int8_t validHR;


        maxim_heart_rate_and_oxygen_saturation(
          irBuffer,
          SPO2_BUFFER_SIZE,
          redBuffer,
          &spo2Value,
          &validSPO2,
          &heartRateAlgo,
          &validHR
        );


        if (
          validSPO2 &&
          spo2Value > 0 &&
          spo2Value <= 100
        )
        {
          currentSpO2 =
            spo2Value;
        }
      }
    }


    // ==========================================================
    // WAVEFORM IR
    // ==========================================================

    if (
      currentPage ==
      PAGE_GRAPH
    )
    {
      // ไม่ใช้กราฟ IR แล้ว
      // กราฟหน้านี้จะเป็น BPM 60 วินาที
    }


    particleSensor.nextSample();
  }


  // ============================================================
  // 1 SECOND SCAN
  // ============================================================

  if (
    millis() - lastScanTime >=
    SCAN_INTERVAL
  )
  {
    lastScanTime =
      millis();

    performOneSecondScan();
  }
}


// ================================================================
// ONE SECOND SCAN
// ================================================================

void performOneSecondScan()
{
  float scanBPM = 0;


  // ============================================================
  // Current BPM
  // ============================================================

  if (
    fingerPresent &&
    beatAvg > 0
  )
  {
    scanBPM =
      (float)beatAvg;
  }


  // ============================================================
  // Add to moving average
  // ============================================================

  if (
    scanBPM > 0
  )
  {
    addBPMToMovingAverage(
      scanBPM
    );
  }


  // ============================================================
  // Add to 60s history
  // ============================================================

  if (
    scanBPM > 0
  )
  {
    addBPMToHistory(
      scanBPM
    );
  }
  else
  {
    // เก็บ 0 เมื่อไม่มีค่าที่เชื่อถือได้
    addBPMToHistory(
      0
    );
  }


  // ============================================================
  // SEND TO WEB SERVER
  // ============================================================

  sendHeartRateToWeb();


  // ============================================================
  // Serial Monitor
  // ============================================================

  Serial.println();

  Serial.println(
    "================================================"
  );

  Serial.println(
    "             1 SECOND SCAN"
  );

  Serial.println(
    "================================================"
  );


  Serial.print(
    "Finger          : "
  );

  if (
    fingerPresent
  )
  {
    Serial.println(
      "DETECTED"
    );
  }
  else
  {
    Serial.println(
      "NO FINGER"
    );
  }


  Serial.print(
    "Instant BPM     : "
  );

  if (
    scanBPM > 0
  )
  {
    Serial.println(
      scanBPM,
      1
    );
  }
  else
  {
    Serial.println(
      "--"
    );
  }


  Serial.print(
    "Moving Avg 10s  : "
  );

  if (
    movingAverage10s > 0
  )
  {
    Serial.println(
      movingAverage10s,
      1
    );
  }
  else
  {
    Serial.println(
      "--"
    );
  }


  Serial.print(
    "SpO2            : "
  );

  if (
    currentSpO2 > 0
  )
  {
    Serial.print(
      currentSpO2
    );

    Serial.println(
      " %"
    );
  }
  else
  {
    Serial.println(
      "--"
    );
  }


  Serial.print(
    "History points  : "
  );

  Serial.print(
    historyCount
  );

  Serial.println(
    " / 60"
  );


  Serial.println(
    "================================================"
  );


  // ============================================================
  // UPDATE DISPLAY
  // ============================================================

  if (
    currentPage ==
    PAGE_NUMERIC
  )
  {
    updateNumericValue();
  }


  if (
    currentPage ==
    PAGE_GRAPH
  )
  {
    drawGraphValues();

    drawBPMGraph();
  }
}


// ================================================================
// ADD MOVING AVERAGE
// ================================================================

void addBPMToMovingAverage(
  float bpm
)
{
  if (
    bpm <= 0
  )
  {
    return;
  }


  bpmBuffer[
    bpmBufferIndex
  ] = bpm;


  bpmBufferIndex++;


  if (
    bpmBufferIndex >=
    MOVING_AVG_SIZE
  )
  {
    bpmBufferIndex = 0;
  }


  if (
    bpmBufferCount <
    MOVING_AVG_SIZE
  )
  {
    bpmBufferCount++;
  }


  float sum = 0;

  for (
    uint8_t i = 0;
    i < bpmBufferCount;
    i++
  )
  {
    sum += bpmBuffer[i];
  }


  if (
    bpmBufferCount > 0
  )
  {
    movingAverage10s =
      sum /
      bpmBufferCount;
  }
}


// ================================================================
// ADD HISTORY
// ================================================================

void addBPMToHistory(
  float bpm
)
{
  bpmHistory[
    historyIndex
  ] = bpm;


  historyIndex++;


  if (
    historyIndex >=
    HISTORY_SIZE
  )
  {
    historyIndex = 0;
  }


  if (
    historyCount <
    HISTORY_SIZE
  )
  {
    historyCount++;
  }
}


// ================================================================
// TOUCH
// ================================================================

void handleTouch()
{
  bool pressed =
    touch.touched();


  if (!pressed)
  {
    touchWasPressed = false;

    return;
  }


  if (
    touchWasPressed
  )
  {
    return;
  }


  if (
    millis() - lastTouchTime <
    TOUCH_DEBOUNCE
  )
  {
    return;
  }


  lastTouchTime =
    millis();

  touchWasPressed = true;


  TS_Point p =
    touch.getPoint();


  int rawX =
    p.x;

  int rawY =
    p.y;


  Serial.print(
    "Touch RAW X="
  );

  Serial.print(
    rawX
  );

  Serial.print(
    " Y="
  );

  Serial.println(
    rawY
  );


  int x =
    map(
      rawX,
      TOUCH_X_MIN,
      TOUCH_X_MAX,
      0,
      SCREEN_WIDTH - 1
    );


  int y =
    map(
      rawY,
      TOUCH_Y_MIN,
      TOUCH_Y_MAX,
      0,
      SCREEN_HEIGHT - 1
    );


  x =
    constrain(
      x,
      0,
      SCREEN_WIDTH - 1
    );


  y =
    constrain(
      y,
      0,
      SCREEN_HEIGHT - 1
    );


  // ============================================================
  // LEFT
  // ============================================================

  if (
    x < TOUCH_LEFT_ZONE
  )
  {
    previousPage();

    return;
  }


  // ============================================================
  // RIGHT
  // ============================================================

  if (
    x > TOUCH_RIGHT_ZONE
  )
  {
    nextPage();

    return;
  }
}


// ================================================================
// NEXT PAGE
// ================================================================

void nextPage()
{
  currentPage++;


  if (
    currentPage >=
    TOTAL_PAGES
  )
  {
    currentPage =
      PAGE_NUMERIC;
  }


  pageNeedsRedraw = true;
}


// ================================================================
// PREVIOUS PAGE
// ================================================================

void previousPage()
{
  if (
    currentPage == 0
  )
  {
    currentPage =
      TOTAL_PAGES - 1;
  }
  else
  {
    currentPage--;
  }


  pageNeedsRedraw = true;
}


// ================================================================
// DRAW CURRENT PAGE
// ================================================================

void drawCurrentPage()
{
  tft.fillScreen(
    COLOR_BG
  );


  if (
    currentPage ==
    PAGE_NUMERIC
  )
  {
    drawNumericPage();
  }
  else
  {
    drawGraphPage();
  }


  drawTouchNavigation();
}


// ================================================================
// HEADER
// ================================================================

void drawHeader(
  const char *title
)
{
  tft.fillRect(
    0,
    0,
    SCREEN_WIDTH,
    30,
    COLOR_BAR
  );


  tft.setTextColor(
    COLOR_TITLE
  );

  tft.setTextSize(2);


  tft.setCursor(
    8,
    7
  );

  tft.print(
    title
  );


  tft.setTextSize(1);

  tft.setTextColor(
    COLOR_TEXT
  );


  tft.setCursor(
    275,
    10
  );

  tft.print(
    currentPage + 1
  );

  tft.print(
    "/"
  );

  tft.print(
    TOTAL_PAGES
  );
}


// ================================================================
// CENTER TEXT
// ================================================================

void printCentered(
  const char *text,
  int16_t y,
  uint16_t color,
  uint8_t size
)
{
  int16_t x1;

  int16_t y1;

  uint16_t w;

  uint16_t h;


  tft.setTextSize(
    size
  );

  tft.setTextColor(
    color
  );


  tft.getTextBounds(
    text,
    0,
    y,
    &x1,
    &y1,
    &w,
    &h
  );


  int16_t x =
    (
      SCREEN_WIDTH -
      (int16_t)w
    ) / 2;


  tft.setCursor(
    x,
    y
  );

  tft.print(
    text
  );
}


// ================================================================
// NUMERIC PAGE
// ================================================================

void drawNumericPage()
{
  drawHeader(
    "HEART RATE"
  );


  drawHeartIcon(
    false
  );


  updateNumericValue();
}


// ================================================================
// NUMERIC DISPLAY
// ================================================================

void updateNumericValue()
{
  // ============================================================
  // Clear main area
  // ============================================================

  tft.fillRect(
    0,
    58,
    SCREEN_WIDTH,
    160,
    COLOR_BG
  );


  // ============================================================
  // NO FINGER
  // ============================================================

  if (
    !fingerPresent
  )
  {
    printCentered(
      "--",
      68,
      0x7BEF,
      7
    );


    printCentered(
      "PLACE FINGER ON SENSOR",
      168,
      0x7BEF,
      1
    );


    printCentered(
      "SpO2: --      10s AVG: --",
      190,
      COLOR_AVG,
      1
    );


    return;
  }


  // ============================================================
  // READING
  // ============================================================

  if (
    beatAvg <= 0
  )
  {
    printCentered(
      "--",
      68,
      COLOR_TITLE,
      7
    );


    printCentered(
      "READING...",
      168,
      COLOR_TITLE,
      1
    );


    printCentered(
      "SpO2: --      10s AVG: --",
      190,
      COLOR_AVG,
      1
    );


    return;
  }


  // ============================================================
  // BPM
  // ============================================================

  char bpmText[10];


  snprintf(
    bpmText,
    sizeof(bpmText),
    "%d",
    beatAvg
  );


  printCentered(
    bpmText,
    68,
    COLOR_HR,
    7
  );


  printCentered(
    "BPM",
    160,
    COLOR_TRACE,
    2
  );


  // ============================================================
  // SpO2
  // ============================================================

  char spo2Text[16];


  if (
    currentSpO2 > 0
  )
  {
    snprintf(
      spo2Text,
      sizeof(spo2Text),
      "SpO2: %d%%",
      (int)currentSpO2
    );
  }
  else
  {
    snprintf(
      spo2Text,
      sizeof(spo2Text),
      "SpO2: --"
    );
  }


  printCentered(
    spo2Text,
    177,
    COLOR_SPO2,
    1
  );


  // ============================================================
  // 10 SECOND MOVING AVERAGE
  // ============================================================

  char avgText[32];


  if (
    movingAverage10s > 0
  )
  {
    snprintf(
      avgText,
      sizeof(avgText),
      "10s AVG: %.1f BPM",
      movingAverage10s
    );
  }
  else
  {
    snprintf(
      avgText,
      sizeof(avgText),
      "10s AVG: --"
    );
  }


  printCentered(
    avgText,
    193,
    COLOR_AVG,
    1
  );
}


// ================================================================
// HEART ICON
// ================================================================

void drawHeartIcon(
  bool filled
)
{
  const int cx =
    SCREEN_WIDTH / 2;

  const int cy = 42;

  const int r = 8;


  uint16_t color =
    filled
      ? ST77XX_RED
      : COLOR_GRID;


  tft.fillRect(
    cx - r - 3,
    cy - r - 3,
    (r + 3) * 2,
    r * 2 + 12,
    COLOR_BG
  );


  if (
    filled
  )
  {
    tft.fillCircle(
      cx - r / 2,
      cy,
      r / 2 + 1,
      color
    );


    tft.fillCircle(
      cx + r / 2,
      cy,
      r / 2 + 1,
      color
    );


    tft.fillTriangle(
      cx - r - 1,
      cy,
      cx + r + 1,
      cy,
      cx,
      cy + r + 3,
      color
    );
  }
  else
  {
    tft.drawCircle(
      cx - r / 2,
      cy,
      r / 2 + 1,
      color
    );


    tft.drawCircle(
      cx + r / 2,
      cy,
      r / 2 + 1,
      color
    );


    tft.drawLine(
      cx - r - 1,
      cy,
      cx,
      cy + r + 3,
      color
    );


    tft.drawLine(
      cx + r + 1,
      cy,
      cx,
      cy + r + 3,
      color
    );
  }
}


// ================================================================
// GRAPH PAGE
// ================================================================

void drawGraphPage()
{
  drawHeader(
    "BPM 60 SEC GRAPH"
  );


  drawGraphValues();


  drawBPMGraph();
}


// ================================================================
// GRAPH VALUES
// ================================================================

void drawGraphValues()
{
  // ============================================================
  // Clear top section
  // ============================================================

  tft.fillRect(
    0,
    32,
    SCREEN_WIDTH,
    25,
    COLOR_BG
  );


  tft.setTextSize(1);

  tft.setTextColor(
    COLOR_TEXT
  );


  tft.setCursor(
    8,
    38
  );

  tft.print(
    "NOW:"
  );


  tft.setTextSize(2);

  tft.setTextColor(
    COLOR_HR
  );


  tft.setCursor(
    45,
    34
  );


  if (
    fingerPresent &&
    beatAvg > 0
  )
  {
    tft.print(
      beatAvg
    );

    tft.print(
      " BPM"
    );
  }
  else
  {
    tft.print(
      "--"
    );
  }


  // ============================================================
  // Moving average
  // ============================================================

  tft.setTextSize(1);

  tft.setTextColor(
    COLOR_AVG
  );


  tft.setCursor(
    185,
    32
  );

  tft.print(
    "AVG10: "
  );


  if (
    movingAverage10s > 0
  )
  {
    tft.print(
      movingAverage10s,
      1
    );
  }
  else
  {
    tft.print(
      "--"
    );
  }


  // ============================================================
  // SpO2
  // ============================================================

  tft.setTextColor(
    COLOR_SPO2
  );


  tft.setCursor(
    185,
    45
  );

  tft.print(
    "SpO2: "
  );


  if (
    currentSpO2 > 0
  )
  {
    tft.print(
      (int)currentSpO2
    );

    tft.print(
      "%"
    );
  }
  else
  {
    tft.print(
      "--"
    );
  }
}


// ================================================================
// BPM GRAPH
// ================================================================

void drawBPMGraph()
{
  // ============================================================
  // Graph area
  // ============================================================

  const int x0 = GX;

  const int y0 = GY;

  const int width = GW;

  const int height = GH;


  // ============================================================
  // Clear graph
  // ============================================================

  tft.fillRect(
    x0,
    y0,
    width,
    height,
    COLOR_BG
  );


  // ============================================================
  // Border
  // ============================================================

  tft.drawRect(
    x0,
    y0,
    width,
    height,
    COLOR_GRID
  );


  // ============================================================
  // Horizontal grid
  //
  // 40 / 80 / 120 / 160 / 200 BPM
  // ============================================================

  const int minBPM = 40;

  const int maxBPM = 200;


  for (
    int bpm = 40;
    bpm <= 200;
    bpm += 40
  )
  {
    int y =
      map(
        bpm,
        minBPM,
        maxBPM,
        y0 + height - 1,
        y0 + 1
      );


    tft.drawFastHLine(
      x0 + 1,
      y,
      width - 2,
      COLOR_GRID
    );


    tft.setTextSize(1);

    tft.setTextColor(
      COLOR_TEXT
    );


    char label[8];


    snprintf(
      label,
      sizeof(label),
      "%d",
      bpm
    );


    tft.setCursor(
      4,
      y - 3
    );

    tft.print(
      label
    );
  }


  // ============================================================
  // Vertical time grid
  // ============================================================

  for (
    int sec = 0;
    sec <= 60;
    sec += 10
  )
  {
    int x =
      x0 +
      map(
        sec,
        0,
        60,
        1,
        width - 2
      );


    tft.drawFastVLine(
      x,
      y0 + 1,
      height - 2,
      COLOR_GRID
    );
  }


  // ============================================================
  // Need at least 2 points
  // ============================================================

  if (
    historyCount < 2
  )
  {
    tft.setTextSize(1);

    tft.setTextColor(
      COLOR_TITLE
    );


    tft.setCursor(
      95,
      y0 + height / 2 - 4
    );


    tft.print(
      "Collecting BPM data..."
    );


    return;
  }


  // ============================================================
  // Find oldest point
  // ============================================================

  int oldestIndex;


  if (
    historyCount <
    HISTORY_SIZE
  )
  {
    oldestIndex = 0;
  }
  else
  {
    oldestIndex =
      historyIndex;
  }


  // ============================================================
  // Plot points
  // ============================================================

  int prevX = -1;

  int prevY = -1;


  for (
    int i = 0;
    i < historyCount;
    i++
  )
  {
    int index =
      (
        oldestIndex +
        i
      ) %
      HISTORY_SIZE;


    float bpm =
      bpmHistory[index];


    // ----------------------------------------------------------
    // Skip invalid
    // ----------------------------------------------------------

    if (
      bpm <= 0
    )
    {
      prevX = -1;

      prevY = -1;

      continue;
    }


    // ----------------------------------------------------------
    // Clamp graph
    // ----------------------------------------------------------

    float graphBPM =
      constrain(
        bpm,
        minBPM,
        maxBPM
      );


    int x =
      x0 +
      1 +
      (
        i *
        (width - 3)
      ) /
      (HISTORY_SIZE - 1);


    int y =
      map(
        (int)graphBPM,
        minBPM,
        maxBPM,
        y0 + height - 2,
        y0 + 2
      );


    // ----------------------------------------------------------
    // Connect line
    // ----------------------------------------------------------

    if (
      prevX >= 0
    )
    {
      tft.drawLine(
        prevX,
        prevY,
        x,
        y,
        COLOR_TRACE
      );
    }


    // ----------------------------------------------------------
    // Point
    // ----------------------------------------------------------

    tft.fillCircle(
      x,
      y,
      2,
      COLOR_TRACE
    );


    prevX = x;

    prevY = y;
  }


  // ============================================================
  // Bottom labels
  // ============================================================

  tft.setTextSize(1);

  tft.setTextColor(
    COLOR_TEXT
  );


  tft.setCursor(
    x0,
    y0 + height + 4
  );

  tft.print(
    "60s"
  );


  tft.setCursor(
    x0 + width / 2 - 5,
    y0 + height + 4
  );

  tft.print(
    "30s"
  );


  tft.setCursor(
    x0 + width - 15,
    y0 + height + 4
  );

  tft.print(
    "NOW"
  );
}


// ================================================================
// TOUCH NAVIGATION
// ================================================================

void drawTouchNavigation()
{
  tft.fillRect(
    0,
    222,
    SCREEN_WIDTH,
    18,
    COLOR_BAR
  );


  tft.setTextSize(1);

  tft.setTextColor(
    COLOR_TEXT
  );


  // Previous
  tft.setCursor(
    8,
    227
  );

  tft.print(
    "< PREV"
  );


  // Center
  tft.setCursor(
    136,
    227
  );

  tft.print(
    "TOUCH"
  );


  // Next
  tft.setCursor(
    265,
    227
  );

  tft.print(
    "NEXT >"
  );
}
