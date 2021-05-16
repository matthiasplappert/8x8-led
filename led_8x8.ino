// Adafruit_NeoMatrix example for single NeoPixel Shield.
// Scrolls 'Howdy' across the matrix in a portrait (vertical) orientation.

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#include <hp_BH1750.h>

// https://sites.google.com/site/astudyofentropy/project-definition/timer-jitter-entropy-sources/entropy-library
#include <Entropy.h>

#define PIN 6

// MATRIX DECLARATION:
// Parameter 1 = width of NeoPixel matrix
// Parameter 2 = height of matrix
// Parameter 3 = pin number (most are valid)
// Parameter 4 = matrix layout flags, add together as needed:
//   NEO_MATRIX_TOP, NEO_MATRIX_BOTTOM, NEO_MATRIX_LEFT, NEO_MATRIX_RIGHT:
//     Position of the FIRST LED in the matrix; pick two, e.g.
//     NEO_MATRIX_TOP + NEO_MATRIX_LEFT for the top-left corner.
//   NEO_MATRIX_ROWS, NEO_MATRIX_COLUMNS: LEDs are arranged in horizontal
//     rows or in vertical columns, respectively; pick one or the other.
//   NEO_MATRIX_PROGRESSIVE, NEO_MATRIX_ZIGZAG: all rows/columns proceed
//     in the same order, or alternate lines reverse direction; pick one.
//   See example below for these values in action.
// Parameter 5 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_GRBW    Pixels are wired for GRBW bitstream (RGB+W NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)


// Configure for 8x8 matrix.
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 8, PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
  NEO_GRB            + NEO_KHZ800);

hp_BH1750 BH1750;

unsigned long lastStateUpdate = millis();

#define STATE_UPDATE_INTERVAL 2000    // 2 seconds

unsigned long lastStateReset = millis();

#define STATE_RESET_INTERVAL 3600000 // 1 hour

unsigned long lastBrightnessUpdate = millis();

int brightness = -1;


#define BRIGHTNESS_UPDATE_INTERVAL 600000 // 10 minutes

/* Different brightness levels empircally measured:
 *  Sunny day: 20 lux
 *  Thumb on top of it: 3.0
 *  Thumb on top with bright LED: 8.0
 */
#define BRIGHTNESS_THRESHOLD_OFF 6.0

#define BRIGHTNESS_OFF 0
#define BRIGHTNESS_ON  5

uint8_t color1[3];

uint8_t color2[3];

bool rules[256];

bool state[8][8];

void resetColors() {
  color1[0] = random(256);
  color1[1] = random(256);
  color1[2] = random(256);

  color2[0] = random(256);
  color2[1] = random(256);
  color2[2] = random(256);
}

void resetRules() {
  for (int i = 0; i < 256; i++) {
    rules[i] = bool(random(2));
  }
}

void resetState() {
  for (int x = 0; x < 8; x++) {
    for (int y = 0; y < 8; y++) {
      state[x][y] = bool(random(2));
    }
  }
}

bool isValid(int x, int y) {
  if (x < 0) {
    return false;
  }
  if (y < 0) {
    return false;
  }
  if (x >= 8) {
    return false;
  }
  if (y >= 8) {
    return false;
  }
  return true;
}

int getNeighbors(int x, int y) {
  uint8_t code;
  if (isValid(x - 1, y - 1)) {
    bitWrite(code, 0, state[x - 1][y - 1]);
  }
  if (isValid(x - 1, y)) {
    bitWrite(code, 1, state[x - 1][y]);
  }
  if (isValid(x - 1, y + 1)) {
    bitWrite(code, 2, state[x - 1][y + 1]);
  }
  if (isValid(x, y - 1)) {
    bitWrite(code, 3, state[x][y - 1]);
  }
  if (isValid(x, y + 1)) {
    bitWrite(code, 4, state[x][y + 1]);
  }
  if (isValid(x + 1, y - 1)) {
    bitWrite(code, 5, state[x + 1][y - 1]);
  }
  if (isValid(x + 1, y)) {
    bitWrite(code, 6, state[x + 1][y - 1]);
  }
  if (isValid(x + 1, y + 1)) {
    bitWrite(code, 7, state[x + 1][y - 1]);
  }
  return code;
}

void updateState() {
  bool newState[8][8];

  for (int x = 0; x < 8; x++) {
    for (int y = 0; y < 8; y++) {
      uint8_t code = getNeighbors(x, y);
      bool newValue = rules[code];
      newState[x][y] = newValue;
    }
  }

  // Copy new state into old state.
  for (int x = 0; x < 8; x++) {
    for (int y = 0; y < 8; y++) {
      state[x][y] = newState[x][y];
    }
  }

//  // Interpolate between colors. Doesn't work super well and looks weird.
//  for (int i = 0; i < 100; i++) {
//    float alpha = float(i) / 100.0;
//    
//    for (int x = 0; x < 8; x++) {
//      for (int y = 0; y < 8; y++) {
//        uint8_t (* newColor)[3];
//        if (newState[x][y]) {
//          newColor = &color1;
//        } else {
//          newColor = &color2;
//        }
//
//        uint8_t (* oldColor)[3];
//        if (state[x][y]) {
//          oldColor = &color1;
//        } else {
//          oldColor = &color2;
//        }
//        
//        uint16_t color = matrix.Color(
//          uint8_t((1 - alpha) * *oldColor[0] + alpha * *newColor[0]),
//          uint8_t((1 - alpha) * *oldColor[1] + alpha * *newColor[1]),
//          uint8_t((1 - alpha) * *oldColor[2] + alpha * *newColor[2])
//         );
//        matrix.drawPixel(x, y, color);
//      }
//    }
//    matrix.show();
//    //delay(500);
//  }
}

void updateMatrix(int brightness) {
  for (int x = 0; x < 8; x++) {
    for (int y = 0; y < 8; y++) {
      uint8_t code = getNeighbors(x, y);
      bool value = state[x][y];
      uint16_t color;
      if (value) {
        color = matrix.Color(color1[0], color1[1], color1[2]);
      } else {
        color = matrix.Color(color2[0], color2[1], color2[2]);
      }
      matrix.drawPixel(x, y, color);
    }
  }
  matrix.setBrightness(brightness);
  matrix.show();
}

int getBrightness() {
  float value1 = rgbToValue(color1);
  float value2 = rgbToValue(color2);
  float value = (value1 + value2) / 2.0;
  //Serial.println(value);
  // TODO: maybe do something with the value?

  // Measure brightness.
  matrix.setBrightness(0);
  matrix.show();
  BH1750.start();
  float lux = BH1750.getLux();
  matrix.setBrightness(brightness);
  matrix.show();
  Serial.println(lux);

  int b;
  if (lux < BRIGHTNESS_THRESHOLD_OFF) {
    b = BRIGHTNESS_OFF;
  } else {
    b = BRIGHTNESS_ON;
  }
  return b;
}

float rgbToValue(uint8_t *c) {
  // https://math.stackexchange.com/questions/556341/rgb-to-hsv-color-conversion-algorithm
  float r = float(c[0]) / 255.0;
  float g = float(c[1]) / 255.0;
  float b = float(c[2]) / 255.0;
  float Cmax = max(r, g);
  Cmax = max(Cmax, b);
  return Cmax;
}

void setup() {
  Serial.begin(9600);

  Entropy.initialize();
  int seed = Entropy.random();
  randomSeed(seed);
  Serial.print("random seed: ");
  Serial.println(seed);
  
  matrix.begin();
  matrix.setBrightness(0);

  bool avail = BH1750.begin(BH1750_TO_GROUND);
  if (!avail) {
    Serial.println("BH1750 not available!");
  }
  BH1750.calibrateTiming();
  BH1750.start();
  BH1750.setQuality(BH1750_QUALITY_LOW);

  resetRules();
  resetState();
  resetColors();
}

void loop() {
  if (millis() - lastStateReset >= STATE_RESET_INTERVAL) {
    Serial.println("Resetting state");
    resetState();
    resetRules();
    resetColors();
    lastStateReset = millis();
  }
  
  if (millis() - lastStateUpdate >= STATE_UPDATE_INTERVAL) {
    Serial.println("Updating state");
    updateState();
    lastStateUpdate = millis();
  }

  if (brightness < 0.0 or millis() - lastBrightnessUpdate >= BRIGHTNESS_UPDATE_INTERVAL) {
    brightness = getBrightness();
    Serial.print("Updating brightness to ");
    Serial.println(brightness);
    lastBrightnessUpdate = millis();
  }
  updateMatrix(brightness);
}
