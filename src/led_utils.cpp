#include "hamal_esp.controller.h"

void warningMode()  {
  bool colors[3] = {1,0,0};
  fadeInOut(20, 84, 3, colors, 1, 2);
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
  leds[Pixel].r = red;
  leds[Pixel].g = green;
  leds[Pixel].b = blue;
}

void setAll(byte red, byte green, byte blue) {
  for (int i = 0; i < NUM_LEDS; i+=2) {
    setPixel(i, red, green, blue);
  }
  showStrip();
}

void setRight(byte red, byte green, byte blue) {
  for (int i = 0; i < NUM_LEDS/2; i+=2) {
    setPixel(i, red, green, blue);
  }
  for (int i = NUM_LEDS/2; i < NUM_LEDS; i+=2) {
    setPixel(i, 0, 120, 120);
  }
  showStrip();
}

void setLeft(byte red, byte green, byte blue) {
  for (int i = NUM_LEDS/2; i < NUM_LEDS; i+=2) {
    setPixel(i, red, green, blue);
  }
  for (int i = 0; i < NUM_LEDS/2; i+=2) {
    setPixel(i, 0, 120, 120);
  }
  showStrip();
}

void showStrip() {
  FastLED.show();
}

void lights_off(){
  for(uint16_t i = 0; i < NUM_LEDS; i++) {
    setPixel(i, 0, 0, 0);
  }
  showStrip();
}

