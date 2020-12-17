#include "Arduino.h"
#include "neopixel.h"

/**
* @brief fill Neopixel ring with solid colour.
* @param colour Choose colour, enumerated in led msg.
* @param brightness Set the brightness of the strip.
* @param strip Neopixel strip object to light up.
* @return Name of colour set.
*/
const char* ring_colour(int8_t colour, uint8_t brightness, Adafruit_NeoPixel* strip) {
  switch (colour) {
    case pipebot_msgs__msg__Leds__COLOUR_WHITE:
        strip->fill(strip->Color(0, 0, 0, strip->gamma8(brightness)));
        strip->show();
        return "White";
      break;
    case pipebot_msgs__msg__Leds__COLOUR_RED:
        strip->fill(strip->Color(strip->gamma8(brightness), 0, 0, 0));
        strip->show();
        return "Red";
        break;
    case pipebot_msgs__msg__Leds__COLOUR_GREEN:
        strip->fill(strip->Color(0, strip->gamma8(brightness), 0, 0));
        strip->show();
        return "Green";
      break;
    case pipebot_msgs__msg__Leds__COLOUR_BLUE:
        strip->fill(strip->Color(0, 0, strip->gamma8(brightness), 0));
        strip->show();
        return "Blue";
      break;
    case pipebot_msgs__msg__Leds__COLOUR_AQUA:
        strip->fill(strip->Color(0, strip->gamma8(brightness),
                    strip->gamma8(brightness), 0));
        strip->show();
        return "Aqua";
      break;
    case pipebot_msgs__msg__Leds__COLOUR_YELLOW:
        strip->fill(strip->Color(strip->gamma8(brightness),
                    strip->gamma8(brightness), 0, 0));
        strip->show();
        return "Yellow";
      break;
    case pipebot_msgs__msg__Leds__COLOUR_FUCHSIA:
        strip->fill(strip->Color(strip->gamma8(brightness), 0,
                    strip->gamma8(brightness), 0));
        strip->show();
        return "Fuchsia";
      break;
    default:
        // Error no matching case
        strip->fill(strip->Color(30, 20, 0, 0));
        strip->show();
        return "Error";
      break;
  }
}

/** @brief Led light pattern. Lit pixel rotates around ring, every revolution
* more leds are turned on until the whole ring is lit.
* @param speed_ms Time before roating to next pixel on ring.
* @param color LED colour.
* @param loops Number of times to spin around. Choose same as LEDs on strip to
* fill by end of function.
* @param strip Neopixel strip object to use.
*/
void loading_chase(unsigned int speed, uint32_t color, int loops,
                    Adafruit_NeoPixel strip) {
  int      length        = 1;
  int      head          = length - 1;
  int      tail          = 0;
  int      loopNum       = 0;
  uint32_t lastTime      = millis();

  for (;;) {  // Repeat forever (or until a 'break' or 'return')
    for (int i=0; i < strip.numPixels(); i++) {  // For each pixel in strip...
      if (((i >= tail) && (i <= head)) ||      //  If between head & tail...
         ((tail > head) && ((i >= tail) || (i <= head)))) {
        strip.setPixelColor(i, color);  // Set colour
      } else {                                             // else off
        strip.setPixelColor(i, strip.gamma8(strip.Color(0, 0, 0, 0)));
      }
    }

    strip.show();  // Update strip with new contents
    // There's no delay here, it just runs full-tilt until the timer and
    // counter combination below runs out.

    if ((millis() - lastTime) > speed) {  // Time to update head/tail?
      if (++head >= strip.numPixels()) {      // Advance head, wrap around
        if (++loopNum >= loops) return;
        head = 0;
      }
      if (++tail >= strip.numPixels()) {      // Advance tail, wrap around
        tail = 0;
        head = ++head;
      }
      lastTime = millis();                   // Save time of last movement
    }
  }
}

/** @brief Led light pattern. Lit pixel rotates around ring, every revolution
* more leds are turned on until the whole ring is lit.
* @param speed_ms Time before roating to next pixel on ring.
* @param color LED colour.
* @param loops Number of times to spin around. Choose same as LEDs on strip to
* fill by end of function.
* @param strip Neopixel strip object to use.
* @param strip2 Neoplixel strip object to use.
*/
void loading_chase_double(unsigned int speed_ms, uint32_t color, int loops,
                            Adafruit_NeoPixel strip, Adafruit_NeoPixel strip2) {
  int      length        = 1;
  int      head          = length - 1;
  int      tail          = 0;
  int      loopNum       = 0;
  uint32_t lastTime      = millis();

  for (;;) {  // Repeat forever (or until a 'break' or 'return')
    for (int i=0; i < strip.numPixels(); i++) {  // For each pixel in strip...
      if (((i >= tail) && (i <= head)) ||      //  If between head & tail...
         ((tail > head) && ((i >= tail) || (i <= head)))) {
        strip.setPixelColor(i, color);  // Set colour
        strip2.setPixelColor(i, color);  // Set colour
      } else {                                             // else off
        strip.setPixelColor(i, strip.gamma8(strip.Color(0, 0, 0, 0)));
        strip2.setPixelColor(i, strip.gamma8(strip.Color(0, 0, 0, 0)));
      }
    }

    strip.show();  // Update strip with new contents
    strip2.show();
    // There's no delay here, it just runs full-tilt until the timer and
    // counter combination below runs out.

    if ((millis() - lastTime) > speed_ms) {  // Time to update head/tail?
      if (++head >= strip.numPixels()) {  // Advance head, wrap around
        if (++loopNum >= loops) return;
        head = 0;
      }
      if (++tail >= strip.numPixels()) {  // Advance tail, wrap around
        tail = 0;
        head = ++head;
      }
      lastTime = millis();  // Save time of last movement
    }
  }
}
