#include "Arduino.h"
#include "errors.h"
#include "config.h"



/**
* @brief Error loop for when communication is lost. R
* Flashes on/off for 5 seconds, then resets board.
*/
void coms_error(Motor& left, Motor& right) {
  // stop motors
  left.move_fwd(0);
  right.move_fwd(0);

  // no comms so unlikely this will be sent...
//  snprintf(status.message.data, status.message.capacity,
//           "Error: Communication with agent lost");
//  status.level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;

  // flash onboard LED
  for (int i = 0; i <=50; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  // restart teensy to try to reconnect
  SCB_AIRCR = 0x05FA0004;
}
