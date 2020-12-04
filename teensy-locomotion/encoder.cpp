#include "Arduino.h"
#include "encoder.h"


/*
* @brief Setup encoder object.
* @param encoder_name Name for use in diagnostic messages.
* @param pin_A Digital Pin A - Interrupt attached to this one.
* @param pin_B Digital Pin B.
* @param counts_per_revolution Number of encoder ticks per turn of the robot's wheel.
* @param id Hardware Id or serial number for use in diagnostic messages.
* @param inverse Swap the direction which counts up, to account for motors mounted in different ways.
* @return true if successful
*/
Encoder::Encoder(const char *encoder_name,
                  const unsigned int pin_A,
                  const unsigned int pin_B,
                  const unsigned int counts_per_revolution,
                  const char *id,
                  const bool inverse) {
  name = encoder_name;
  pin_A_ = pin_A;
  pin_B_ = pin_B;
  counts_per_revolution_ = counts_per_revolution;
  hardware_id = id;
  this->inverse = inverse;
}


/*
* @brief Setup encoder object.
* @param encoder_name Name for use in diagnostic messages.
* @param pin_A Digital Pin A - Interrupt attached to this one.
* @param pin_B Digital Pin B.
* @param counts_per_revolution Number of encoder ticks per turn of the robot's wheel.
* @param id Optional, default = "". Hardware Id or serial numer for use in diagnostic messages.
* @param inverse Optional, default = false. Swap the direction which counts up, to account for motors mounted in different ways.
* @return true if successful
*/
bool Encoder::setup() {
  int success = true;
  pinMode(pin_A_, INPUT);
  pinMode(pin_B_, INPUT);
  digitalWrite(pin_A_, HIGH);  // turn on pullup resistor
  digitalWrite(pin_B_, HIGH);


  switch (inst_counter) {
    case 0:

      instances[0] = this;
      if (inverse) {
        attachInterrupt(digitalPinToInterrupt(pin_A_), count_glue_0_inv, RISING);
      } else {
        attachInterrupt(digitalPinToInterrupt(pin_A_), count_glue_0, RISING);
      }
      break;

    case 1:
      instances[1] = this;
      if (inverse) {
        attachInterrupt(digitalPinToInterrupt(pin_A_), count_glue_1_inv, RISING);
      } else {
        attachInterrupt(digitalPinToInterrupt(pin_A_), count_glue_1, RISING);
      }
      break;

    default:
      // problem, more instances than we can cope with. (In this case add
      // more cases to switch above and create the required 'glue' functions.)
      success = false;
    }
  inst_counter++;
  return success;
}

void Encoder::count_glue_0() {
  if (Encoder::instances[0] != NULL)
    Encoder::instances[0]->count();
}
void Encoder::count_glue_0_inv() {
  if (Encoder::instances[0] != NULL)
    Encoder::instances[0]->count_inverse();
}
void Encoder::count_glue_1() {
  if (Encoder::instances[1] != NULL)
    Encoder::instances[1]->count();
}
void Encoder::count_glue_1_inv() {
  if (Encoder::instances[1] != NULL)
    Encoder::instances[1]->count_inverse();
}

/*
* @brief Interrupt callback for encoder. In/decerements ticks
* and checks if a revolution is completed
*/
void Encoder::count() {
  if (digitalRead(pin_A_) == digitalRead(pin_B_)) {
    wheel_position++;
    ticks++;
    if (wheel_position > counts_per_revolution_) {
      wheel_position = 0;
      total_revolutions++;
    }
  } else {
    wheel_position--;
    ticks--;
    if (wheel_position < -counts_per_revolution_) {
      wheel_position = 0;
      total_revolutions--;
    }
  }
}

/*
* @brief Interrupt callback for encoder. In/decerements ticks
* and checks if a revolution is completed.
* Inversed for motors mounted in opposite direction.
*/
void Encoder::count_inverse() {
  if (digitalRead(pin_A_) == digitalRead(pin_B_)) {
    wheel_position--;
    ticks--;
    if (wheel_position < -counts_per_revolution_) {
      wheel_position = 0;
      total_revolutions--;
    }
  } else {
    wheel_position++;
    ticks++;
    if (wheel_position > counts_per_revolution_) {
      wheel_position = 0;
      total_revolutions++;
    }
  }
}

// initialise variables
int Encoder::inst_counter = 0;
Encoder * Encoder::instances[2] = { NULL, NULL };
