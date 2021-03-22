/*
* MIT License
*
* Copyright (c) 2021 University of Leeds
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#include "Arduino.h"
#include "diagnostics.h"

/**
* @brief Create diagnostic status message and fill with initial values.
* @param status Pointer to diagnostic status.
* @param name Name of the component the status refers to.
* @param message Initial status message
* @param hardwareID ID number for the hardware component
* @param level Error level, Ok/Warn/Error/Stale
* @return pointer to status
*/
diagnostic_msgs__msg__DiagnosticStatus* create_diagnostic_status(
  diagnostic_msgs__msg__DiagnosticStatus *status,
  const char *name,
  const char *message,
  const char *hardwareID,
  uint8_t level) {
  status = diagnostic_msgs__msg__DiagnosticStatus__create();
  status->level = level;

  int name_length = strlen(name) + 1;
  status->name.capacity = name_length;
  status->name.data = (char*)malloc(name_length*sizeof(char));
  snprintf(status->name.data, status->name.capacity, name);
  status->name.size = strlen(status->name.data);

  int msg_length = strlen(message) + 1;
  status->message.data = (char*)malloc(50*sizeof(char));
  status->message.capacity = msg_length;
  snprintf(status->message.data, status->message.capacity, message);
  status->message.size = strlen(status->message.data);

  int id_length = strlen(hardwareID) + 1;
  status->hardware_id.capacity = id_length;
  status->hardware_id.data = (char*)malloc(id_length*sizeof(char));
  snprintf(status->hardware_id.data, status->hardware_id.capacity, hardwareID);
  status->hardware_id.size = strlen(status->hardware_id.data);

  return status;
}

/**
* @brief Update diagnostic status message and level
* @param status Pointer to diagnostic status.
* @param message Status message
* @param level Error level, Ok/Warn/Error/Stale
* @return pointer to status
*/
diagnostic_msgs__msg__DiagnosticStatus* update_diagnostic_status(
  diagnostic_msgs__msg__DiagnosticStatus *status,
  const char *message,
  uint8_t level) {
  int msg_length = strlen(message) + 1;
  status->message.capacity = msg_length;
  snprintf(status->message.data, status->message.capacity, message);
  status->message.size = strlen(status->message.data);

  status->level = level;

  return status;
}

/**
* @brief Create diagnostic key value message and fill with initial values.
* @param status Pointer to diagnostic key value pair.
* @param Key Name of the key.
* @param value Initial value.
* @return pointer to key value pair.
*/
diagnostic_msgs__msg__KeyValue* create_diagnostic_KeyValue(
  diagnostic_msgs__msg__KeyValue *keyvalue,
  const char *key,
  const char *value) {
  // Init Key
  keyvalue = diagnostic_msgs__msg__KeyValue__create();
  int key_length = strlen(key) + 1;
  keyvalue->key.data = (char*)malloc(key_length*sizeof(char));
  keyvalue->key.capacity = key_length;
  // Use Key
  snprintf(keyvalue->key.data, keyvalue->key.capacity, key);
  keyvalue->key.size = strlen(keyvalue->key.data);
  // Init Value
  int value_length = strlen(value) + 1;
  keyvalue->value.data = (char*)malloc(20*sizeof(char));
  keyvalue->value.capacity = value_length;
  // Use Value
  snprintf(keyvalue->value.data, keyvalue->value.capacity, value);
  keyvalue->value.size = strlen(keyvalue->value.data);
  return keyvalue;
}

/**
* @brief Update diagnostic key value.
* @param status Pointer to diagnostic key value pair.
* @param value New value.
* @return pointer to key value pair.
*/
diagnostic_msgs__msg__KeyValue* update_diagnostic_KeyValue(
  diagnostic_msgs__msg__KeyValue *keyvalue,
  const char *value) {
  int value_length = strlen(value) + 1;
  keyvalue->value.capacity = value_length;
  snprintf(keyvalue->value.data, keyvalue->value.capacity, value);
  keyvalue->value.size = strlen(keyvalue->value.data);
  return keyvalue;
}

/**
* @brief Error loop for when communication is lost. R
* Flashes on/off for 5 seconds, then resets board.
*/
void coms_error(Motor *left, Motor *right, int led_pin) {
  // stop motors
  left->move_fwd(0);
  right->move_fwd(0);

  // flash onboard LED
  for (int i = 0; i <=50; i++) {
    digitalWrite(led_pin, !digitalRead(led_pin));
    delay(100);
  }
  // restart teensy to try to reconnect
  SCB_AIRCR = 0x05FA0004;
}
