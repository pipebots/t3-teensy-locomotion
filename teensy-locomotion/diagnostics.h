#ifndef diagnostics_h
#define diagnostics_h

#include "Arduino.h"
#include "motor.h"
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>

diagnostic_msgs__msg__DiagnosticStatus* create_diagnostic_status(diagnostic_msgs__msg__DiagnosticStatus *,  const char *,  const char *,  const char *,  uint8_t level);

diagnostic_msgs__msg__DiagnosticStatus* update_diagnostic_status(diagnostic_msgs__msg__DiagnosticStatus *, const char *, uint8_t);

diagnostic_msgs__msg__KeyValue* create_diagnostic_KeyValue(diagnostic_msgs__msg__KeyValue *, const char *, const char *);

diagnostic_msgs__msg__KeyValue* update_diagnostic_KeyValue(diagnostic_msgs__msg__KeyValue *, const char *);

void coms_error(Motor *, Motor *);

#endif
