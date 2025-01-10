#ifndef DEBUG_H
#define DEBUG_H

#include "Battery.h"
#include "config.h"
#include <Arduino.h>

int voltage_measurement_test(Battery *bty);
int temperature_measurement_test(Battery *bty);
int debug(Battery *bty);
void print_bcc_status(bcc_status_t bccStatus);

#endif