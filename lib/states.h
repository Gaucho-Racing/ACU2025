#ifndef STATES_H
#define STATES_H
#include "bcc_mc33771c.h"
#include "bcc_c.h"
#include "bcc_com_c.h"

void shutdown(); 
void standby();
void precharge();
void charge();
void normal();

#endif