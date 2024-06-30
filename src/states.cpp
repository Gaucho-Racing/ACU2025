#include "../lib/states.h"
#include "main.cpp"

void shutdown(){
    battery->cellBalancing(true, false);
    bcc_status_t result = BCC_Sleep(battery->drvConfig);
}