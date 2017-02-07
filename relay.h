#ifndef __RELAY_H__
#define __RELAY_H__

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"




typedef struct
{
    uint8_t          bit0;
    uint8_t          bit1;
    uint8_t          bit2;
    uint8_t          bit3;
    uint8_t          bit4;
    uint8_t          bit5;
    uint8_t          bit6;
    uint8_t          bit7;
} byte;





void get_adv_data(ble_evt_t * p_ble_evt);



#endif
