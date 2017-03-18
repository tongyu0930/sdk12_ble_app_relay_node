#ifndef __RELAY_H__
#define __RELAY_H__

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"


void init_storage(void);

void get_adv_data(ble_evt_t * p_ble_evt);

void manual_init(void);

#endif

