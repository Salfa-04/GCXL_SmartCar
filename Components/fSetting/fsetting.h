#ifndef __FSETTING_H
#define __FSETTING_H

#include "stm32f4xx_hal.h"

extern uint8_t fSettings[128];
void fset_save(uint8_t *data, int8_t len);
void fset_read(uint8_t **ptr);

#endif /* __FSETTING_H */
