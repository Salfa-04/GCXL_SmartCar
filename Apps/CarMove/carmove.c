#include "carmove.h"

#include "chassis.h"

static inline void carmove_setoff(void) {}

void carmove(CarStates states) {
  if (states == CarMoveOK)
    return;
  else if (states == CarSetOff)
    carmove_setoff();
}
