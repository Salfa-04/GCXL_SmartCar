#ifndef __CAR_MOVE_H
#define __CAR_MOVE_H

typedef enum CarMove {
  CarMoveOK = 0,

  CarSetOff,
  CarGoHome,
  CarBackward,
} CarStates;

void carmove(CarStates states);

#endif /* __CAR_MOVE_H */
