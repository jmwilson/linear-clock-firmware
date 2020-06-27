#ifndef __TLC5926_H
#define __TLC5926_H

#include <cstdint>

typedef enum {
    TLC592x_STATE_READY,
    TLC592x_STATE_BUSY
} TLC592x_State;

typedef enum {
    TLC592x_OP_SHIFT_OUT,
    TLC592x_OP_SWITCH_TO_SPECIAL,
    TLC592x_OP_SWITCH_TO_NORMAL
} TLC592x_Op;

class TLC592x {
public:
    TLC592x(void);

    void switchToSpecialMode(void);
    void switchToNormalMode(void);

    void shiftOut(uint8_t *buffer, uint16_t size_in_bits);

    void enablePWM(uint16_t count);
    void disablePWM(void);

    TLC592x_State state;
    TLC592x_Op    op;
    uint16_t      txSize;
    uint16_t      txCount;
    uint8_t      *txBuffer;
};

#endif // __TLC5926_H
