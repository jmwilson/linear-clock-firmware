#ifndef __TLC5926_H
#define __TLC5926_H

#ifdef __cplusplus
extern "C" {
#endif

void TLC592x_Switch_To_Special_Mode(void);
void TLC592x_Switch_To_Normal_Mode(void);
void TLC592x_Set_Brightness(uint16_t level);

#ifdef __cplusplus
}
#endif
#endif // __TLC5926_H
