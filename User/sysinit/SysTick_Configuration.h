#ifndef _SysTick_Configuration_H
#define _SysTick_Configuration_H

#define SysTick_Counter_Disable        ((u32)0xFFFFFFFE)
#define SysTick_Counter_Enable         ((u32)0x00000001)
#define SysTick_Counter_Clear          ((u32)0x00000000)
#define CTRL_TICKINT_Set      ((u32)0x00000002)
#define CTRL_TICKINT_Reset    ((u32)0xFFFFFFFD)
void SysTick_Configuration(void);
#endif /* __SysTick_Configuration_H */
