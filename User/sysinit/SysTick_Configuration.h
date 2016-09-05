#ifndef _SysTick_Configuration_H
#define _SysTick_Configuration_H

#define SysTick_Counter_Disable        ((u32)0xFFFFFFFE)
#define SysTick_Counter_Enable         ((u32)0x00000001)
#define SysTick_Counter_Clear          ((u32)0x00000000)
#define CTRL_TICKINT_Set      ((u32)0x00000002)
#define CTRL_TICKINT_Reset    ((u32)0xFFFFFFFD)

struct TIME_STAMP
{
	U32 sec;
	U32 usec;
};

void SysTick_Configuration(void);
void getTimeStamp( struct TIME_STAMP * stamp);
void deltaTimeStamp( struct TIME_STAMP* cur, struct TIME_STAMP* pre, struct TIME_STAMP* delta);
double deltaTimeStampDouble( struct TIME_STAMP* cur, struct TIME_STAMP* pre );
float deltaTimeStampFloat( struct TIME_STAMP* cur, struct TIME_STAMP* pre );
#define copyTimeStamp(from, to) {to.sec = from.sec; to.usec = from.usec;}

#endif /* __SysTick_Configuration_H */
