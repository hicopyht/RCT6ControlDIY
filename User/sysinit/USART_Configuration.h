#ifndef _USART_Configuration_H
#define _USART_Configuration_H


extern const u32 baudrate_table[8];


void USART1_Configuration(void);
void USART1_SendData(uint16_t Data);
U8 UART1_CAL(void);
U8 InU1FrameBuf(void);
U8 OutU1FrameBuf(void);
U8 InU1SendBuf(void);
void U1Printf(void);

void inBaseBuf( U8 byte );
U8 outBaseBuf( U8* byte );
void calBaseBuf(void);
void SendEncoders( U16 left, U16 right);
void SendEncodersAndGyro( U16 left, U16 right, S16 gyro );
void SendEncodersAndMpu6000(U16 left, U16 right, S16 gx, S16 gy, S16 gz, S16 ax, S16 ay, S16 az);
void SendBatteryState( U16 voltage_adc );
void SendBreakerStatus( U8 b0, U8 b1);
void SendParam( float value );
void ReceiveBaseCommand( U8 byte );
U8 calBaseCmd(void);
void simulateEncoder( U16* left, U16 * right );

void USART2_Configuration(void);
void USART2_SendData(uint16_t Data);
U8 UART2_CAL(void);
U8 InU2FrameBuf(void);
U8 OutU2FrameBuf(void);
U8 InU2SendBuf(void);
void U2Printf(void);
void U2PrintfDMA(void);


void USART3_Configuration(void);
void USART3_SendData(uint16_t Data);
U8 UART3_CAL(void);


#endif /* __USART_Configuration_H */

