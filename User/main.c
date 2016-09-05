#include "Configuration.h"


main()
{	
	U8 byte = 0;
	//
	InitDevice();
	printf("/***************************************/\r\n");
	printf("RCT6 Robot Control Board Simple Version, v1.0.\r\n");
	printf("/***************************************/\r\n");
	DebugPrintf();

	while(1)
	{

		// Process gyroscope and encoder measurement
		if( is_control_motor == TRUE )
		{
			Motor_Control();
			is_control_motor = FALSE;
		}
	
		if(ADC1_2_OK == TRUE)	//ADC1_2 采样结果更新，频率50Hz
		{
			ADC1_2_OK = FALSE;	//清除标志位
			InternalTemp = GetTemp(ADC_ConvertedValue[ADC_CHANNEL_TEMP]);	//根据ADC结果计算温度
			Power = GetPower(ADC_ConvertedValue[ADC_CHANNEL_1V2]);

			// Send battery state
			SendBatteryState( (U16)ADC_ConvertedValue[ADC_CHANNEL_10] );
			//
			SendBreakerStatus( PB8_VALUE(), PB9_VALUE() ); // HIGH: 0, LOW 1
			
			//printf("Voltage: %d \r\n", (U16)ADC_ConvertedValue[ADC_CHANNEL_10] );
			//DebugPrintf();
			/*
			printf("\r\n Temp:%d\t%d", ADC_ConvertedValue[ADC_CHANNEL_TEMP], GetTemp(ADC_ConvertedValue[ADC_CHANNEL_TEMP]) );
			printf(" 1V2 Ref: %d,\t%f", ADC_ConvertedValue[ADC_CHANNEL_1V2], ADC_ConvertedValue[ADC_CHANNEL_1V2] * 3.31 / 4095 );
			printf("\r\n 10 = %d,\t11 = %d", ADC_ConvertedValue[ADC_CHANNEL_10], ADC_ConvertedValue[ADC_CHANNEL_11]);
			DebugPrintf();
			*/
		}

		// Base command
		//calBaseBuf();
		if( outBaseBuf(&byte) )
			ReceiveBaseCommand(byte);
		
		// UART2接受字节
		if(IsU2RecOK == TRUE)
		{
			UART2_CAL();
			IsU2RecOK = FALSE;
		}

		// UART2发送帧
		if(IsU2Send == TRUE)
		{
			OutU2FrameBuf();
			IsU2Send = FALSE;
		}


	}				    
	
}

