
typedef union{
	float f;
	U8 bytes[4];
} FLOAT4;

void delayus( vu32 count );
void delayms(vu32 nCount);
void delayms2(u32 nTime);
u16 GetPower(u16 advalue);
u16 GetTemp(u16 advalue);
int fputc(int ch, FILE *f);
u16 average(u16 *ADdata, u8 Num, u8 ADi);
u16 average2(u16 *ADdata, u8 Num, u8 ADi);
void U16ToChar(uint16 data, char * pk_ch);
void U8ToChar(uint8 data, char * pk_ch);
float s16s8toFloat( S16 number, S8 evalue);
void floatToBytes4( float *f, U8* b0, U8* b1, U8* b2, U8* b3 );
float bytes4ToFloat( U8 b0, U8 b1, U8 b2, U8 b3);



//不同类型数据转换
#define    U8toU16(x, y) ((U16)(((U16)(y))<<8|((U16)(x))))
#define    U8toS16(x, y) ((S16)(((U16)(y))<<8|((U16)(x))))

#define    U8toU32(x1, x2,x3, x4)   ((U32)(((U32)(x4))<<24|((U32)(x3))<<16|((U32)(x2))<<8|((U16)(x1))))
#define    U8toS32(x1, x2,x3, x4)   ((S32)(((U32)(x4))<<24|((U32)(x3))<<16|((U32)(x2))<<8|((U16)(x1))))

#define    U16toU8(x)    (U8*)(&(x))
#define    S16toU8(x)    (U8*)(&(x))

#define limitMinMax( x, min, max) {if(x>max) x= max; if(x<min) x = min;}
