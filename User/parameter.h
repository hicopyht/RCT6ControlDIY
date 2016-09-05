#ifndef _Parameter_H
#define _Parameter_H

#define FLASH_PAGE_SIZE         	(0x800)	   			/* 2048 */
#define FLASH_PAGE_WORD_SIZE  (0x200)				/*512*/
#define DATA_BUFFER_SIZE   		FLASH_PAGE_SIZE  	
#define FLASH_SIZE            		(0x40000) 			/*256K */
//#define PARAM_START_ADDRESS    (0x08000000+FLASH_SIZE - 8*FLASH_PAGE_SIZE) /* 120th page, 128 pages, 2k/page, */
#define PARAM_START_ADDRESS	(0x0803E000)		/* 124th page*/


typedef union 
{
	U32 word;
	U16 halfworld[2];
	U8 bytes[4];
	float f;
} ParamValue;

struct ParamData
{
	const char *name;
	ParamValue value;
};

#define PARAM_NUM	14
extern const char * param_names[PARAM_NUM];
extern const float default_values[PARAM_NUM] ;
extern struct ParamData param_vector[PARAM_NUM];

void paramsInit(void);
void paramsInfoPrint(void);
void paramsLoad(void);
void paramsSave(void);
void paramsReset( U8 save );
U32 paramFind( const char* name );
U8 paramValidName( const char* name );
U8 paramValidIndex( U8 index );
U8 paramSetFloat( U8 index, float value );
U8 paramGetFloat( U8 index, float* value );
U8 paramSetBytes4(U8 index, U8* bytes4 );
U8 paramGetBytes4(U8 index, U8* bytes4);
//
bool paramWriteValue( const char* name, ParamValue value );
bool paramWriteLong( const char* name, U32 data );
bool paramWriteFloat(  const char* name, float data );
//
bool paramReadValue( const char* name, ParamValue* value );
bool paramReadLong( const char* name, U32* data );
bool paramReadFloat( const char* name, float* data );
float paramReadFloatWithDefault( const char* name, float default_value );
//
void FlashWritePage(  U32 address, U8 * data  );
void FlashReadPage( U32 address, U8 * data );
bool FlashWriteWord( U32 address, U32 data);
U32 FlashReadWord( U32 address );


//void testParamWord(void);
//void testParamPage(void);

#endif
