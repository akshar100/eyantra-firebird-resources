#include  <lpc214x.h>

/*************Macros*******************/
#define DATA_PORT() IO1SET=(1<<19)	
#define READ_DATA() IO1SET=(1<<18)
#define EN_HI() IO1SET=(1<<17)

#define COMMAND_PORT() IO1CLR=(1<<19)	
#define WRITE_DATA() IO1CLR=(1<<18)
#define EN_LOW() IO1CLR=(1<<17)

/**************************************/

/***********Prototypes****************/

void Init_LCD_Pin(void);
void DelaymSec(unsigned int j);
void LCD_Command(unsigned int data);
void LCD_4Bit_Mode(void);
void LCD_Init(void);
void LCD_String(unsigned char *data);
void LCD_Home(void);
void LCD_Cursor(unsigned char Row,unsigned char Col);
void LCD_Print(unsigned char Row, char Col,unsigned int Val, unsigned int Digits);
void Init_Peripherals(void);
void Init_Ports(void);
/**************************************/

