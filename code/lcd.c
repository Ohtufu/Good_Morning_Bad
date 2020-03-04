//=======================================================
//  LCD.C : LCD 구동함수의 모음
//=======================================================

#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

// LCD 포트 주소
#define LCD_PORT	PORTC
#define LCD_DDR		DDRC
// 내부 함수 		
static void checkbusy(void);
static void write_command(char command);
static void write_data(char ch);

//===================================
// 기능 : LCD Display를 초기화한다.
//==================================

void LcdInit(void)
{
	LCD_DDR	= 0xFF;			// LCD포트를 출력으로 설정
	_delay_ms(15);
	write_command(0x30);
	_delay_ms(5);
	write_command(0x30);
	_delay_ms(1);
	write_command(0x32);

	LcdCommand(FUNSET);
	LcdCommand(DISP_OFF);
	LcdCommand(ALLCLR);
	LcdCommand(ENTMOD);

	LcdCommand(DISP_ON);		// 화면을 켠다.
}

//======================================================
// LCD에 명령을 출력하는 함수 
//
//	입력 : command - LCD에 내리는 명령 
//	  		      lcd.h에 정의된 명령을 사용할 것
//
//======================================================

void LcdCommand(char command)
{ 
	checkbusy();
	write_command(command);
	if(command == ALLCLR || command == HOME)
		_delay_ms(2);
}

//======================================================
// 현재위치에 문자 하나를 출력한다.  
//
//	입력 : ch - 화면에 쓸 문자 코드
//
//======================================================

void LcdPutchar(char ch)
{
	checkbusy();
	write_data(ch);
}

//======================================================
// 현재위치에 문자열을 출력한다.  
//
//	입력 : str - 출력할 문자열
//
//======================================================

void LcdPuts(char* str)
{
	while(*str)  			// *str이 NULL 문자가 아니면 루프를 돈다.
	{
		LcdPutchar(*str);	// 문자 *str을 화면에 출력
		str++;				// str이 다음 문자를 가리킴
	}
}

//=======================================================
// 글자를 쓸 위치를 지정된 위치(line, pos)로 이동시킨다.
// 입력 : 	line - 화면의 행(0행부터 시작)
//			pos  - 화면의 열(0열부터 시작)
//=======================================================
void LcdMove(char line, char pos)
{
	pos = (line << 6) + pos; 
  	pos |= 0x80;			// 비트 7를 세트한다.

  	LcdCommand(pos);
}

//=======================================================
// 명령 레지스터에 명령을 쓴다.
// 입력 : command - LCD에 내리는 명령 코드
//=======================================================

static void write_command(char command)
{ 
	char temp;
 
	//
	// 상위 니블 출력
	//
  	temp = (command & 0xF0)|0x04;	//0x04 : RS=0(명령)
									// RW=0(쓰기),E=1
  	LCD_PORT = temp;
  	LCD_PORT = temp & ~0x04;		// E = 0

  	//
  	// 하위 니블 출력
  	//
  	temp = (command << 4) | 0x04;	// 0x04 :RS=0(명령)
									// RW=0(쓰기),E=1
  	LCD_PORT = temp;	
	LCD_PORT = temp & ~0x04;		// E = 0
	_delay_us(1);
}

//========================================
// 데이터 레지스터에 명령을 쓴다.
// 입력 : ch - LCD에 쓸 데이터 
//========================================

static void write_data(char ch)
{
	unsigned char temp;

	// 상위 니블 출력
	//
  	temp = (ch & 0xF0)|0x05;		// 0x05:RS=1(데이터)
									// RW=0(쓰기),E=1
	LCD_PORT = temp;
	LCD_PORT = temp & ~0x04; 		// E = 0


	// 하위 니블 출력
	//
	temp = (ch << 4) | 0x05;		// 0x05:RS=1(데이터)
									// RW=0(쓰기),E=1
	LCD_PORT = temp;
	LCD_PORT = temp & ~0x04;		// E = 0
}


//========================================
// 문자코드 ch에 새로운 글꼴을 등록한다.
// 입력 : ch	- 문자코드
//	      font	- 글꼴 데이터
//========================================

void LcdNewchar(char ch, char font[])	// 글자 등록함수
{
	int i;
		
	ch <<= 3;			// ch = ch << 3;과 같음
	ch |= 0x40;		// 비트6을 세트 => CGRAM 주소설정

	LcdCommand(ch);	// CGRAM 주소설정 =>LcdPutchar()로 
					// 쓰는 문자는 CGRAM에 저장

	for(i=0; i<8; i++)	// 글꼴을 CGRAM에 저장한다.
		LcdPutchar(font[i]);
}


//==================================================
// 50usec의 시간지연으로 BF 플래그 검사를 대체
//==================================================

static void checkbusy()
{
  	_delay_us(10); 
  	_delay_us(10); 
  	_delay_us(10); 
  	_delay_us(10); 
  	_delay_us(10); 
}
