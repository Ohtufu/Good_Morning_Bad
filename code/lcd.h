//
//================================================
//  파일명 : lcd.h - LCD제어 모듈의 헤더파일
//================================================
//

#ifndef __LCD_H__
#define __LCD_H__

// LCD 제어 명령

#define ALLCLR			0x01	// 화면을 지운다.
#define HOME			0x02	// 커서를 홈으로 보낸다.
#define LN21			0xc0	// 커서를 2번째 라인의 첫번째에 위치시킴
#define ENTMOD			0x06	// entry mode
#define FUNSET			0x28	// function set   
#define DISP_ON			0x0c	// 디스플레이를 켠다.
#define DISP_OFF		0x08	// 디스플레이를 끈다.
#define CURSOR_ON		0x0e	// 커서를 켠다.
#define CURSOR_OFF		0x0c	// 커서를 끈다.
#define CURSOR_LSHIFT  	0x10	// 커서를 왼쪽을 이동시킨다.
#define CURSOR_RSHIFT	0x14	// 커서를 오른쪽으로 이동시킨다.
#define DISP_LSHIFT		0x18	// 디스플레이를 왼쪽으로 이동시킨다.
#define DISP_RSHIFT		0x1c	// 디스플레이를 오른쪽으로 이동시킨다.

// LCD 제어모듈 함수

void LcdInit(void);			
void LcdCommand(char command);
void LcdMove(char line, char pos);
void LcdPutchar(char ch);
void LcdPuts(char* str);
void LcdNewchar(char ch, char font[]);


#endif	// __LCD_H__
