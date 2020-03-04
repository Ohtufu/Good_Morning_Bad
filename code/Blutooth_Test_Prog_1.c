
#include <avr/io.h>
#include <avr/interrupt.h> 
#include <util/delay.h>
#include "lcd.h"
#define  Avg_Num     4         //  이동 평균 갯수 
#define  Amp_Gain   11         //  증폭기 이득 

void init_serial(void) ;  //  Serial 토신포트 초기화
void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
void SerialPutChar(char ch);
void SerialPutString(char str[]);
void Display_Number_LCD( unsigned int num, unsigned char digit ) ;    // 부호없는 정수형 변수를 10진수 형태로 LCD 에 디스플레이 
void msec_delay(unsigned int n);
void usec_delay(unsigned int n);
void DC_Motor_Run_Fwd( short duty );    // DC 모터 정회전(PWM구동) 함수 
void DC_Motor_Run_Rev( short duty );    // DC 모터 역회전(PWM구동) 함수  
void DC_Motor_Stop( void );             // DC 모터 정지 함수  
void DC_Motor_PWM( short Vref );        // DC 모터 PWM 신호 발생 함수  
                                        // 정토크(Vref>0), 역토크(Vref<0), 영토크(Vref=0) 모두 포함 


unsigned char Time_Delay_Polling( unsigned short d_time ) ;   // 시간지연 체크함수(폴링방식

static volatile unsigned char cnumber[5] = {0, 0, 0, 0, 0}; 
static volatile unsigned short CDS_sensor_ouput= 0,  CDS_sensor_ouput_avg = 0 ; 
static volatile unsigned char int_num = 0 , mode = 0;
static volatile unsigned short    distance_1 = 0, sensor_count = 0  ;

static volatile unsigned short    distance_1_old = 0 ;

static volatile  unsigned char    Warning_Flag = 0 ;
static volatile  unsigned short   Delay_Time = 0;
static volatile short  Vmax = 0 ; 

static volatile  char  recv_cnt = 0, rdata=0, new_recv_flag = 0, rdata_old = 0 ;  

static volatile char Send_Message_1[] = { "Tx Success!" } ;                    

/********************************************************************************************************************
                                      					main
********************************************************************************************************************/
int main(void)
{ 
     short duty = 0;
   	// unsigned char rdata=0 ;  	  
     // Push Switch : 외부인터럽트 0 (INT0 : PD0 )에 연결   
	//스위치. 
	 DDRD &= ~0x02;     // PD0 (외부인터럽트 INT0 ) : 입력설정   
     PORTD |= 0x02;     // PD0 : 내부풀업사용   
 
	 DDRB |= 0x10; 	   // LED (PB0 ) :출력설정	

	 PORTB |= 0x10;    // LED OFF
	 
	DDRB |= 0x02;    // 3 초음파센서 Trigger signal( PA1   )
	PORTB &= ~0x02;   // PA1  : Low    
	
	DDRB |= 0x08;// 능동버저(BUzzer)(PA3)
	PORTB &= ~0x08;//PB1 = low(버저 off)

	DDRB |= 0x10;//LED(PA4)
	PORTB |= 0x10;//PA4 = high => led off high비트는 1이여서 1을 표현할때는 or을 사용한다.

     DDRB |= 0x20;   // 모터구동신호 + 단자:  PWM 포트( pin: OC1A(PB5) )   --> 출력 설정 
	 DDRA |= 0x01;   // 모터구동신호 - 단자 : 범용 입/출력포트(pin : PA0 ) --> 출력 설정 

	 init_serial() ;   // Serial Port (USART1) 초기화
	 
	 LcdInit();       // LCD 초기화 

     UCSR1B |=  0x80  ;      // UART1 송신(RX) 완료 인터럽트 허용
     sei() ;                 // 전역인터럽트허용
    ////////  외부 인터럽트(INT0 ) 설정  ///////////

    EICRA &= ~0x04;  // INT1 하강모서리에서 인터럽트 걸림
    EICRA |=  0x08;  // INT1 하강모서리에서 인터럽트 걸림

    EIMSK |=  0x02;  // INT1 인터럽트  허용

  ///////////////////////////////////////////////


     LcdCommand( ALLCLR ) ;    // LCD Clear
  	 LcdMove(0,0);    
	 LcdPuts("Blutooth Module"); 
     
	 /*****   AD Converter **********/

     ADMUX &= ~0xE0;    //  ADC 기준전압 = AREF ,   ADC 결과 오른쪽정렬 
     ADCSRA |= 0x87;     // ADC enable, Prescaler = 128
 
  	 LcdMove(1,0);    
	 LcdPuts("HC-06 Test Prog"); 
	 /**** Timer0 Overflow Interrupt  ******/
     /**************************************/
     TCCR0 = 0x00; 
     TCNT0 = 256 - 156;       //  내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                             //  오버플로인터럽트 주기 = 10msec
                             //  156 = 10msec/ 64use

     TIMSK = 0x01;  // Timer0 overflow interrupt enable 
	

// 3 Echo Signals Pulse Width(초음파센서 왕복시간 T측정) measurment,  Timer3 

	TCCR3A = 0x00;// 타이머용도.  
	TCCR3B = 0x02;     // 타이머 3 시작(분주비 8) ,  0.5usec 단위로 측정 

/////////////////////////////////////////////////////////
	
   // 초음파센서 Echo Signals : external interrupt 4( pin: INT4 (PE4)),  ////바꿀것 

	EICRB = 0x04;   //0000 0100 INT5 Both falling edge and rising edge interrupt
	EIMSK |= 0x20;   // 0010 0000INT5 Enable 
	
	
    //sei();         // Global Interrupt Enable 
	//cli():전역 인터럽트 허용 금지 

   ///////////////////////////////////////
   	//  최초 초음파센서 1 트리거 신호 발생(초음파 1 발사)  
	PORTB |= 0x02;    // PA1 : High
	usec_delay(20) ;  // 20usec 동안 High 유지 
	PORTB &= 0xFD;    // PA1 : Low 
  /////////////////////////////////////////////
   ////////////////////////////////////////////
    // 모터구동신호 ( pin: OC1A(PB5) ),   Timer1, PWM signal (period= 200 usec )

	TCCR1A = 0x82;    // OC1A(PB5)) :  PWM 포트 설정,   Fast PWM ( mode 14 )
	TCCR1B = 0x1b;    // 64 분주 타이머 1 시작 (내부클럭 주기 =  64/(16*10^6) = 4 usec ),  Fast PWM ( mode 14 ) 
	ICR1 = 50;       // PWM 주기 = 50 * 4 usec = 200 usec (  PWM 주파수 = 1/200usec = 5 kHz )

    Vmax = ICR1; 

	OCR1A = duty;      //  OC1A(PB5) PWM duty = 0 설정 : 모터 정지
   //////////////////////////////////////////////////////////////////
	 

	TCCR0 |= 0x07; // Clock Prescaler N=1024 (Timer 0 Start)
  
	 while(1)
	 {

         if( new_recv_flag == 1 )      // 1 문자 수신완료 시 
		 { 
  

     	//////////////  명령어 처리   //////////////

			if( rdata == '0' )          // 문자 0 이 수신되면 
			{
			   duty += 300;
			   if(duty >= Vmax) duty = Vmax; 
               SerialPutString("1:ON 2:RE 3:STOP.\n");  // 휴대폰으로 메시지 전송
                
			//	PORTB &= ~0x10;         // LED ON
			}
			else if( rdata == '1' )     // 문자 1 이 수신되면
			{
			    duty = 50;
                DC_Motor_Run_Fwd( duty );      // DC Motor 정회전
                msec_delay( 75 ); 
				DC_Motor_Stop(); 
				
			}
			else if( rdata == '2')      // 문자 2 가 수신되면
			{
                
		        DC_Motor_Run_Rev( duty );     // DC Motor 역회전 5초
				msec_delay( 80 );     // 회전이랑 역회전을 다르게 한이유는 약간 차이가 나서
				DC_Motor_Stop(); 
			
			} 
				else if( rdata == '3')      // 문자 3 가 수신되면
			{
                
		         DC_Motor_Stop();               // DC Motor 정지 
			
			} 
		

			else if( rdata != 0xFF)    //  명령 오류 이면
			{

                SerialPutString("Command Error!!  Try again.\n" ); //  명령 오류 메시지 전
			}


		    rdata = 0xFF;
            new_recv_flag = 0;      // 새 문자(명령) 수신 플래그 Reset
  

        }
    //////////////////////////////////////////////////

 
	}

} 

ISR(  USART1_RX_vect )
{

    rdata = UDR1; 
 
    SerialPutChar( rdata);           // Echo  수신된 데이터를 바로 송신하여 수신된 데이터가 정확한지 확인 
    SerialPutChar('\n');             // 휴대폰으로 데이터 전송시 Line Feed('\n')를 항상 끝에 전송해야함

    recv_cnt++ ;                     // 수신된 데이터 바이트수 저장

    new_recv_flag = 1;               // 새 문자(명령) 수신 플래그 Set

}

void init_serial(void)
{
    UCSR1A=0x00;                    //초기화
    UCSR1B=0x18;                    //송수신허용,버퍼인터럽트 금지
    UCSR1C=0x06;                    //데이터 전송비트 수 8비트로 설정.
    
    UBRR1H=0x00;
    UBRR1L=103;                     //Baud Rate 9600 
}
ISR(TIMER0_OVF_vect)   // Timer0 overflow interrupt( 10 msec)  service routine
{

    static unsigned short  time_index = 0,  time_index2 = 0,count1 = 0,CDS_Sum = 0 ; 
    static unsigned short  CDS_sensor_ouput_buf[Avg_Num ]   ; 
    unsigned char i = 0 ;


    TCNT0 = 256 - 156;       //  내부클럭주기 = 1024/ (16x10^6) = 64 usec,  
                             //  오버플로인터럽트 주기 = 10msec
                             //  156 = 10msec/ 64usec

     time_index++ ; 
	 time_index2++ ; 


    if( time_index == 25 )    // 샘플링주기 =  250 msec = 10msec x 25 
    {

       time_index = 0; 


      /**************   CDS Sensor signal detection(AD 변환) ************/

	   ADMUX &= ~0x1F;    //  ADC Chanel 0 : ADC0 선택

	   ADCSRA |= 0x40;   // ADC start 

	   while( ( ADCSRA & 0x10 ) == 0x00  ) ;  // Check if ADC Conversion is completed 

	   CDS_sensor_ouput = ADC;   
 
     /******************************************************/
   ////////////////////////////////////////////////////////////////////
   //////////                                               /////////// 
   //////////  Avg_Num(4개) 개씩 이동 평균(Moving Average)  ///////////
   //////////                                               ///////////
   ////////////////////////////////////////////////////////////////////

	   if( count1 <= ( Avg_Num -1 ) )
	   {

             CDS_sensor_ouput_buf[ count1 ] = CDS_sensor_ouput ;
			 CDS_Sum +=  CDS_sensor_ouput_buf[ count1 ] ; 
			 count1++ ; 
	   } 
	   else
	   {

             CDS_Sum +=  CDS_sensor_ouput  ;	       // 가장 최근 값 더하고  
             CDS_Sum -=  CDS_sensor_ouput_buf[ 0 ] ;   // 가장 오랜된 값 빼고 

             CDS_sensor_ouput_avg = CDS_Sum / Avg_Num ;     // 4개 이동 평균 

             for( i = 0; i <= (Avg_Num - 2) ; i++ )
			 {
                 CDS_sensor_ouput_buf[ i ]  = CDS_sensor_ouput_buf[ i+1 ] ;
			 } 

             CDS_sensor_ouput_buf[ Avg_Num - 1 ]  = CDS_sensor_ouput ;  

            ////////////////////////////////////////////////////////////////
         //////////////////////////////////////////////////////////////////
        if( CDS_sensor_ouput_avg <= 500 )
	      {	
	       //PORTB &= ~0x10;   // PB4  : Low ( LED ON )  
		   mode = 0;
           }

	        else if( CDS_sensor_ouput_avg > 500 )
	          {	
	          if (int_num == 0){
	 		  LcdCommand( ALLCLR ) ;    // LCD Clear
         	  LcdMove(0,0); 
	          LcdPuts("Good Moring");
			  mode = 1;
			   }
     
          /////////////////////////////////////  
		 else if(int_num ==1){
			LcdCommand( ALLCLR ) ;    // LCD Clear
         	LcdMove(1,0); 
	        LcdPuts("Completion");
			mode = 2;
			  }
			  
	      else if(int_num==2){
			  LcdCommand( ALLCLR ) ; 
			  }
         }
	   }
	 }  
    if( time_index2 == 50 )   // 50 msec 주기// 표 7.2 확인. 
    {

       time_index2 = 0; 

       //  초음파센서 1 트리거 신호 발생(초음파 1 발사) 

	   PORTB |= 0x02;    // PA1 : High//트리거 신호 유발 하도록 설정.
	   usec_delay(20) ;  // 20usec 동안 High 유지 
	   PORTB &= ~0x02;    // PA1 : Low 
      }

    if(mode == 1){  //  경고음 발생(거리가 40cm 이하일 때) 
    if( distance_1 <=  10  )    Warning_Flag = 1 ;     // 측정된 거리가 40 cm 이하이면 경고음 발생 플래그 set
       else                       Warning_Flag = 0 ;    
    Delay_Time =  distance_1 / 10 + 1;            // 거리에 비레하는 주기(= Delay_Time * 50 msec )를 갖는 경고음 발생
   if( Delay_Time <= 1)   Delay_Time = 1 ;   // 경고음주기 하한 : 0.1초
	   if( Delay_Time >= 4)   Delay_Time = 4 ;   // 경고음주기 상한 : 0.4초 
 
    if( Warning_Flag == 1 )
	   {
           if( Time_Delay_Polling( Delay_Time ) == 1 )     // 50msec * Delay_Time 경과 후 
	       {
               PORTB ^= 0x08  ;    // PB3(버저) toggle : 버저 단속음, 낮을수록 더짧은 시간으로 .
	       PORTB ^= 0x10  ;    // PB4(LED) toggle :  LED ON, OFF 반복 
	       }
	   }
       else if( Warning_Flag == 0 )
	   {
           PORTB &= ~0x08  ;    // PB3(버저) OFF : 버저 OFF 
		   PORTB |= 0x10  ;     // PB4(LED) OFF :  LED  OFF 
	   }
	  }
	  else{
	      PORTB &= ~0x08  ;
	  }
//////////////////////////////////////////////////////////////
		  

 
}

ISR(INT1_vect)    //  INT0 서비스 프로그램
{


      sei();            // 전역인터럽트 허용( 다른인터럽트(타이머인터럽트) 허용하고 싶을때) 
      EIMSK &= ~0x02;     // INT1 인터럽트 금지( 채터링 때문에 또 걸릴 수가 있어 금지.)


     int_num++;           // 스위치가 한번 눌러질 때마다 눌러진 횟수 1 증가 

     if( int_num == 3) int_num = 0 ;

	 if( int_num == 1 )      {
	 

	 }
//////////////////// 채터링 방지 ///////////////////

	  msec_delay( 20 );
	  while( ~PIND & 0x02 );
	  msec_delay( 20 );

	  EIFR = 0x02;   // 플래그비트 리셋	

///////////////////////////////////////////////////
      EIMSK |= 0x02;     // INT0 인터럽트 허용(위에 금지 한것 때문에 다시 허용)

 

}
ISR(INT5_vect)
{

    static unsigned short count1 = 0, count2 = 0, del_T = 0, flag = 0 ;


	  if(flag == 0) //에코 상
	  {
		  count1 = TCNT3; 
		  flag = 1;
	  } 
	  else // 에코 하강앳지 
	  { 
		  count2 = TCNT3; 
		  del_T = count2 - count1;
    	  distance_1 = del_T/(2*58); 

          if( distance_1 > 380 )  // 반사되는 초음파가 검출되지 않을때 
		  {
		      distance_1 = distance_1_old ;   // 직전 측정값 사용 
		  } 

          distance_1_old = distance_1 ;    // 직전 측정값 저장 변수 업데이트  

		  flag = 0; 

	  } 


} 
	
     




void Display_Number_LCD( unsigned int num, unsigned char digit )       // 부호없는 정수형 변수를 10진수 형태로 LCD 에 디스플레이 
{

	HexToDec( num, 10); //10진수로 변환 

	if( digit == 0 )     digit = 1 ;
	if( digit > 5 )      digit = 5 ;
 
    if( digit >= 5 )     LcdPutchar( NumToAsc(cnumber[4]) );  // 10000자리 디스플레이
	
	if( digit >= 4 )     LcdPutchar(NumToAsc(cnumber[3]));    // 1000자리 디스플레이 

	if( digit >= 3 )     LcdPutchar(NumToAsc(cnumber[2]));    // 100자리 디스플레이 

	if( digit >= 2 )     LcdPutchar(NumToAsc(cnumber[1]));    // 10자리 디스플레이

	if( digit >= 1 )     LcdPutchar(NumToAsc(cnumber[0]));    //  1자리 디스플레이

}
//======================================
// 한 문자를 송신한다.
//======================================

void SerialPutChar(char ch)
{
	while(!(UCSR1A & (1<<UDRE)));			// 버퍼가 빌 때를 기다림
  	UDR1 = ch;								// 버퍼에 문자를 쓴다
}
void SerialPutString(char *str)
 {

    while(*str != '\0')          // 수신된 문자가 Null 문자( 0x00 )가 아니면 
    {
        SerialPutChar(*str++);
    }
}



//=============================================
// 문자열을 송신한다.
// 입력   : str - 송신한 문자열을 저장할 버퍼의 주소
//=============================================






void HexToDec( unsigned short num, unsigned short radix) 
{
	int j ;

	for(j=0; j<5 ; j++) cnumber[j] = 0 ;

	j=0;
	do
	{
		cnumber[j++] = num % radix ; 
		num /= radix; 

	} while(num);

} 

char NumToAsc( unsigned char Num )
{
	if( Num <10 ) Num += 0x30; 
	else          Num += 0x37; 

	return Num ;
}



void msec_delay(unsigned int n)
{	
	for(; n>0; n--)		// 1msec 시간 지연을 n회 반복
		_delay_ms(1);		// 1msec 시간 지연
}

void usec_delay(unsigned int n)
{	
	for(; n>0; n--)		// 1usec 시간 지연을 n회 반복
		_delay_us(1);		// 1usec 시간 지연
}


unsigned char Time_Delay_Polling( unsigned short d_time )
{

    static unsigned short  curr_delay = 0; 
	unsigned char  ret_val = 0;


    curr_delay++ ;  

    if( curr_delay >= d_time )   // 50msec * d_time 경과 후 
	{
       ret_val = 1; 
       curr_delay = 0 ;
	} 


    return  ret_val ;


}
void DC_Motor_Run_Fwd( short duty )   // DC 모터 정회전 함수 
{

    if( duty > Vmax )     duty = Vmax ;

    PORTA &= ~0x01;     //  모터구동신호 - 단자 : 0 V 인가( PA0 = 0 );  
	OCR1A = duty;       //  모터구동신호 + 단자 : OC1A(PB5) PWM duty 설정 


}

void DC_Motor_Run_Rev( short duty )   // DC 모터 역회전 함수 
{

    if( duty > Vmax )     duty = Vmax ;

    PORTA |= 0x01;            //  모터구동신호 - 단자 : 5 V 인가( PA0 = 1 );  
	OCR1A = Vmax - duty;      //  모터구동신호 + 단자 : OC1A(PB5) PWM duty 설정 


}


void DC_Motor_Stop( void )   // DC 모터 정지 함수 
{

    PORTA &= ~0x01;     //  모터구동신호 - 단자 : 0 V 인가( PA0 = 0 );  
	OCR1A = 0;          //  모터구동신호 + 단자 : OC1A(PB5) PWM duty = 0 설정 


}


void DC_Motor_PWM( short Vref )   // DC 모터 PWM 신호 발생 함수  
{

   if ( Vref > Vmax )       Vref = Vmax ;
   else if( Vref < -Vmax )  Vref = -Vmax ;

   if( Vref > 0 )  
   {
      DC_Motor_Run_Fwd( Vref ) ;
   }
   else if( Vref == 0 )  
   {
      DC_Motor_Stop() ;
   }
   else if( Vref < 0 )  
   {
      DC_Motor_Run_Rev( -Vref ) ;
   }


}

///////////////////////////////////////////////////////////////////////////////


