
#include <avr/io.h>
#include <avr/interrupt.h> 
#include <util/delay.h>
#include "lcd.h"
#define  Avg_Num     4         //  �̵� ��� ���� 
#define  Amp_Gain   11         //  ������ �̵� 

void init_serial(void) ;  //  Serial �����Ʈ �ʱ�ȭ
void HexToDec( unsigned short num, unsigned short radix); 
char NumToAsc( unsigned char Num ); 
void SerialPutChar(char ch);
void SerialPutString(char str[]);
void Display_Number_LCD( unsigned int num, unsigned char digit ) ;    // ��ȣ���� ������ ������ 10���� ���·� LCD �� ���÷��� 
void msec_delay(unsigned int n);
void usec_delay(unsigned int n);
void DC_Motor_Run_Fwd( short duty );    // DC ���� ��ȸ��(PWM����) �Լ� 
void DC_Motor_Run_Rev( short duty );    // DC ���� ��ȸ��(PWM����) �Լ�  
void DC_Motor_Stop( void );             // DC ���� ���� �Լ�  
void DC_Motor_PWM( short Vref );        // DC ���� PWM ��ȣ �߻� �Լ�  
                                        // ����ũ(Vref>0), ����ũ(Vref<0), ����ũ(Vref=0) ��� ���� 


unsigned char Time_Delay_Polling( unsigned short d_time ) ;   // �ð����� üũ�Լ�(�������

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
     // Push Switch : �ܺ����ͷ�Ʈ 0 (INT0 : PD0 )�� ����   
	//����ġ. 
	 DDRD &= ~0x02;     // PD0 (�ܺ����ͷ�Ʈ INT0 ) : �Է¼���   
     PORTD |= 0x02;     // PD0 : ����Ǯ�����   
 
	 DDRB |= 0x10; 	   // LED (PB0 ) :��¼���	

	 PORTB |= 0x10;    // LED OFF
	 
	DDRB |= 0x02;    // 3 �����ļ��� Trigger signal( PA1   )
	PORTB &= ~0x02;   // PA1  : Low    
	
	DDRB |= 0x08;// �ɵ�����(BUzzer)(PA3)
	PORTB &= ~0x08;//PB1 = low(���� off)

	DDRB |= 0x10;//LED(PA4)
	PORTB |= 0x10;//PA4 = high => led off high��Ʈ�� 1�̿��� 1�� ǥ���Ҷ��� or�� ����Ѵ�.

     DDRB |= 0x20;   // ���ͱ�����ȣ + ����:  PWM ��Ʈ( pin: OC1A(PB5) )   --> ��� ���� 
	 DDRA |= 0x01;   // ���ͱ�����ȣ - ���� : ���� ��/�����Ʈ(pin : PA0 ) --> ��� ���� 

	 init_serial() ;   // Serial Port (USART1) �ʱ�ȭ
	 
	 LcdInit();       // LCD �ʱ�ȭ 

     UCSR1B |=  0x80  ;      // UART1 �۽�(RX) �Ϸ� ���ͷ�Ʈ ���
     sei() ;                 // �������ͷ�Ʈ���
    ////////  �ܺ� ���ͷ�Ʈ(INT0 ) ����  ///////////

    EICRA &= ~0x04;  // INT1 �ϰ��𼭸����� ���ͷ�Ʈ �ɸ�
    EICRA |=  0x08;  // INT1 �ϰ��𼭸����� ���ͷ�Ʈ �ɸ�

    EIMSK |=  0x02;  // INT1 ���ͷ�Ʈ  ���

  ///////////////////////////////////////////////


     LcdCommand( ALLCLR ) ;    // LCD Clear
  	 LcdMove(0,0);    
	 LcdPuts("Blutooth Module"); 
     
	 /*****   AD Converter **********/

     ADMUX &= ~0xE0;    //  ADC �������� = AREF ,   ADC ��� ���������� 
     ADCSRA |= 0x87;     // ADC enable, Prescaler = 128
 
  	 LcdMove(1,0);    
	 LcdPuts("HC-06 Test Prog"); 
	 /**** Timer0 Overflow Interrupt  ******/
     /**************************************/
     TCCR0 = 0x00; 
     TCNT0 = 256 - 156;       //  ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                             //  �����÷����ͷ�Ʈ �ֱ� = 10msec
                             //  156 = 10msec/ 64use

     TIMSK = 0x01;  // Timer0 overflow interrupt enable 
	

// 3 Echo Signals Pulse Width(�����ļ��� �պ��ð� T����) measurment,  Timer3 

	TCCR3A = 0x00;// Ÿ�̸ӿ뵵.  
	TCCR3B = 0x02;     // Ÿ�̸� 3 ����(���ֺ� 8) ,  0.5usec ������ ���� 

/////////////////////////////////////////////////////////
	
   // �����ļ��� Echo Signals : external interrupt 4( pin: INT4 (PE4)),  ////�ٲܰ� 

	EICRB = 0x04;   //0000 0100 INT5 Both falling edge and rising edge interrupt
	EIMSK |= 0x20;   // 0010 0000INT5 Enable 
	
	
    //sei();         // Global Interrupt Enable 
	//cli():���� ���ͷ�Ʈ ��� ���� 

   ///////////////////////////////////////
   	//  ���� �����ļ��� 1 Ʈ���� ��ȣ �߻�(������ 1 �߻�)  
	PORTB |= 0x02;    // PA1 : High
	usec_delay(20) ;  // 20usec ���� High ���� 
	PORTB &= 0xFD;    // PA1 : Low 
  /////////////////////////////////////////////
   ////////////////////////////////////////////
    // ���ͱ�����ȣ ( pin: OC1A(PB5) ),   Timer1, PWM signal (period= 200 usec )

	TCCR1A = 0x82;    // OC1A(PB5)) :  PWM ��Ʈ ����,   Fast PWM ( mode 14 )
	TCCR1B = 0x1b;    // 64 ���� Ÿ�̸� 1 ���� (����Ŭ�� �ֱ� =  64/(16*10^6) = 4 usec ),  Fast PWM ( mode 14 ) 
	ICR1 = 50;       // PWM �ֱ� = 50 * 4 usec = 200 usec (  PWM ���ļ� = 1/200usec = 5 kHz )

    Vmax = ICR1; 

	OCR1A = duty;      //  OC1A(PB5) PWM duty = 0 ���� : ���� ����
   //////////////////////////////////////////////////////////////////
	 

	TCCR0 |= 0x07; // Clock Prescaler N=1024 (Timer 0 Start)
  
	 while(1)
	 {

         if( new_recv_flag == 1 )      // 1 ���� ���ſϷ� �� 
		 { 
  

     	//////////////  ��ɾ� ó��   //////////////

			if( rdata == '0' )          // ���� 0 �� ���ŵǸ� 
			{
			   duty += 300;
			   if(duty >= Vmax) duty = Vmax; 
               SerialPutString("1:ON 2:RE 3:STOP.\n");  // �޴������� �޽��� ����
                
			//	PORTB &= ~0x10;         // LED ON
			}
			else if( rdata == '1' )     // ���� 1 �� ���ŵǸ�
			{
			    duty = 50;
                DC_Motor_Run_Fwd( duty );      // DC Motor ��ȸ��
                msec_delay( 75 ); 
				DC_Motor_Stop(); 
				
			}
			else if( rdata == '2')      // ���� 2 �� ���ŵǸ�
			{
                
		        DC_Motor_Run_Rev( duty );     // DC Motor ��ȸ�� 5��
				msec_delay( 80 );     // ȸ���̶� ��ȸ���� �ٸ��� �������� �ణ ���̰� ����
				DC_Motor_Stop(); 
			
			} 
				else if( rdata == '3')      // ���� 3 �� ���ŵǸ�
			{
                
		         DC_Motor_Stop();               // DC Motor ���� 
			
			} 
		

			else if( rdata != 0xFF)    //  ��� ���� �̸�
			{

                SerialPutString("Command Error!!  Try again.\n" ); //  ��� ���� �޽��� ��
			}


		    rdata = 0xFF;
            new_recv_flag = 0;      // �� ����(���) ���� �÷��� Reset
  

        }
    //////////////////////////////////////////////////

 
	}

} 

ISR(  USART1_RX_vect )
{

    rdata = UDR1; 
 
    SerialPutChar( rdata);           // Echo  ���ŵ� �����͸� �ٷ� �۽��Ͽ� ���ŵ� �����Ͱ� ��Ȯ���� Ȯ�� 
    SerialPutChar('\n');             // �޴������� ������ ���۽� Line Feed('\n')�� �׻� ���� �����ؾ���

    recv_cnt++ ;                     // ���ŵ� ������ ����Ʈ�� ����

    new_recv_flag = 1;               // �� ����(���) ���� �÷��� Set

}

void init_serial(void)
{
    UCSR1A=0x00;                    //�ʱ�ȭ
    UCSR1B=0x18;                    //�ۼ������,�������ͷ�Ʈ ����
    UCSR1C=0x06;                    //������ ���ۺ�Ʈ �� 8��Ʈ�� ����.
    
    UBRR1H=0x00;
    UBRR1L=103;                     //Baud Rate 9600 
}
ISR(TIMER0_OVF_vect)   // Timer0 overflow interrupt( 10 msec)  service routine
{

    static unsigned short  time_index = 0,  time_index2 = 0,count1 = 0,CDS_Sum = 0 ; 
    static unsigned short  CDS_sensor_ouput_buf[Avg_Num ]   ; 
    unsigned char i = 0 ;


    TCNT0 = 256 - 156;       //  ����Ŭ���ֱ� = 1024/ (16x10^6) = 64 usec,  
                             //  �����÷����ͷ�Ʈ �ֱ� = 10msec
                             //  156 = 10msec/ 64usec

     time_index++ ; 
	 time_index2++ ; 


    if( time_index == 25 )    // ���ø��ֱ� =  250 msec = 10msec x 25 
    {

       time_index = 0; 


      /**************   CDS Sensor signal detection(AD ��ȯ) ************/

	   ADMUX &= ~0x1F;    //  ADC Chanel 0 : ADC0 ����

	   ADCSRA |= 0x40;   // ADC start 

	   while( ( ADCSRA & 0x10 ) == 0x00  ) ;  // Check if ADC Conversion is completed 

	   CDS_sensor_ouput = ADC;   
 
     /******************************************************/
   ////////////////////////////////////////////////////////////////////
   //////////                                               /////////// 
   //////////  Avg_Num(4��) ���� �̵� ���(Moving Average)  ///////////
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

             CDS_Sum +=  CDS_sensor_ouput  ;	       // ���� �ֱ� �� ���ϰ�  
             CDS_Sum -=  CDS_sensor_ouput_buf[ 0 ] ;   // ���� ������ �� ���� 

             CDS_sensor_ouput_avg = CDS_Sum / Avg_Num ;     // 4�� �̵� ��� 

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
    if( time_index2 == 50 )   // 50 msec �ֱ�// ǥ 7.2 Ȯ��. 
    {

       time_index2 = 0; 

       //  �����ļ��� 1 Ʈ���� ��ȣ �߻�(������ 1 �߻�) 

	   PORTB |= 0x02;    // PA1 : High//Ʈ���� ��ȣ ���� �ϵ��� ����.
	   usec_delay(20) ;  // 20usec ���� High ���� 
	   PORTB &= ~0x02;    // PA1 : Low 
      }

    if(mode == 1){  //  ����� �߻�(�Ÿ��� 40cm ������ ��) 
    if( distance_1 <=  10  )    Warning_Flag = 1 ;     // ������ �Ÿ��� 40 cm �����̸� ����� �߻� �÷��� set
       else                       Warning_Flag = 0 ;    
    Delay_Time =  distance_1 / 10 + 1;            // �Ÿ��� ���ϴ� �ֱ�(= Delay_Time * 50 msec )�� ���� ����� �߻�
   if( Delay_Time <= 1)   Delay_Time = 1 ;   // ������ֱ� ���� : 0.1��
	   if( Delay_Time >= 4)   Delay_Time = 4 ;   // ������ֱ� ���� : 0.4�� 
 
    if( Warning_Flag == 1 )
	   {
           if( Time_Delay_Polling( Delay_Time ) == 1 )     // 50msec * Delay_Time ��� �� 
	       {
               PORTB ^= 0x08  ;    // PB3(����) toggle : ���� �ܼ���, �������� ��ª�� �ð����� .
	       PORTB ^= 0x10  ;    // PB4(LED) toggle :  LED ON, OFF �ݺ� 
	       }
	   }
       else if( Warning_Flag == 0 )
	   {
           PORTB &= ~0x08  ;    // PB3(����) OFF : ���� OFF 
		   PORTB |= 0x10  ;     // PB4(LED) OFF :  LED  OFF 
	   }
	  }
	  else{
	      PORTB &= ~0x08  ;
	  }
//////////////////////////////////////////////////////////////
		  

 
}

ISR(INT1_vect)    //  INT0 ���� ���α׷�
{


      sei();            // �������ͷ�Ʈ ���( �ٸ����ͷ�Ʈ(Ÿ�̸����ͷ�Ʈ) ����ϰ� ������) 
      EIMSK &= ~0x02;     // INT1 ���ͷ�Ʈ ����( ä�͸� ������ �� �ɸ� ���� �־� ����.)


     int_num++;           // ����ġ�� �ѹ� ������ ������ ������ Ƚ�� 1 ���� 

     if( int_num == 3) int_num = 0 ;

	 if( int_num == 1 )      {
	 

	 }
//////////////////// ä�͸� ���� ///////////////////

	  msec_delay( 20 );
	  while( ~PIND & 0x02 );
	  msec_delay( 20 );

	  EIFR = 0x02;   // �÷��׺�Ʈ ����	

///////////////////////////////////////////////////
      EIMSK |= 0x02;     // INT0 ���ͷ�Ʈ ���(���� ���� �Ѱ� ������ �ٽ� ���)

 

}
ISR(INT5_vect)
{

    static unsigned short count1 = 0, count2 = 0, del_T = 0, flag = 0 ;


	  if(flag == 0) //���� ��
	  {
		  count1 = TCNT3; 
		  flag = 1;
	  } 
	  else // ���� �ϰ����� 
	  { 
		  count2 = TCNT3; 
		  del_T = count2 - count1;
    	  distance_1 = del_T/(2*58); 

          if( distance_1 > 380 )  // �ݻ�Ǵ� �����İ� ������� ������ 
		  {
		      distance_1 = distance_1_old ;   // ���� ������ ��� 
		  } 

          distance_1_old = distance_1 ;    // ���� ������ ���� ���� ������Ʈ  

		  flag = 0; 

	  } 


} 
	
     




void Display_Number_LCD( unsigned int num, unsigned char digit )       // ��ȣ���� ������ ������ 10���� ���·� LCD �� ���÷��� 
{

	HexToDec( num, 10); //10������ ��ȯ 

	if( digit == 0 )     digit = 1 ;
	if( digit > 5 )      digit = 5 ;
 
    if( digit >= 5 )     LcdPutchar( NumToAsc(cnumber[4]) );  // 10000�ڸ� ���÷���
	
	if( digit >= 4 )     LcdPutchar(NumToAsc(cnumber[3]));    // 1000�ڸ� ���÷��� 

	if( digit >= 3 )     LcdPutchar(NumToAsc(cnumber[2]));    // 100�ڸ� ���÷��� 

	if( digit >= 2 )     LcdPutchar(NumToAsc(cnumber[1]));    // 10�ڸ� ���÷���

	if( digit >= 1 )     LcdPutchar(NumToAsc(cnumber[0]));    //  1�ڸ� ���÷���

}
//======================================
// �� ���ڸ� �۽��Ѵ�.
//======================================

void SerialPutChar(char ch)
{
	while(!(UCSR1A & (1<<UDRE)));			// ���۰� �� ���� ��ٸ�
  	UDR1 = ch;								// ���ۿ� ���ڸ� ����
}
void SerialPutString(char *str)
 {

    while(*str != '\0')          // ���ŵ� ���ڰ� Null ����( 0x00 )�� �ƴϸ� 
    {
        SerialPutChar(*str++);
    }
}



//=============================================
// ���ڿ��� �۽��Ѵ�.
// �Է�   : str - �۽��� ���ڿ��� ������ ������ �ּ�
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
	for(; n>0; n--)		// 1msec �ð� ������ nȸ �ݺ�
		_delay_ms(1);		// 1msec �ð� ����
}

void usec_delay(unsigned int n)
{	
	for(; n>0; n--)		// 1usec �ð� ������ nȸ �ݺ�
		_delay_us(1);		// 1usec �ð� ����
}


unsigned char Time_Delay_Polling( unsigned short d_time )
{

    static unsigned short  curr_delay = 0; 
	unsigned char  ret_val = 0;


    curr_delay++ ;  

    if( curr_delay >= d_time )   // 50msec * d_time ��� �� 
	{
       ret_val = 1; 
       curr_delay = 0 ;
	} 


    return  ret_val ;


}
void DC_Motor_Run_Fwd( short duty )   // DC ���� ��ȸ�� �Լ� 
{

    if( duty > Vmax )     duty = Vmax ;

    PORTA &= ~0x01;     //  ���ͱ�����ȣ - ���� : 0 V �ΰ�( PA0 = 0 );  
	OCR1A = duty;       //  ���ͱ�����ȣ + ���� : OC1A(PB5) PWM duty ���� 


}

void DC_Motor_Run_Rev( short duty )   // DC ���� ��ȸ�� �Լ� 
{

    if( duty > Vmax )     duty = Vmax ;

    PORTA |= 0x01;            //  ���ͱ�����ȣ - ���� : 5 V �ΰ�( PA0 = 1 );  
	OCR1A = Vmax - duty;      //  ���ͱ�����ȣ + ���� : OC1A(PB5) PWM duty ���� 


}


void DC_Motor_Stop( void )   // DC ���� ���� �Լ� 
{

    PORTA &= ~0x01;     //  ���ͱ�����ȣ - ���� : 0 V �ΰ�( PA0 = 0 );  
	OCR1A = 0;          //  ���ͱ�����ȣ + ���� : OC1A(PB5) PWM duty = 0 ���� 


}


void DC_Motor_PWM( short Vref )   // DC ���� PWM ��ȣ �߻� �Լ�  
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


