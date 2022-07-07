
#define F_CPU 16000000UL
#define abs(n) (n > 0 ? n : -n)
#define error 3

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int ADC_SetChannel(unsigned char input);
void Uart_Trans(unsigned char data);
void Uart_TransNum(int num);

volatile int mode = -1;
volatile int exc = 0;
volatile int t = 0;
volatile int t_comp = 0;
volatile int ADC_data[8] = {0};
volatile int ADC_max[8] = {0};
volatile int ADC_min[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
volatile int ADC_norm[8] = {0};
volatile int binary[8] = {0};
volatile int weight[8] = {-12, -8, -4, -1, 1, 4, 8, 12};
volatile int result = 0;
volatile int speedL = 0;
volatile int speedR = 0;

ISR(INT0_vect) {
   mode = 0; // ADC max, min 구하기 / 종료
}

ISR(INT1_vect) {
   mode = 1; // 주행
}

ISR(TIMER0_OVF_vect) {
   TCNT0 = 100;
   
   for (int i =0; i < 8; i++) {
      ADC_data[i] = ADC_SetChannel(i);
   }
   
   if (mode == 0) {
      for (int i = 0; i < 8; i++) {
         if (ADC_data[i] > ADC_max[i]) ADC_max[i] = ADC_data[i];
         else if (ADC_data[i] < ADC_min[i]) ADC_min[i] = ADC_data[i];
      }
   }
   
   if (mode == 1) {
      // 정규화
      for (int i = 0; i < 8; i++) {
         ADC_norm[i] = ((float)(ADC_data[i] - ADC_min[i]) / (float)(ADC_max[i] - ADC_min[i])) * 100;
      }
      // 2진화
      for (int i = 0; i < 8; i++) {
         if (ADC_norm[i] < 35) binary[i] = 1; // 검은색
         else binary[i] = 0; // 흰색
      }
	   int led_bef = 0xff;
	   int led = 0xff;
	  int led_arr[8] = {255, 255, 255, 255, 255, 255, 255, 255};
	  led_bef = led; // led 초기화 전에 led_bef 에 값 저장
	 
	//  led = 0xff; // led 초기화
	  	   
      if (binary[0] == 1) led_arr[0] = 0b11111110;
      if (binary[1] == 1) led_arr[1] = 0b11111101;
      if (binary[2] == 1) led_arr[2] = 0b11111011;
      if (binary[3] == 1) led_arr[3] = 0b11110111;
      if (binary[4] == 1) led_arr[4] = 0b11101111;
      if (binary[5] == 1) led_arr[5] = 0b11011111;
      if (binary[6] == 1) led_arr[6] = 0b10111111;
      if (binary[7] == 1) led_arr[7] = 0b01111111;
      for (int i = 0; i < 8; i++) {
         led &= led_arr[i]; // led 에 새로운 led 값 저장
      }
      PORTA = led;
     
	  
      for (int i = 0; i < 8; i++) {
         result += binary[i] * weight[i];
      }

	  // stop: start 에서 가로선 구간 통과 시 걸린 시간 범위 안에 든 경우 => 멈춤(모터 정지 -> 모드 변경)
	 /* if (led == 0x00) {
		  t++; // 가로선 지나는 시간 카운팅 시작
	  }*/
	   if (led_bef == 0x00 && led != 0x00) { // 가로선 지난 순간
		  if (t > t_comp - error || t < t_comp + error) { // stop 구간인지 확인
			  OCR1A = 0;
			  OCR1B = 0; // 모터 정지
			  mode = 0; // 모드 변경으로 모터 작동 X
		  }
		  t_comp = t; // 카운팅한 값 저장
		  t = 0; // 초기화
	  }
      
      if ((led == 0xff && led_bef != 0xff) | abs(result) > 23) {
         if (result < 0) exc = -1;
         else if (result > 0) exc = 1;
      } // 선을 이탈했을 경우 | 3~5개의 LED에 불이 들어 온 경우
   /*   if (exc == -1) {
         while(1){
			 speedL = 790; speedR = 0;
				PORTA=0xf0;
			for (int i = 0; i < 8; i++) {
				for (int i =0; i < 8; i++) {
					ADC_data[i] = ADC_SetChannel(i);
				}
				 if (ADC_data[i] > ADC_max[i]) ADC_max[i] = ADC_data[i];
				 else if (ADC_data[i] < ADC_min[i]) ADC_min[i] = ADC_data[i];
			 }
			for (int i = 0; i < 8; i++) {
					 ADC_norm[i] = ((float)(ADC_data[i] - ADC_min[i]) / (float)(ADC_max[i] - ADC_min[i])) * 100;
			 }
			for (int i = 0; i < 8; i++) {
					 if (ADC_norm[i] < 35) binary[i] = 1; // 검은색
					 else binary[i] = 0; // 흰색
			 }
			for (int i = 0; i < 8; i++) {
					 result += binary[i] * weight[i];
			 }
			  if(binary[5]==1)
			  break;
		} // 선이 IR센서 중심부에 올 때까지 유지
      }
     else if (exc == 1) {
         
		 while(1){
			 PORTA=0xf0;
			 speedR = 790; speedL = 0;
			 for (int i = 0; i < 8; i++) {
				 for (int i =0; i < 8; i++) {
					 ADC_data[i] = ADC_SetChannel(i);
				 }
				 if (ADC_data[i] > ADC_max[i]) ADC_max[i] = ADC_data[i];
				 else if (ADC_data[i] < ADC_min[i]) ADC_min[i] = ADC_data[i];
			 }
			 for (int i = 0; i < 8; i++) {
				 ADC_norm[i] = ((float)(ADC_data[i] - ADC_min[i]) / (float)(ADC_max[i] - ADC_min[i])) * 100;
			 }
			 for (int i = 0; i < 8; i++) {
				 if (ADC_norm[i] < 35) binary[i] = 1; // 검은색
				 else binary[i] = 0; // 흰색
			 }
			 for (int i = 0; i < 8; i++) {
				 result += binary[i] * weight[i];
			 }
			 if(binary[5]==1)
			    break;
		 }
      }*/
      
     // else {
         speedL = 580 - (result * 10);
            if (speedL > 799) speedL = 790;
         speedR = 580 + (result * 10);
            if (speedR > 799) speedR = 790;
     // }
      
      OCR1A = speedL;
      OCR1B = speedR;
      
      //Uart_TransNum(OCR1A);
      //Uart_Trans('-');
      //Uart_TransNum(OCR1B);
      //Uart_Trans(13);
      
	  exc = 0;
      result = 0;
   }
   
} // 10ms 주기로 발생

int main(void)
{
   DDRA = 0xff; // LED
   DDRD = 0b00001000; // TXD_UART (PD3) 출력, 그외 입력 설정
   DDRF = 0x00; // IR
   DDRB = 0xff; // MotorPWM
   DDRE = 0x0f; // MotorDirection
   PORTA = 0xff;
   PORTE = 0b00001010;
      
   UCSR1B = (1<<RXEN1) | (1<<TXEN1); // 수신, 송신 기능 활성화
   UBRR1H = 0;
   UBRR1L = 8; // 통신속도 설정 115200
   
   EIMSK = (1<<INT0) | (1<<INT1);
   EICRA = (1<<ISC01) | (0<<ISC00) | (1<<ISC11) | (0<<ISC10); // INT0,1 모두 falling edge
   
   ADMUX = 0b01000000; // 기준 전압: AVCC
   ADCSRA = 0b10000111; // ADC 활성화, 분주비: 128
   
   // 8bit -> 10ms 주기로 반복 (while문 대체)
   TCCR0 = (0<<WGM01) | (0<<WGM00) | (0<<COM01) | (0<<COM00) | (1<<CS02) | (1<<CS01) |(1<<CS00);
   // Normal mode, PreScaler = 1024
   TCNT0 = 100; // 주기 10ms 설정을 위한 초기값
   TIMSK = (1<<TOIE0); // Overflow Interrupt Enable
   //16bit -> 모터 조절
   TCCR1A = (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
   TCCR1B = (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
   // Fast PWM, mode 14, PreScaler = 1
   ICR1 = 799;
   OCR1A = 0;
   OCR1B = 0;
   
   sei();
   
   while (1)
   {
   }
}


int ADC_SetChannel(unsigned char input)
{
   ADMUX = (input | 0x40); // 0x40 = 0b01000000
   ADCSRA |= (1<<ADSC); // ADC 변환 시작
   while(!(ADCSRA & (1<<ADIF)));
   return ADC;
}

void Uart_Trans(unsigned char data) // 한 자리씩만 전송
{
   while(!(UCSR1A & (1<<UDRE1)));
   UDR1 = data;
}

void Uart_TransNum(int num) // 하나씩 여러번 전송 (자릿수를 순서대로 보내 하나의 수처럼 보이게)
{
   if (num < 0) {
      Uart_Trans('-');
      num = -num;
   }
   
   // ASCII ) '0' = 48 => Uart_Trans의 인수가 char형태니까 숫자를 문자로 바꿔줘야 함
   Uart_Trans(((num % 10000) / 1000) + 48); // 천의 자리
   Uart_Trans(((num % 1000) / 100) + 48); // 백의 자리
   Uart_Trans(((num % 100) / 10) + 48); // 십의 자리
   Uart_Trans((num % 10) + 48); // 일의 자리
   /* (num % ~ )을 하는 이유: 해당 자릿수만 필요함 -> 나머지 계산으로 앞에 필요없는 부분을 없애줌
      ex) 1200의 백의 자리 구하기: 1200 % 1000 = 200 -> 200 / 100 = 2
   */
}