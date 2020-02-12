#include <stdint.h>
#include "tm4c123gh6pm.h"
#define RED        0x02
#define BLUE      0x04
#define GREEN    0x08
#define WHITE     0x0E
//Global Variables
//200- 53khz, 400 - 26khz, 300 - 35khz,265 ~ 40khz 

int address;
uint16_t periodOfDutyCycle = 264;//Period of the Duty Cycle
uint16_t dutycycle = 132; //duty cycle
uint32_t StartHIGH = 0, StartLOW = 0;
uint32_t Log1HIGH = 0, Log1LOW = 0, Log0HIGH = 0, Log0LOW = 0;
uint32_t BitStage = 0; //0 = StartHIGH;;;; 1 = StartLOW;;;; 2 = Logical1HIGH;;;; 3 = Logical1LOW;;;;
int startBitHigh, startBitLow, startBitHighFlag = 0;
int HighL1, LowL1, HighFlagL1 = 0;
int t400 = 0;
int temp123 = 0;
int startHighOrLow = 0;//start = 0, high = 1, low = 2
	
void Timer_Config(void){ //16 Mhz / 16,000 = 1ms
	// Timer1A configurationn 500
	SYSCTL_RCGCTIMER_R |= 0x02;	  // enable clock to timer Block 1
	TIMER1_CTL_R = 0;					             	// disable Timer1 during configuration
	TIMER1_CFG_R = 0x04;				          // 16-bit timer
	TIMER1_TAMR_R = 0x02;				        // periodic timer
	TIMER1_TAPR_R = 460;		              // 16MHz/400 = 40kHz 
	TIMER1_TAILR_R = 1;			          
	TIMER1_ICR_R = 0x01;				            // clear Timer1A timeout flag
	TIMER1_IMR_R |= 0x01;				          // enable Timer1A timeout interrupt 
	TIMER1_CTL_R |= 0x01;				          // enable Timer1A
	NVIC_PRI5_R = (NVIC_PRI5_R & 0xffffff) | 0xc0000000;
	NVIC_EN0_R |= 0x00200000;	        	// enable IRQ21
}

void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4 input, PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4
	GPIO_PORTF_DIR_R = 0x0E;            // PF4 in, PF3-1 out.
  GPIO_PORTF_DEN_R = 0x1E;          // enable digital pins PF4-PF1
}

void PWM1_0B_Init(uint16_t period, uint16_t duty){
  volatile unsigned long delay;
  SYSCTL_RCGCPWM_R |= 0x02;             // 1) activate PWM1
  SYSCTL_RCGCGPIO_R |= 0x08;            // 2) activate port D
  delay = SYSCTL_RCGCGPIO_R;            // allow time to finish activating
  GPIO_PORTD_AFSEL_R |= 0x02;           // enable alt funct on PD1
  GPIO_PORTD_PCTL_R &= ~0x000000F0;     // configure PD0 as M1PWM0B
  GPIO_PORTD_PCTL_R |= 0x00000050;
  GPIO_PORTD_AMSEL_R &= ~0x02;          // disable analog functionality on PD1
  GPIO_PORTD_DEN_R |= 0x02;             // enable digital I/O on PD1
  SYSCTL_RCC_R &= ~0x00100000;           // 3) use PWM divider
	//SYSCTL_RCC_R = 0x00100000 |           // 3) use PWM divider
    //(SYSCTL_RCC_R & (~0x000E0000));   //    configure for /2 divider
  PWM1_0_CTL_R = 0;                     // 4) re-loading down-counting mode
  PWM1_0_GENB_R = 0xC08;
  PWM1_0_LOAD_R = period - 1;           // 5) cycles needed to count down to 0
  PWM1_0_CMPB_R = duty - 1;             // 6) count value when output rises
  PWM1_0_CTL_R |= 0x00000001;           // 7) start PWM1
  PWM1_ENABLE_R |= 0x00000002;          // enable PD1/M1PWM0
}

void PWM1_0B_Duty(uint16_t duty){//Update DutyCycle
  PWM1_0_CMPB_R = duty - 1;             // 6) count value when output rises
}

void UART0_Init(void){
	//uoc_1 provide clk to UART0 by writing 1 to RCGCUART register
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
	//uoc_2 provide clk to PORTA by writing 1 to RCGCGPIO register
	SYSCTL_RCGCGPIO_R  |= SYSCTL_RCGCGPIO_R0;
	//uoc_3 disable the UART0 by writing 0 to UARTCTL register of UART0
	UART0_CTL_R = 0;
	//uoc_4 write the integer porition of th Baud rate to the UARTIBRD register of UART0
	UART0_IBRD_R = 104;
	//uoc_5 write the fractional portion of the Baud rate to the UARTFBRD register of UART0
	UART0_FBRD_R = 11;
	//uoc_6 select the system clock as UART clock source by writing a 0 to UARTCC register of UART0
	UART0_CC_R = 0;
	//uoc_7 configure the line control value for 1 stop bit, no FIFO, no interrupt, no parity, and 8-bit data size.
	//That vives use 0x60 for the UARTLCRH register of UART0
	UART0_LCRH_R |= 0X60;
	//uoc_8 set TxE and RxE bits in UARTCTL register to enable the transmitter and receiver of UART0
	UART0_CTL_R |= 0x0300;
	//uoc_9 set UARTEN bit in UARTCTL register to enable the UART0
	UART0_CTL_R |= 1;
	//uoc_10 make PA0 and PA1 pins to be used as digital I/O
	GPIO_PORTA_DEN_R |= 3;
	//uoc_11 select the alternate functions of PA0 (RxD) and PA1 (TxD) pins using to GPIOAFSEL
	GPIO_PORTA_AFSEL_R |= 3;
	//uoc_12 configure PA0 and PA1 pins for UART function
	GPIO_PORTA_PCTL_R |= 0X00000011;
}

void UART1_Init(void){
	//uoc_1 provide clk to UART0 by writing 1 to RCGCUART register
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
	//uoc_2 provide clk to PORTC by writing 1 to RCGCGPIO register
	SYSCTL_RCGCGPIO_R  |= SYSCTL_RCGCGPIO_R1;
	//Enable Port C Clock
	SYSCTL_RCGC2_R |= 0x00000004;
	//uoc_3 disable the UART0 by writing 0 to UARTCTL register of UART1
	UART1_CTL_R = 0;
	//uoc_4 write the integer porition of th Baud rate to the UARTIBRD register of UART0
	UART1_IBRD_R = 104;
	//uoc_5 write the fractional portion of the Baud rate to the UARTFBRD register of UART0
	UART1_FBRD_R = 11;
	//uoc_6 select the system clock as UART clock source by writing a 0 to UARTCC register of UART0
	UART1_CC_R = 0;
	//uoc_7 configure the line control value for 1 stop bit, no FIFO, no interrupt, no parity, and 8-bit data size.
	//That vives use 0x60 for the UARTLCRH register of UART1
	UART1_LCRH_R |= 0X60;
	//uoc_8 set TxE and RxE bits in UARTCTL register to enable the transmitter and receiver of UART0
	UART1_CTL_R |= 0x0300;
	//uoc_9 set UARTEN bit in UARTCTL register to enable the UART1
	UART1_CTL_R |= 1;
	//uoc_10 make PC4 and PPC5 pins to be used as digital I/O
	GPIO_PORTC_DEN_R |= 0x30;
	//uoc_11 select the alternate functions of PC4 (RxD) and PC5 (TxD) pins using to GPIOAFSEL
	GPIO_PORTC_AFSEL_R |= 0x30;
	//uoc_12 configure PC4 and PC5 pins for UART function
	GPIO_PORTC_PCTL_R |= 0X00220000;
}

void UART0_poll_Tx(uint8_t ByteToSend){
	while((UART0_FR_R & UART_FR_TXFF) != 0);
	UART0_DR_R = ByteToSend;
}

void UART1_poll_Tx(uint8_t ByteToSend){
	while((UART1_FR_R & UART_FR_TXFF) != 0);
	UART1_DR_R = ByteToSend;
}

void UART0_transmit_String(const uint8_t *MessageString){//UART Transmit String Function
	while( *MessageString){
		UART0_poll_Tx(*MessageString);
		MessageString++;
	}	
}

void UART1_transmit_String(char *MessageString){//UART Transmit String Function
	while( *MessageString){
		UART1_poll_Tx(*MessageString);
		MessageString++;
	}	
}

uint8_t UART0_poll_Rx(void){
	while((UART0_FR_R & UART_FR_RXFE) != 0);
	return UART0_DR_R;
}

uint8_t UART1_poll_Rx(void){
	while((UART1_FR_R & UART_FR_RXFE) != 0);
	return UART1_DR_R;
}

void Display(char *MessageString){
	while(*MessageString){
		UART0_poll_Tx(*MessageString);
		MessageString++;
	}
}

void PortF_Output(uint32_t data){ // write PF3-PF1 outputs
  GPIO_PORTF_DATA_R = data;      
}
void delayMs(uint16_t delayTime){
	uint32_t i, j;
	for(i = 0; i < delayTime; i++)
		for(j = 0; j < 318; j++)
	{}
}
void sendStartBit(void){
	startHighOrLow = 0;
	startBitHigh = 0;
	startBitLow = 0;
	startBitHighFlag = 0;
	PWM1_0B_Duty(dutycycle);
	Timer_Config();
	while(startBitHigh < 39);
	PWM1_0B_Duty(1);
	startBitHighFlag = 1;
	while(startBitLow < 20);
	TIMER1_CTL_R = 0;					             	// disable Timer1 during configuration
}	

void sendHighBit(void){
	startHighOrLow = 1;
	startBitHigh = 0;
	startBitLow = 0;
	startBitHighFlag = 0;
	PWM1_0B_Duty(dutycycle);
	Timer_Config();
	while(startBitHigh < 43);
	PWM1_0B_Duty(1);
	startBitHighFlag = 1;
	while(startBitLow < 16);
	TIMER1_CTL_R = 0;					             	// disable Timer1 during configuration	
}	

void sendLowBit(void){
	
	startHighOrLow = 2;
	startBitHigh = 0;
	startBitLow = 0;
	startBitHighFlag = 0;
	PWM1_0B_Duty(dutycycle);
	Timer_Config();
	while(startBitHigh < 15);
	PWM1_0B_Duty(1);
	startBitHighFlag = 1;
	while(startBitLow < 16);
	TIMER1_CTL_R = 0;					             	// disable Timer1 during configuration
	PortF_Output(BLUE);
}	

void Timer1A_Handler(void){               // Timer interval for starting Timer2A
	volatile uint32_t readback;
	if(startHighOrLow == 0){//start
		
		if(startBitHigh <= 40 && startBitHighFlag == 0){
			startBitHigh++;
		}
		else if(startBitLow <= 20 && startBitHighFlag == 1){
			startBitLow++;
		}
	}
	else if(startHighOrLow == 1){//high
		
		if(startBitHigh <= 43 && startBitHighFlag == 0){
			startBitHigh++;
		}
		else if(startBitLow <= 16 && startBitHighFlag == 1){
			startBitLow++;
		}
	}
	else{//low
		
		if(startBitHigh <= 16 && startBitHighFlag == 0){
			startBitHigh++;
		}
		else if(startBitLow <= 16 && startBitHighFlag == 1){
			startBitLow++;
		}
	}
	TIMER1_ICR_R = 0x01;                                 	// clear interrupt flag	
	readback = TIMER1_ICR_R;
}

void addressCall(void){
	if(address == 0){
		sendLowBit();  sendLowBit();
	}
	else if(address == 1){
		sendLowBit(); sendHighBit();
	}
	else if(address == 2){
		sendHighBit(); sendLowBit();
	}
	else{
		sendHighBit(); sendHighBit();
	}
}

int main(void){
	address = 0;
	char UART0Byte, UART1Byte = 0;
	char name[25]; int nameIndex = 0;
	char baud[25]; int baudIndex = 0;
	char pswd[25]; int pswdIndex = 0;
	//PLL_Init();
	SYSCTL_RCGC2_R |= 0x00000002;//Enable Clock B
	GPIO_PORTB_DEN_R |= 0x0F;
	GPIO_PORTB_DIR_R |= 0x0F;
	GPIO_PORTB_DATA_R |= 0x06;//Starts off with 1s so that H-bridge has direction be forward
	PortF_Init();
	UART0_Init(); //initialize polling
	UART1_Init();
	delayMs(1);
	Display("Enter '1' to Configure Bluetooth\n\r");
	UART0Byte = UART0_poll_Rx();
	UART0_poll_Tx(UART0Byte)
	UART0Byte = '0';
	Display("\n\r");
	if(UART0Byte == '1'){ //uart config if statement
		Display("Enter Name for BlueTooth\r\n");
		delayMs(100);
		UART0Byte = UART0_poll_Rx();
		while(UART0Byte != 0x0D){
			UART0_poll_Tx(UART0Byte);
			name[nameIndex] = UART0Byte;
			nameIndex++;
			UART0Byte = UART0_poll_Rx();
		}
		UART0_poll_Tx('\r');
		UART0_poll_Tx('\n');
		name[nameIndex] = '\r';
		name[nameIndex+1] = '\n';
		
		
		Display("Enter Baud Rate\r\n");
		delayMs(100);
		UART0Byte = UART0_poll_Rx();
		while(UART0Byte != 0x0D){
			UART0_poll_Tx(UART0Byte);
			baud[baudIndex] = UART0Byte;
			baudIndex++;
			UART0Byte = UART0_poll_Rx();
		}
		
		baud[baudIndex] = '\r';
		baud[baudIndex+1] = '\n';
		UART0_poll_Tx('\r');
		UART0_poll_Tx('\n');
		
		Display("Enter PSWD\r\n");
		delayMs(100);
		UART0Byte = UART0_poll_Rx();
		while(UART0Byte != 0x0D){
			UART0_poll_Tx(UART0Byte);
			pswd[pswdIndex] = UART0Byte;
			pswdIndex++;
			UART0Byte = UART0_poll_Rx();
		}
		
		pswd[pswdIndex] = '\r';
		pswd[pswdIndex+1] = '\n';
		UART0_poll_Tx('\r');
		UART0_poll_Tx('\n');
		
		UART1_transmit_String(name);
		delayMs(100);
			
		UART1Byte = UART1_poll_Rx();
		while(UART1Byte != 0x0D){
			UART0_poll_Tx(UART1Byte);
			UART1Byte = UART1_poll_Rx();
		}
		
		UART1_transmit_String(baud);
		delayMs(100);
		
		UART1Byte = UART1_poll_Rx();
		while(UART1Byte != 0x0D){
			UART0_poll_Tx(UART1Byte);
			UART1Byte = UART1_poll_Rx();
		}
		
    //57600 Baud Rate
    //uoc_4 write the integer porition of th Baud rate to the UARTIBRD register of UART1
    UART1_IBRD_R = 17;
    //uoc_5 write the fractional portion of the Baud rate to the UARTFBRD register of UART1
    UART1_FBRD_R = 23;
		
		UART1_transmit_String(pswd);
		delayMs(100);
		
		UART1Byte = UART1_poll_Rx();
		while(UART1Byte != 0x0D){
			UART0_poll_Tx(UART1Byte);
			UART1Byte = UART1_poll_Rx();
		}
	}
	else{		
	}
	PWM1_0B_Init(periodOfDutyCycle, 1); //40KHz, but starts off
	while(1)
		//sendStartBit();	sendLowBit();sendLowBit();/*addressCall();*/sendLowBit();	sendLowBit(); sendLowBit();	sendLowBit();
		//MB - PB3-Dir, PB2 - pow
		//MA - PB0-Dir, PB1 - pow
		//a-motor by batter - backward - backward 01 10  first motor backward  - 0X 1X
		//s-mbybatter       - backward - forward  00 11  second motor backward - X1 X0
		//d-forward         - forward  - backward 11 00  first motor forward   - 1X 0X
		//w                 - none     - none     11 11  second motor forward  - X0 X1
		UART1Byte = UART1_poll_Rx();
		UART1_poll_Tx(UART1Byte);	
		Display("\n\r");
		switch(UART1Byte){
			case 'w': GPIO_PORTB_DATA_R &= 0x00; GPIO_PORTB_DATA_R |= 0x09; PortF_Output(GREEN); break; //forward
			case 'a': GPIO_PORTB_DATA_R &= 0x00; GPIO_PORTB_DATA_R |= 0x0D; PortF_Output(GREEN); break; //backward
			case 's': GPIO_PORTB_DATA_R &= 0x00; GPIO_PORTB_DATA_R |= 0x06; PortF_Output(GREEN); break; //turn left
			case 'd': GPIO_PORTB_DATA_R &= 0x00; GPIO_PORTB_DATA_R |= 0x0B; PortF_Output(GREEN); break; //turn right
			case 'q': GPIO_PORTB_DATA_R |= 0x06; GPIO_PORTB_DATA_R &= 0x00; PortF_Output(RED); break; //stop motion
			case 'c': address++; address = address%4;break;
			case '0':	sendStartBit(); addressCall(); sendLowBit(); sendLowBit();  //7'b1xx0000
                sendLowBit();	sendLowBit();	PWM1_0B_Duty(1); break;
			case '1':	sendStartBit(); addressCall(); sendLowBit(); sendLowBit();  //7'b1xx0001
                sendLowBit();	sendHighBit(); PWM1_0B_Duty(1); break;
			case '2':	sendStartBit();	addressCall(); sendLowBit(); sendLowBit();  //7'b1xx0010
                sendHighBit(); sendLowBit(); PWM1_0B_Duty(1); break;
			case '3':	sendStartBit(); addressCall(); sendLowBit(); sendLowBit();  //7'b1xx0011
                sendHighBit(); sendHighBit(); PWM1_0B_Duty(1); break;
			case '4':	sendStartBit(); addressCall(); sendLowBit(); sendHighBit(); //7'b1xx0100
                sendLowBit();	sendLowBit(); PWM1_0B_Duty(1); break;
			default: 
			break;
		}
	}
}

