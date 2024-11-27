#include "stm32f10x.h"
#include "cmsis_os.h"

//Pin definition
//LEDs
#define LED1 0  //PA0
#define LED2 1  //PA1
#define LED3 2  //PA2
#define LED4 15 //PA15
#define LED5 8  //PA8
#define LED6 6  //PA6
#define LED7 5  //PA5
#define LED8 11 //PA11

//Display LCD
#define LCD_DB4 8 //PA8
#define LCD_DB5 6 //PA6
#define LCD_DB6 5 //PA5
#define LCD_DB7 11 //PA11
#define LCD_RS  15 //PA15
#define LCD_EN  12 //PA12

//Comandos
#define POSICAO_00 0x02 //00 linha 0 e coluna 0
#define POSICAO_10 0xC0 //10 linha 1 e coluna 0
#define LIMPAR 0x01 // limpa o display  

//Switches
#define SW1 12  //PB12
#define SW2 13  //PB13
#define SW3 14  //PB14
#define SW4 15  //PB15
#define SW5 5   //PB5
#define SW6 4   //PB4
#define SW7 3   //PB3
#define SW8 3   //PA3
#define SW9 4   //PA4
#define SW10 8  //PB8
#define SW11 9  //PB9
#define SW12 11 //PB11
#define SW13 10 //PB10
#define SW14 7  //PA7
#define SW15 15 //PC15
#define SW16 14 //PC14
#define SW17 13 //PC13

//Potentiometer
#define POT 1 //PB1

//Buzzer
#define BUZ 0 //PB0


int val01_delay = 1000000;
int val02_delay = 1000000;

int val01_osDelay = 1000000;
int val02_osDelay = 1000000;

/*----------------------------------------------------------------------------
  Simple delay routine
 *---------------------------------------------------------------------------*/
void delay_ms(uint16_t t)
{
	volatile unsigned long l = 0;
	for(uint16_t i = 0; i < t; i++)
		for(l = 0; l < 6000; l++)
		{
		}
}
void delay_us(uint16_t t)
{
	volatile unsigned long l = 0;
	for(uint16_t i = 0; i < t; i++)
		for(l = 0; l < 6; l++)
		{
		}
}

void setup_RedPill(){

	//int16_t swa, swb, swc;  //Variables to read the switches according to the port it is connected
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable AF clock
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
	delay_ms(100);
	
	RCC->APB2ENR |= 0xFC |(1<<9);		//ENABLE clocks for GPIOs and ADC1
	//Setting up outputs for leds
	ADC1->CR2 = 1;	/* ADON = 1 (power-up) */
	ADC1->CR2 |=(1<<2); //enable calibration
	ADC1->SMPR2 = 1<<3; /* SMP1 = 001 */
	delay_us(1);	/* wait 1us to make sure the adc module is stable */
	GPIOA->CRL = 0x43344333;	//PA3, PA4 and PA7: inputs (switches)
	GPIOA->CRH = 0x33333333;  //PA8 - PA15: outputs (leds)
	
	//Settig up inputs for switches
	GPIOB->CRL = 0x4444440B; //PB0 set for output+alternate wave form, since it is connected to buzzer.
	GPIOB->CRH = 0x44444444;
	GPIOB->ODR = 0xF000; //set pull-up in PB12 - PB15 	
	GPIOC->CRH = 0x44444444;
	GPIOC->ODR = 0xFFFFFFFF; //set pull-up in GPIOC
	
	delay_ms(1); //wait for I/O setup
	GPIOA->ODR &=~(1<<LCD_RS); //Turn off LED4
	delay_ms(1); //wait for LED4 to turn off
}

/*----------------------------------------------------------------------------
  LED setup routines
 *---------------------------------------------------------------------------*/
void ledsON(int led){
		switch(led){
			case 1:
				GPIOA->ODR |= (1<<LED1);  // led 1 - PA0
				break;
			
			case 2:
				GPIOA->ODR |= (1<<LED2);  // led 2 - PA0
				break;
			
			case 3:
				GPIOA->ODR |= (1<<LED3);  // led 3 - PA0
				break;
			
			case 4:
				GPIOA->ODR |= (1<<LED4);  // led 4 - PA0
				break;
			
			case 5:
				GPIOA->ODR |= (1<<LED5);  // led 5 - PA0
				break;
			
			case 6:
				GPIOA->ODR |= (1<<LED6);  // led 6 - PA0
				break;
			
			case 7:
				GPIOA->ODR |= (1<<LED7);  // led 7 - PA0
				break;
			
			case 8:
				GPIOA->ODR |= (1<<LED8);  // led 8 - PA0
				break;
			
			default: break;
		}
}

void ledsOFF(int led){
	switch(led){
			case 1:
				GPIOA->ODR &= ~(1<<LED1);	 // led 1 - PA0
				break;
			
			case 2:
				GPIOA->ODR &= ~(1<<LED2);	 // led 2 - PA0
				break;
			
			case 3:
				GPIOA->ODR &= ~(1<<LED3);	 // led 3 - PA0
				break;
			
			case 4:
				GPIOA->ODR &= ~(1<<LED4);	 // led 4 - PA0
				break;
			
			case 5:
				GPIOA->ODR &= ~(1<<LED5);	 // led 5 - PA0
				break;
			
			case 6:
				GPIOA->ODR &= ~(1<<LED6);	 // led 6 - PA0
				break;
			
			case 7:
				GPIOA->ODR &= ~(1<<LED7);	 // led 7 - PA0
				break;
			
			case 8:
				GPIOA->ODR &= ~(1<<LED8);	 // led 8 - PA0
				break;
			
			default: break;
		}
}
void leds_apaga_todos(){ 
    GPIOA->ODR &= ~((1<<LED1)|(1<<LED2)|(1<<LED3)|(1<<LED4)|(1<<LED5)|(1<<LED6)|(1<<LED7)|(1<<LED8));
}


void lcd_putValue(unsigned char value)
{
	uint16_t aux;
	aux = 0x0000; //clear aux
	GPIOA->BRR = (1<<5)|(1<<6)|(1<<8)|(1<<11); /* clear PA5, PA6, PA8, PA11 */
	//GPIOA->BSRR = (value>>4)&0x0F; /* put high nibble on PA0-PA3 */	
	aux = value & 0xF0;
	aux = aux>>4;
	GPIOA->BSRR = ((aux&0x0008)<<8) | ((aux&0x0004)<<3) | ((aux&0x0002)<<5) | ((aux&0x0001)<<8);
	//GPIOA->BSRR = ;
	//GPIOA->BSRR = ;
	//GPIOA->BSRR = ;
	
	GPIOA->ODR |= (1<<LCD_EN); /* EN = 1 for H-to-L pulse */
	delay_ms(3);			/* make EN pulse wider */
	GPIOA->ODR &= ~ (1<<LCD_EN);	/* EN = 0 for H-to-L pulse */
	delay_ms(1);			/* wait	*/

	GPIOA->BRR = (1<<5)|(1<<6)|(1<<8)|(1<<11); /* clear PA5, PA6, PA8, PA11 */
	//GPIOA->BSRR = value&0x0F; /* put low nibble on PA0-PA3 */	
	aux = 0x0000; //clear aux
	aux = value & 0x0F;
	GPIOA->BSRR = ((aux&0x0008)<<8) | ((aux&0x0004)<<3) | ((aux&0x0002)<<5) | ((aux&0x0001)<<8);
	//GPIOA->BSRR = (aux&0x0008)<<8;
	//GPIOA->BSRR = (aux&0x0004)<<6;
	//GPIOA->BSRR = (aux&0x0002)<<5;
	//GPIOA->BSRR = (aux&0x0001)<<5;
	//GPIOA->ODR = aux;
	
	GPIOA->ODR |= (1<<LCD_EN); /* EN = 1 for H-to-L pulse */
	delay_ms(3);			/* make EN pulse wider */
  GPIOA->ODR &= ~(1<<LCD_EN);	/* EN = 0 for H-to-L pulse */
  delay_ms(1);			/* wait	*/
}
void lcd_command(unsigned char cmd) {
	GPIOA->ODR &= ~ (1<<LCD_RS);	/* RS = 0 for command */
	lcd_putValue(cmd);
}
void lcd_data(unsigned char data){
	GPIOA->ODR |= (1<<LCD_RS);	/* RS = 1 for data */
	lcd_putValue(data); 
}
void lcd_print(char * str){
  unsigned char i = 0;

	while(str[i] != 0) /* while it is not end of string */
	{
		lcd_data(str[i]); /* show str[i] on the LCD */
		i++;
	}
}


void lcd_init(){
	delay_ms(15);
	GPIOA->ODR &= ~(1<<LCD_EN);	/* LCD_EN = 0 */
	delay_ms(3); 			/* wait 3ms */
	lcd_command(0x33); //lcd init.
	delay_ms(5);
	lcd_command(0x32); //lcd init.
	delay_us(3000);
	lcd_command(0x28); // 4-bit mode, 1 line and 5x8 charactere set
	delay_ms(3);
	lcd_command(0x0e); // display on, cursor on
	delay_ms(3);
	lcd_command(0x01); // display clear
	delay_ms(3);
	lcd_command(0x06); // move right
	delay_ms(3);
}

void adc_init(){
	ADC1->SQR3 = 9;	/* choose channel 9 as the input */
	ADC1->CR2 = 1;	/* ADON = 1 (start conversion) */
	ADC1->CR2 |= 2; //autocalibration
	delay_us(2);
}
int chaves_ler_todas(){
	int chave_id_press;

    if ((GPIOB->IDR & (1 << SW1)) == 0) {
        return chave_id_press = 1;  // SW1 pressionado
    } else if ((GPIOB->IDR & (1 << SW2)) == 0) {
        return chave_id_press = 2;  // SW2 pressionado
    } else if ((GPIOB->IDR & (1 << SW3)) == 0) {
        return chave_id_press = 3;  // SW3 pressionado
    } else if ((GPIOB->IDR & (1 << SW4)) == 0) {
        return chave_id_press = 4;  // SW4 pressionado
    } else if ((GPIOB->IDR & (1 << SW5)) == 0) {
        return chave_id_press = 5;  // SW5 pressionado
    } else if ((GPIOB->IDR & (1 << SW6)) == 0) {
        return chave_id_press = 6;  // SW6 pressionado
    } else if ((GPIOB->IDR & (1 << SW7)) == 0) {
        return chave_id_press = 7;  // SW7 pressionado
    } else if ((GPIOA->IDR & (1 << SW8)) == 0) {
        return chave_id_press = 8;  // SW8 pressionado
    } else if ((GPIOA->IDR & (1 << SW9)) == 0) {
        return chave_id_press = 9;  // SW9 pressionado
    } else if ((GPIOB->IDR & (1 << SW10)) == 0) {
        return chave_id_press = 10;  // SW10 pressionado
    } else if ((GPIOB->IDR & (1 << SW11)) == 0) {
        return chave_id_press = 11;  // SW11 pressionado
    } else if ((GPIOB->IDR & (1 << SW12)) == 0) {
        return chave_id_press = 12;  // SW12 pressionado
		} else if ((GPIOB->IDR & (1 << SW13)) == 0) {
        return chave_id_press = 13;  // SW13 pressionado
    } else if ((GPIOA->IDR & (1 << SW14)) == 0) {
        return chave_id_press = 14;  // SW14 pressionado
    } else if ((GPIOC->IDR & (1 << SW15)) == 0) {
        return chave_id_press = 15;  // SW15 pressionado
		} else if ((GPIOC->IDR & (1 << SW16)) == 0) {
        return chave_id_press = 16;  // SW16 pressionado
    } else if ((GPIOC->IDR & (1 << SW17)) == 0) {
        return chave_id_press = 17;  // SW17 pressionado
		}
		return chave_id_press = 0;
}

uint16_t readADC(){
	while((ADC1->SR&(1<<1)) == 0); /* wait until the EOC flag is set */		
	return ADC1->DR;
}




void led_Thread1 (void const *argument);
void led_Thread3 (void const *argument);
void adc_Thread  (void const *argument);
void buzzer_Thread  (void const *argument);					
void control_mode  (void const *argument);					

osThreadDef(led_Thread1, osPriorityNormal, 1, 0);
osThreadDef(led_Thread3, osPriorityNormal, 1, 0);
osThreadDef(adc_Thread, osPriorityNormal, 1, 0);
osThreadDef(buzzer_Thread, osPriorityNormal, 1, 0);
osThreadDef(control_mode, osPriorityAboveNormal, 1, 0);

osThreadId T_led_ID1;
osThreadId T_led_ID3;	
osThreadId T_adc_ID;
osThreadId T_buzzer_ID;
osThreadId T_control_mode;

// Mutex para integridade dos dados
osMutexId mutex_id;           
osMutexDef(mutex_id); 
volatile uint16_t ADC_VALUE = 0;
volatile uint8_t toggleBuzzer =  0;
volatile uint8_t mode = 0;
#define velocidade_default 1000
volatile uint16_t velocidade = velocidade_default;
//void velocidade_Thread(void const *argument){}
void control_mode(void const *argument){
	for(;;){	
	ADC_VALUE = readADC();
	osSignalSet(T_buzzer_ID, 0x01);
	switch(chaves_ler_todas()){
		case 5: //SW5 Button C modo piscar funcao 1 
			mode = 1;	
			osSignalSet(T_led_ID1, 0x01); // Sinaliza thread 1
			break;
		case 6: //SW6 Button D modo piscar funcao 2
			mode = 2;
			osSignalSet(T_led_ID3, 0x01); // Sinaliza thread Gray Cod
			break;
		case 7: //SW7 Button E modo piscar funcao 3
			mode = 3;
			osSignalSet(T_adc_ID, 0x01); // Sinaliza thread Potenciômetro
			break;
		case 8: //SW8 Button F modo sonoro/mudo funcao 4
			osMutexWait(mutex_id, osWaitForever);
      toggleBuzzer = !toggleBuzzer; // Alterna entre som/mudo
			osMutexRelease(mutex_id);
		  osSignalSet(T_buzzer_ID, 0x01);
			break;
		case 1: 	//SW1 Button Y
			osDelay(300);
			velocidade = velocidade*2;
			break;
		case 2:		//SW2 Button A
			osDelay(300);
			velocidade = velocidade/2;
			break;
		case 3:		//SW3 Button X
			osDelay(300); //debounce
			velocidade = velocidade_default;
			break;
		case 4: 	//SW4 Button B
			osDelay(300);
			velocidade = ADC_VALUE;	
			break;
		case 15: //SW15 Button L
			velocidade = 500;
			break;
		case 16: //SW16 Button M
			velocidade = 2000;
			break;
		case 17: //SW17 Button N
			velocidade = 10000;
			break;
		case 14: //teste lcd
			lcd_command(POSICAO_00);
			lcd_print("Par e Impar");
			lcd_command(POSICAO_10);
			lcd_print("velocidade");
			break;
		default:
			break;
		}
	  osDelay(100); // Verifica o estado dos botões a cada 100ms
	}
}
/*-------------------------------------------------------------------------------
  Thread LED(F1.1 - piscar leds impares)
-------------------------------------------------------------------------------*/
void led_Thread1(void const *argument) {
    for (;;) {
			osSignalWait(0x01, osWaitForever);
			leds_apaga_todos();
			 while(mode == 1){				
        // Liga todos os LEDs impares
        for (int i = 1; i <= 8; i+=2) {
					ledsON(i);   
        }
				// Desliga todos os LEDs pares
        for (int i = 2; i <= 8; i+=2) {
					ledsOFF(i);
        }	
				osDelay(velocidade); //1 segundo
				 // Liga todos os LEDs pares
        for (int i = 2; i <= 8; i+=2) {
					ledsON(i);
        }
				 // Desliga todos os LEDs �mpares
        for (int i = 1; i <= 8; i+=2) {
					ledsOFF(i);
        }
				osDelay(velocidade); //1 segundo
				}
    }
}
/*-------------------------------------------------------------------------------
  Thread LED(F2 - piscar leds Gray)
-------------------------------------------------------------------------------*/
void led_Thread3(void const *argument) {	
		uint8_t gray_code[] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0111, 0b0101, 0b0100, 0b1100};
    for(;;) {
			osSignalWait(0x01, osWaitForever);
			leds_apaga_todos();
			while(mode == 2){
        for (int i = 0; i <= 8; i++) {
					if(i == 7){
						gray_code[i] = 0b1000;
					}
            ledsON(gray_code[i]);
						osDelay(velocidade);      
            ledsOFF(gray_code[i]);   
            osDelay(velocidade);  
        }
			}
    }
}
/*-------------------------------------------------------------------------------
  Thread POT_LED(F3 - Acender leds pelo barra do potenciometro)
-------------------------------------------------------------------------------*/
void adc_Thread(void const *argument) {
	uint8_t num_leds;
	for(;;){
    osSignalWait(0x01, osWaitForever);
		while(mode == 3){
			num_leds = (ADC_VALUE * 8) / 512; // 0 a 8 LEDs
		for (int i = 1; i <= 8; i++) {
			if (i <= num_leds) {
				ledsON(i);
			} else {
				ledsOFF(i);
			}
		}
		 // Sinaliza a Thread do Buzzer
    osSignalSet(T_buzzer_ID, 0x01);
		osDelay(100); // Atualização a cada 100 ms
		}
	}
}
/*-------------------------------------------------------------------------------
  Thread Buzzer(F4 - Trocar freq pelo num leds)
-------------------------------------------------------------------------------*/
void buzzer_Thread(void const *argument) {
    uint8_t local_freq_buzzer;
		uint8_t local_toggleBuzzer;
    for (;;) {
			// Aguarda sinal da Thread ADC
      osSignalWait(0x01, osWaitForever);
			// Protege a leitura de freq_buzzer
      osMutexWait(mutex_id, osWaitForever);
      local_freq_buzzer = ADC_VALUE; // Lê o valor atualizado
			local_toggleBuzzer = toggleBuzzer;
      osMutexRelease(mutex_id);
			if (local_toggleBuzzer && local_freq_buzzer > 0) {
				RCC->APB1ENR |= (1<<1);
				TIM3->CCR2 = 50;
				TIM3->CCER = 0x1 << 8; /*CC2P = 0, CC2E = 1 */
				TIM3->CCMR2 = 0x0030;  /* toggle channel 3 */
				TIM3->ARR = (local_freq_buzzer)*1047*2;				
				TIM3->CR1 = 1;
			 } 
			else {
				// Desliga o buzzer se nenhum LED estiver aceso
				TIM3->CR1 = 0;                      // Desativa o timer
				RCC->APB1ENR &= ~(1 << 1);          // Desabilita o clock
			}			
			osDelay(100);
		}
}

/*----------------------------------------------------------------------------
  Initilise and create the threads
 *---------------------------------------------------------------------------*/
int main(void) {
	osKernelInitialize (); 
	setup_RedPill(); 
	adc_init();
	lcd_init();
	
  mutex_id = osMutexCreate(osMutex(mutex_id));
	
	T_control_mode = osThreadCreate(osThread(control_mode), NULL);
	T_led_ID1 = osThreadCreate(osThread(led_Thread1), NULL); 
	T_led_ID3 = osThreadCreate(osThread(led_Thread3), NULL); 
	T_adc_ID = osThreadCreate(osThread(adc_Thread), NULL); 	
	T_buzzer_ID = osThreadCreate(osThread(buzzer_Thread), NULL); 		
	
	osKernelStart(); // Inicia o RTOS
	
	while (1) {}
}