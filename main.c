//  пример работы с NRF24LE1 - чтение-запись UART, моргание GPIO отсылка nRF и  чтение DHT

/* if we have DHT sensor */
#define myDHTPIN GPIO_PIN_ID_P1_4

#include "libs.h"
#include "nRFLE.c"



#include "uart.h"
#include "../src/uart/src/uart_configure_manual_baud_calc.c"


#define led_toggle()	gpio_pin_val_complement_sbit(P0_SB_D0)
#define led_on()		gpio_pin_val_set_sbit(P0_SB_D0)
#define led_off()		gpio_pin_val_clear_sbit(P0_SB_D0)


uint8_t dhtread (int DHTPIN,int *temp,int *hum) { // reading DTH11 22 sensors
unsigned char datadht[5];
unsigned char j = 0, i = 0;
	unsigned int crcdata;
    
datadht[0] = datadht[1] = datadht[2] = datadht[3] = datadht[4] = 0;  
   
 //pin as output and set 0
gpio_pin_configure(DHTPIN, GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT | GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR | GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);

delay_ms(18);

gpio_pin_val_set(DHTPIN); //set 1

//pin as input
gpio_pin_configure(DHTPIN,GPIO_PIN_CONFIG_OPTION_DIR_INPUT);

//=============check DHT response
delay_us(51);
if (gpio_pin_val_read(DHTPIN)) return 0;
delay_us(81);
if (!gpio_pin_val_read(DHTPIN)) return 0;

//===============receive 40 data bits

while (gpio_pin_val_read(DHTPIN));
for (j=0; j<5; j++)
    {
    datadht[j]=0;
    for(i=0; i<8; i++)
        {
        while (!gpio_pin_val_read(DHTPIN));	
        delay_us (30);
	  if (gpio_pin_val_read(DHTPIN)) 
            datadht[j]|=1<<(7-i); 
        while (gpio_pin_val_read(DHTPIN));
        }
    }
    
 crcdata= (datadht[0] + datadht[1] + datadht[2] + datadht[3]);
 if (datadht[0]==0 && datadht[1]==0 && datadht[2]==0 && datadht[3]==0) { // no sensor found
	 return 0; // failed
 }
 if ( crcdata != datadht[4]) return 0; // crc error
 if (datadht[1]==0 && datadht[3]==0) {
	    // dht11
	    *temp=datadht[2]*10; // умножение на 10,чтобы было однинаково как у dht22,можно убрать.
	    *hum=datadht[0]*10;
	  
	  }else  { // dht22
	     *temp= (datadht[0] * 256 + datadht[1]);
             *hum = ((datadht[2] & 0x7F)* 256 + datadht[3]);
             if (datadht[2] & 0x80)  return 0; // clientnf.temperature_Sensor *= -1;
	  }
	return 1;
}


int rf_read(unsigned char data[32]) {
int r = 0;
	int count;
		rf_irq_clear_all(); //clear all interrupts in the 24L01
		rf_set_as_rx(true); //change the device to an RX to get the character back from the other 24L01

		//wait a while to see if we get the data back (change the loop maximum and the lower if
		//  argument (should be loop maximum - 1) to lengthen or shorten this time frame
	 //led_off();
		for(count = 0; count < 25000; count++)
		{
		  
			if((rf_irq_pin_active() && rf_irq_rx_dr_active()))
			{
			  //state=1;
//if (clientnf.count <= 2147483646) clientnf.count++;      /// счетчик передач для контроля качества канала
//else 		.count = 0;

				rf_read_rx_payload(data //(const uint8_t*)&servernf
				       , 32); //get the payload into data
				r=1;
				break;
			
			}
			
			

			//if loop is on its last iteration, assume packet has been lost.
			//if(count == 24999) clientnf.Error_Message++;
		}



		rf_irq_clear_all(); //clear interrupts again

		rf_set_as_tx(); //resume normal operation as a TX
		//led_off();
return r;	
}

void rf_write(unsigned char  data[32]) {
// led_on();
	//(const uint8_t*)&clientnf
	 	rf_write_tx_payload(data, 32, true); //transmit received char over RF
                 delay_ms(1); 
		//wait until the packet has been sent or the maximum number of retries has been reached
		  //while(!(rf_irq_pin_active() && rf_irq_tx_ds_active())) delay_ms(1);  // zu - cleared by vovka
  //       led_off();
}



//====================main========================

 int mode = 0;
 
 void  uart_init() { // set 38400 
//Set up UART pins
	 
  gpio_pin_configure(GPIO_32P_PIN_ID_FUNC_RXD,
                  GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
                  GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS);

   gpio_pin_configure(GPIO_32P_PIN_ID_FUNC_TXD,
                  GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
                  GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_SET |
                  GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);

   //Set up UART for 38400 baud
   uart_configure_manual_baud_calc( UART_CONFIG_OPTION_ENABLE_RX |
                           UART_CONFIG_OPTION_MODE_1_UART_8_BIT |
                           UART_CONFIG_OPTION_CLOCK_FOR_MODES_1_3_USE_BR_GEN |
                           UART_CONFIG_OPTION_BIT_SMOD_SET,
                 1011 //0x03F3 - see datasheet on NRF24LE for possible values
	                    ); 
}

void putchar(char c)
{
   uart_send_wait_for_complete(c);
}

char getchar()
{
   unsigned char retchar;
   if (! (uart_rx_data_ready()) ) return 0;
   uart_wait_for_rx_and_get(&retchar);
   return retchar;
}

int cnt;
unsigned int val;
int i;

void rf_init(int chan) {
	radiobegin(); //
		setChannel(chan);
		setDataRate(2); // 1 - 250кб , 2 - 1 мб , 3 -2 мб.
		setAutoAck(false);
		setCRCLength(2); // 0 - crc off ,1 - 8bit ,2 - 16bit
		setPALevel(3) ; // мощность 0..3
		openAllPipe();
}

void main()
{
int state=0; int temp, hum;

unsigned int count; //counter for for loop
//unsigned int crcdata;


  //CLKCTRL=0;
  
gpio_pin_configure(GPIO_PIN_ID_P0_0, GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT | GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR | GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);
//gpio_pin_configure(GPIO_PIN_ID_P0_1, GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT | GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR | GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);
		
gpio_pin_configure(GPIO_PIN_ID_P0_6, GPIO_PIN_CONFIG_OPTION_DIR_INPUT); 

adc_configure (ADC_CONFIG_OPTION_RESOLUTION_10_BITS | ADC_CONFIG_OPTION_RESULT_JUSTIFICATION_RIGHT); //ADC_CONFIG_OPTION_REF_SELECT_VDD 
//adc_configure (ADC_CONFIG_OPTION_RESOLUTION_12_BITS);

//pwm_configure(PWM_CONFIG_OPTION_PRESCALER_VAL_10 || PWM_CONFIG_OPTION_WIDTH_8_BITS);

	/*
	
	-pinMode()
	gpio_pin_configure(GPIO_PIN_ID_P0_6, // укажем необходимые параметры
			GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
			GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR |
			GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);
	*/
	
uart_init();
rf_init(100); //open 100 chan (addreses already defined in nRLE.c)

led_on();  delay_ms(500);  led_off(); // welcome Blink on P0.0 
dhtread(myDHTPIN,&temp,&hum);
printf("nrf24le1 :) output ready press a key to report temperature  (now: %d,%d)\r\n",temp,hum);

while(1)
	{
	  
//read_sensor();  //putchar('*');
		
		
		//val = adc_start_single_conversion_get_value(ADC_CHANNEL_AIN6); //GPIO_PIN_ID_P0_1
		/*for(i=0;i<20;i++) {
		val = adc_start_single_conversion_get_value(i) ; //ADC_CHANNEL_AIN6);
		printf("val%d=%u   PIN=%d %d\r\n",i,val,ADC_CHANNEL_AIN6,GPIO_PIN_ID_P0_6);
		printf("val%d=%u   PIN2=%d %d\r\n",i,val,ADC_CHANNEL_AIN1,GPIO_PIN_ID_P0_6);
			
		}
		*/
		
		//gpio_pin_val_set(GPIO_PIN_ID_P0_6); //установка 1
	 delay_ms(50); 
             //gpio_pin_val_clear(GPIO_PIN_ID_P0_6); //установка 0
	 delay_ms(50); 
		
		val=0;
		val=getchar();
if (val) {
   dhtread(myDHTPIN,&temp,&hum);
   printf("nrf24out2 step:%d val=%u ch=%c temp=%d hum=%d\r\n",cnt,val,val,temp,hum); cnt++;
   }

{ // reading RF
unsigned char  d[32];

if (rf_read(d)) { // send resp back  if we have recv OK
	dhtread(myDHTPIN,&temp,&hum);
	d[5]=temp/10; d[6]=hum/10; // pack it
	rf_write(d); //just echo back
	 //mode = !mode;	 if (mode) led_on(); else led_off(); // blinking
	led_on(); delay_ms(100); led_off(); // and blinking
        }
}
	 
	
		
delay_ms(1);
	
	} //end loop
}