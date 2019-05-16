/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include <string.h>
#include <assert.h>
#include "asf.h"

// Descomente o define abaixo, para desabilitar o Bluetooth e utilizar modo Serial via Cabo
//#define DEBUG_SERIAL


#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

#define LED_PIO_ID	   ID_PIOC
#define LED_PIO        PIOC
#define LED_PIN		   8
#define LED_PIN_MASK   (1<<LED_PIN)

#define LED1_PIO           PIOB
#define LED1_PIO_ID        11
#define LED1_PIO_IDX       4u
#define LED1_PIO_IDX_MASK  (1u << LED1_PIO_IDX)

#define LED2_PIO           PIOA
#define LED2_PIO_ID        10
#define LED2_PIO_IDX       6u
#define LED2_PIO_IDX_MASK  (1u << LED2_PIO_IDX)

#define LED3_PIO           PIOA
#define LED3_PIO_ID        10
#define LED3_PIO_IDX       21u
#define LED3_PIO_IDX_MASK  (1u << LED3_PIO_IDX)

#define BUT1_PIO           PIOA
#define BUT1_PIO_ID        10
#define BUT1_PIO_IDX       2u
#define BUT1_PIO_IDX_MASK (1u << BUT1_PIO_IDX)

#define BUT2_PIO           PIOA
#define BUT2_PIO_ID        10
#define BUT2_PIO_IDX       3u
#define BUT2_PIO_IDX_MASK (1u << BUT2_PIO_IDX)

#define BUT3_PIO           PIOA
#define BUT3_PIO_ID        10
#define BUT3_PIO_IDX       4u
#define BUT3_PIO_IDX_MASK (1u << BUT3_PIO_IDX)

#define BUT_DEBOUNCING_VALUE  100

#define MAX_SIZE 32

#define VOLT_REF        (3300)
#define MAX_DIGITAL     (4095)

/** The conversion data is done flag */
volatile bool g_is_conversion_done = false;
volatile bool g_is_res_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;
volatile uint32_t g_res_value = 0;

volatile bool g_delay = false;

/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 2
#define AFEC_CHANNEL_RES_PIN 5

volatile long g_systimer = 0;
volatile char lista[MAX_SIZE][3];
volatile int n = 0;
volatile char eop = '-';
volatile int prev_volume;

void BUT_RISE(char button){
	
	lista[n][0] = button;
	lista[n][1] = '0';
	lista[n][2] = eop;
	
	n++;
	
}

void BUT_FALL(char button){
	
	lista[n][0] = button;
	lista[n][1] = '1';
	lista[n][2] = eop;
	
	n++;
	
}

void BUT1_Handler(){
	if(pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK)){
		BUT_RISE('q');
		pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
	}
	else{
		BUT_FALL('q');
		pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
	}
}

void BUT2_Handler(){
	if(pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIO_IDX_MASK)){
		BUT_RISE('w');
		pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
	}
	else{
		BUT_FALL('w');
		pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
	}
}

void BUT3_Handler(){
	if(pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIO_IDX_MASK)){
		BUT_RISE('e');
		pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
	}
	else{
		BUT_FALL('e');
		pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
	}
}

void BUT_init(){
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	
	pio_set_input(BUT1_PIO,BUT1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT2_PIO,BUT2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT3_PIO,BUT3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	pio_pull_up(BUT1_PIO, BUT1_PIO_IDX_MASK, 1);
	pio_pull_up(BUT2_PIO, BUT2_PIO_IDX_MASK, 1);
	pio_pull_up(BUT3_PIO, BUT3_PIO_IDX_MASK, 1);
	
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_PIO_IDX_MASK);
	
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIO_IDX_MASK, PIO_IT_EDGE, BUT1_Handler);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIO_IDX_MASK, PIO_IT_EDGE, BUT2_Handler);
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIO_IDX_MASK, PIO_IT_EDGE, BUT3_Handler);
	
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_EnableIRQ(BUT3_PIO_ID);
	
	NVIC_SetPriority(BUT1_PIO_ID, 1);
	NVIC_SetPriority(BUT2_PIO_ID, 1);
	NVIC_SetPriority(BUT3_PIO_ID, 1);
}

void LED_init(){
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	
	pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 0, 0, 0);
	
	pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
	pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
	pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
}

void SysTick_Handler() {
	g_systimer++;
}

static void AFEC_Temp_callback(void)
{
	g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	g_is_conversion_done = true;
}

static void AFEC_Res_callback(void)
{
	g_res_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_RES_PIN);
	g_is_res_done = true;
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	afec_start_software_conversion(AFEC0);

}

void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
	afec_start_software_conversion(AFEC0);
	
}

/**
 * converte valor lido do ADC para temperatura em graus celsius
 * input : ADC reg value
 * output: Temperature in celsius
 */
static int32_t convert_adc_to_temp(int32_t ADC_value){

  int32_t ul_vol;
  int32_t ul_temp;

  /*
   * converte bits -> tens?o (Volts)
   */
	ul_vol = ADC_value * VOLT_REF / (float) MAX_DIGITAL;

  /*
   * According to datasheet, The output voltage VT = 0.72V at 27C
   * and the temperature slope dVT/dT = 2.33 mV/C
   */
  ul_temp = (ul_vol - 720)  * 100 / 233 + 27;
  return(ul_temp);
}

static int32_t convert_adc_to_res(int32_t ADC_value){

  int32_t ul_vol;
  int32_t ul_res;

  /*
   * converte bits -> tens?o (Volts)
   */
	ul_vol = ADC_value * VOLT_REF / (float) MAX_DIGITAL;

  /*
   * According to datasheet, The output voltage VT = 0.72V at 27C
   * and the temperature slope dVT/dT = 2.33 mV/C
   */
  ul_res = (ul_vol - 720)  * 100 / 233 + 27; //MUDAR
  return(ul_res);
}

static int32_t convert_adc_to_axis(int32_t ADC_value){
	
	return (ADC_value-2200);
}

static int32_t convert_adc_to_volume(int32_t ADC_value){
	
	return (ADC_value*100 / MAX_DIGITAL);
}

static void config_ADC_TEMP_RES(void){
/*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_2,	AFEC_Temp_callback, 1);
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_5,	AFEC_Res_callback, 1);

	/*** Configuracao espec?fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, 0x200);
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_RES_PIN, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

	/* Selecina canal e inicializa convers?o */
	//afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	//afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
}



void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter ? meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, ((ul_sysclk) / ul_div) / freq);

	/* Configura e ativa interrup?c?o no TC canal 0 */
	/* Interrup??o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void config_console(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
	
	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			//timestart = g_systimer; // reset timeout
			buffer[counter++] = rx;
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}

void hc05_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	 // RX - PB0  TX - PB1 
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
	char buffer_rx[128];
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);	
	usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEKeyboard", 1000);
	usart_log("hc05_server_init", buffer_rx);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+PIN1234", 1000);
	usart_log("hc05_server_init", buffer_rx);
}

void sendBT(char lista[3]){
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, lista[0]);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, lista[1]);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, lista[2]);
}


int main (void)
{
	board_init();
	sysclk_init();
	BUT_init();
	LED_init();
	delay_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_console();
	config_ADC_TEMP_RES();
	
	TC_init(TC0, ID_TC1, 1, 1);
	TC_init(TC0, ID_TC0, 0, 10);
	
	#ifndef DEBUG_SERIAL
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	
	char buffer[1024];
	
	afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
	afec_start_software_conversion(AFEC0);
	if (g_is_conversion_done){
		afec_channel_enable(AFEC0, AFEC_CHANNEL_RES_PIN);
		afec_start_software_conversion(AFEC0);
	}
	
	while(1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		while(n > 0){
			
			printf("Botao: %c\n", lista[0][0]);
			sendBT(lista[0]);
			for(int i = 1; i < n; i++){
				lista[i - 1][0] = lista[i][0];
				lista[i - 1][1] = lista[i][1];
				lista[i - 1][2] = lista[i][2];
			}
			n--;

		}
		
		if (g_is_conversion_done==true){
			
			int volume = convert_adc_to_volume(g_res_value);
			if(volume != prev_volume){
				printf("Volume: %d\n", volume);
				char volume_char = (char) volume;
				lista[n][0] = 'v';
				lista[n][1] = volume_char;
				lista[n][2] = eop;
				n++;
			}
			prev_volume = volume;
			g_is_conversion_done = false;
		}
		if(g_is_res_done==true){
			g_is_res_done = false;
		}
	}
}
