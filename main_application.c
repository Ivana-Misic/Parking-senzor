/* Standard includes. */
#include <stdio.h>
#include <conio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/* Hardware simulator utility functions */
#include "HW_access.h"

/* SERIAL SIMULATOR CHANNEL */
#define COM_CH_0 (0)
#define COM_CH_1 (1)
#define R_BUF_SIZE (16)
#define MAX_QUEUE_SIZE (16)
#define QUEUE_MAX_TICKS_TO_WAIT (10)

/* Temporary constants */
#define MAX_SENSOR_VALUE (80)
#define MIN_SENSOR_VALUE (10)

/* TASK PRIORITIES */
#define	TASK_SERIAL_SEND_PRI		(2 + tskIDLE_PRIORITY)
#define TASK_SERIAL_REC_PRI			(3 + tskIDLE_PRIORITY)
#define	SERVICE_TASK_PRI			(1 + tskIDLE_PRIORITY)
#define	DISPLAY_TASK_PRI			(4 + tskIDLE_PRIORITY)

/* TASKS: FORWARD DECLARATIONS */
void led_blink(void* pvParameters);
void SerialSend_Task(void* pvParameters);
void SerialReceive_Task(void* pvParameters);
void ShowDataOn7Seg(void* pvParameters);
//void updateGlobalSensorValues(void* pvParameters);

/* Helper functions */
uint8_t checkInterval(uint8_t value);

/* TRASNMISSION DATA - CONSTANT IN THIS APPLICATION */
const char trigger[] = "S";

/* Gloabal variables containging read-only data from sensors */
uint8_t left_sensor = 0;
uint8_t right_sensor = 0;

/* RECEPTION DATA BUFFER */
//uint8_t rx_buffer_ch_0[R_BUF_SIZE];
//unsigned volatile r_point;

/* 7-SEG NUMBER DATABASE - ALL HEX DIGITS */
static const unsigned char hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
										0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };

/* Enumeration types */

/* Structures used in the project */
typedef struct tx_packet {
	uint8_t channel_num;
	SemaphoreHandle_t* sempahore;
} tx_packet;

typedef struct rx_packet {
	uint8_t channel_num;
	SemaphoreHandle_t* sempahore;
	QueueHandle_t* queue;
} rx_packet;

typedef struct LED_struct {
	uint8_t led_select;
	uint8_t mask;
	int time_to_wait;
} t_LED;

/* GLOBAL OS-HANDLES */
SemaphoreHandle_t LED_BinarySemaphore_0;
SemaphoreHandle_t LED_BinarySemaphore_1;
SemaphoreHandle_t TX_BinarySemaphore_0;
SemaphoreHandle_t TX_BinarySemaphore_1;
SemaphoreHandle_t RX_BinarySemaphore_0;
SemaphoreHandle_t RX_BinarySemaphore_1;
TimerHandle_t tH0;
QueueHandle_t rx2seg7_queue_0;
QueueHandle_t rx2seg7_queue_1;
xTaskHandle led_tsk_0_freq_1_Hz_1_ON;
xTaskHandle led_tsk_0_freq_2_Hz_4_ON;
xTaskHandle led_tsk_0_freq_2_Hz_8_ON;
xTaskHandle led_tsk_1_freq_1_Hz_1_ON;
xTaskHandle led_tsk_1_freq_2_Hz_4_ON;
xTaskHandle led_tsk_1_freq_2_Hz_8_ON;

// ISR - signals that an input LED has been switched off or on
//uint32_t OnLED_ChangeInterrupt() {
//
//	BaseType_t higherPriorityTaskWoken = pdFALSE;
//
//	xSemaphoreGiveFromISR(LED_BinarySemaphore, &higherPriorityTaskWoken);
//
//	portYIELD_FROM_ISR(higherPriorityTaskWoken);
//}

// ISR - signals that a new message has arrived
uint32_t prvProcessRXInterrupt() {

	BaseType_t higherPriorityTaskWoken = pdFALSE;

	if (get_RXC_status(COM_CH_0) != 0) {
		if (xSemaphoreGiveFromISR(RX_BinarySemaphore_0, &higherPriorityTaskWoken) != pdTRUE) {
			printf("Error\n");
		}
	}
	if (get_RXC_status(COM_CH_1) != 0) {

		if (xSemaphoreGiveFromISR(RX_BinarySemaphore_1, &higherPriorityTaskWoken) != pdTRUE) {
			printf("Error\n");
		}
	}

	portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

uint8_t checkInterval(uint8_t value) {

	uint8_t scaled_max = (uint8_t)MAX_SENSOR_VALUE - (uint8_t)MIN_SENSOR_VALUE;
	uint8_t half_val = (uint8_t)((float)(scaled_max) * 0.5);
	uint8_t scaled_value = value - (uint8_t)MIN_SENSOR_VALUE;

	if (value < (uint8_t)MIN_SENSOR_VALUE) {
		return 0;
	}
	else if (scaled_value < half_val) {
		return 1;
	}
	else if (scaled_value < scaled_max) {
		return 2;
	}
	else
		return 3;
}

void led_blink(void* pvParameters) {

	t_LED* led_st = (t_LED*)pvParameters;
	uint8_t LED_value;

	while (1) {
		get_LED_BAR(led_st->led_select, &LED_value);
		LED_value = (~LED_value) & led_st->mask;
		set_LED_BAR(led_st->led_select, LED_value);
		vTaskDelay(pdMS_TO_TICKS(led_st->time_to_wait));		// Wait for 500 ms
	}
}

void SerialReceive_Task(void* pvParameters)
{
	rx_packet* packet = (rx_packet*)pvParameters;
	uint8_t cc = 0;
	uint8_t pointer = 0;
	char buffer[R_BUF_SIZE];

	for (;;)
	{
		xSemaphoreTake(*(packet->sempahore), portMAX_DELAY);	// waiting for interrupt
		get_serial_character(packet->channel_num, &cc);			// recieiving data item
		//printf("primio karakter: %u\n", (unsigned)cc);		// prikazuje primljeni karakter u cmd prompt
		if (cc == 13) {
			buffer[pointer] = 0;
			pointer = 0;
			xQueueSend(*(packet->queue), (void*)buffer, (TickType_t)0);

			if (packet->channel_num == COM_CH_0) {
				left_sensor = (buffer[0] - 48) * 10 + (buffer[1] - 48);
			}
			else if (packet->channel_num == COM_CH_1) {
				right_sensor = (buffer[0] - 48) * 10 + (buffer[1] - 48);
			}
		}
		else if (pointer < R_BUF_SIZE) {
			buffer[pointer++] = cc;
		}
	}
}

void SerialSend_Task(void* pvParameters)
{
	uint8_t t_point = 0;
	tx_packet* packet = (tx_packet*)pvParameters;

	for (;;)
	{
		xSemaphoreTake(*(packet->sempahore), portMAX_DELAY);						// waiting for timer to update the semaphore

		for (t_point = 0; t_point < sizeof(trigger); t_point++) {					// Sending the message
			send_serial_character(packet->channel_num, trigger[t_point++]);
		}
	}
}

void ShowDataOn7Seg(void* pvParameters) {

	char msg_0[R_BUF_SIZE];
	char msg_1[R_BUF_SIZE];

	for (;;) {
		if (xQueueReceive(rx2seg7_queue_0, msg_0, (TickType_t)QUEUE_MAX_TICKS_TO_WAIT) == pdPASS)	{
			if (xQueueReceive(rx2seg7_queue_1, msg_1, (TickType_t)QUEUE_MAX_TICKS_TO_WAIT) == pdPASS) {

				//printf("%c%c\n", msg_0[0], msg_0[1]);
				//printf("%c%c\n", msg_1[0], msg_1[1]);

				select_7seg_digit(0);
				set_7seg_digit(hexnum[msg_0[0] - 48]);
				select_7seg_digit(1);
				set_7seg_digit(hexnum[msg_0[1] - 48]);
				select_7seg_digit(2);
				set_7seg_digit(hexnum[msg_1[0] - 48]);
				select_7seg_digit(3);
				set_7seg_digit(hexnum[msg_1[1] - 48]);
			}
		}
	}
}

static void TimerCallback( TimerHandle_t xTimer ) {

	static uint8_t timer_0 = 0;
	uint8_t interval_range_0, interval_range_1;
	
	if (timer_0++ > 39) {
		xSemaphoreGive(TX_BinarySemaphore_0);	// Serial Communication Channel 0
		xSemaphoreGive(TX_BinarySemaphore_1);	// Serial Communication Channel 1
		timer_0 = 0;
	}

	// printf("%d\n", timer_0);

	interval_range_0 = checkInterval(left_sensor);
	interval_range_1 = checkInterval(right_sensor);

	printf("Interval Range 0 is : %d    %d\n", interval_range_0, left_sensor);
	printf("Interval Range 1 is : %d    %d\n", interval_range_1, right_sensor);

	// Diodes for the left sensor //

	switch (interval_range_0) {
	case 0:	// value < MIN 
		vTaskSuspend(led_tsk_0_freq_1_Hz_1_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_4_ON);
		vTaskResume(led_tsk_0_freq_2_Hz_8_ON);
		break;
	case 1:	// MIN < value < HALF
		vTaskSuspend(led_tsk_0_freq_1_Hz_1_ON);
		vTaskResume(led_tsk_0_freq_2_Hz_4_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_8_ON);
		break;
	case 2:	// HALF < value < MAX
		vTaskResume(led_tsk_0_freq_1_Hz_1_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_4_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_8_ON);
		break;
	case 3: // MAX < value
		set_LED_BAR(1, 0x00);
		vTaskSuspend(led_tsk_0_freq_1_Hz_1_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_4_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_8_ON);
		break;
	default:
		set_LED_BAR(1, 0x00);
		vTaskSuspend(led_tsk_0_freq_1_Hz_1_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_4_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_8_ON);
	}

	// Diodes for the right sensor //

	switch (interval_range_1) {
	case 0:
		vTaskResume(led_tsk_1_freq_2_Hz_8_ON);
		vTaskSuspend(led_tsk_1_freq_1_Hz_1_ON);
		vTaskSuspend(led_tsk_1_freq_2_Hz_4_ON);
		break;
	case 1:
		vTaskResume(led_tsk_1_freq_2_Hz_4_ON);
		vTaskSuspend(led_tsk_1_freq_1_Hz_1_ON);
		vTaskSuspend(led_tsk_1_freq_2_Hz_8_ON);
		break;
	case 2:
		vTaskResume(led_tsk_1_freq_1_Hz_1_ON);
		vTaskSuspend(led_tsk_1_freq_2_Hz_4_ON);
		vTaskSuspend(led_tsk_1_freq_2_Hz_8_ON);
		break;
	case 3:
		set_LED_BAR(2, 0x00);
		vTaskSuspend(led_tsk_1_freq_1_Hz_1_ON);
		vTaskSuspend(led_tsk_1_freq_2_Hz_4_ON);
		vTaskSuspend(led_tsk_1_freq_2_Hz_8_ON);
		break;
	default:
		set_LED_BAR(2, 0x00);
		vTaskSuspend(led_tsk_1_freq_1_Hz_1_ON);
		vTaskSuspend(led_tsk_1_freq_2_Hz_4_ON);
		vTaskSuspend(led_tsk_1_freq_2_Hz_8_ON);
	}

}

/* MAIN - SYSTEM STARTUP POINT */
void main_demo(void)
{
	
	init_LED_comm();						// Initializing LED bar
	set_LED_BAR(1, 0x00);					// Switching off all of the output LEDs
	set_LED_BAR(2, 0x00);					// Switching off all of the output LEDs
	init_serial_uplink(COM_CH_0);			// Initializing Tx for channel 0
	init_serial_downlink(COM_CH_0);			// Initializing Rx for channel 0
	init_serial_uplink(COM_CH_1);			// Initializing Tx for channel 1
	init_serial_downlink(COM_CH_1);			// Initializing Rx for channel 1
	init_7seg_comm();						// Initializing 7seg display

	/* ON INPUT CHANGE INTERRUPT HANDLER */
	// vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);
	/* SERIAL RECEPTION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXInterrupt);

	/* Create LED interrapt semaphore */
	LED_BinarySemaphore_0 = xSemaphoreCreateBinary();
	LED_BinarySemaphore_1 = xSemaphoreCreateBinary();
	RX_BinarySemaphore_0 = xSemaphoreCreateBinary();
	RX_BinarySemaphore_1 = xSemaphoreCreateBinary();
	TX_BinarySemaphore_0 = xSemaphoreCreateBinary();
	TX_BinarySemaphore_1 = xSemaphoreCreateBinary();

	/* Queues */
	rx2seg7_queue_0 = xQueueCreate(MAX_QUEUE_SIZE, R_BUF_SIZE);
	rx2seg7_queue_1 = xQueueCreate(MAX_QUEUE_SIZE, R_BUF_SIZE);

	/* Packets to be sent */
	tx_packet tx_packet_channel_0;
	tx_packet tx_packet_channel_1;
	tx_packet_channel_0.channel_num = COM_CH_0;
	tx_packet_channel_0.sempahore = &TX_BinarySemaphore_0;
	tx_packet_channel_1.channel_num = COM_CH_1;
	tx_packet_channel_1.sempahore = &TX_BinarySemaphore_1;

	rx_packet rx_packet_channel_0;
	rx_packet rx_packet_channel_1;
	rx_packet_channel_0.channel_num = COM_CH_0;
	rx_packet_channel_0.sempahore = &RX_BinarySemaphore_0;
	rx_packet_channel_0.queue = &rx2seg7_queue_0;
	rx_packet_channel_1.channel_num = COM_CH_1;
	rx_packet_channel_1.sempahore = &RX_BinarySemaphore_1;
	rx_packet_channel_1.queue = &rx2seg7_queue_1;

	t_LED led_st_0_freq_1_Hz_1_ON;
	t_LED led_st_0_freq_2_Hz_4_ON;
	t_LED led_st_0_freq_2_Hz_8_ON;
	t_LED led_st_1_freq_1_Hz_1_ON;
	t_LED led_st_1_freq_2_Hz_4_ON;
	t_LED led_st_1_freq_2_Hz_8_ON;

	led_st_0_freq_1_Hz_1_ON.led_select = 1;
	led_st_0_freq_1_Hz_1_ON.mask = 0x01;
	led_st_0_freq_1_Hz_1_ON.time_to_wait = 500;

	led_st_0_freq_2_Hz_4_ON.led_select = 1;
	led_st_0_freq_2_Hz_4_ON.mask = 0x0f;
	led_st_0_freq_2_Hz_4_ON.time_to_wait = 250;

	led_st_0_freq_2_Hz_8_ON.led_select = 1;
	led_st_0_freq_2_Hz_8_ON.mask = 0xff;
	led_st_0_freq_2_Hz_8_ON.time_to_wait = 250;

	led_st_1_freq_1_Hz_1_ON.led_select = 2;
	led_st_1_freq_1_Hz_1_ON.mask = 0x01;
	led_st_1_freq_1_Hz_1_ON.time_to_wait = 500;

	led_st_1_freq_2_Hz_4_ON.led_select = 2;
	led_st_1_freq_2_Hz_4_ON.mask = 0x0f;
	led_st_1_freq_2_Hz_4_ON.time_to_wait = 250;

	led_st_1_freq_2_Hz_8_ON.led_select = 2;
	led_st_1_freq_2_Hz_8_ON.mask = 0xff;
	led_st_1_freq_2_Hz_8_ON.time_to_wait = 250;

	/* led bar TASK */
	if (xTaskCreate(led_blink, "LED_0_freq_1_Hz_1_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_0_freq_1_Hz_1_ON), DISPLAY_TASK_PRI, &led_tsk_0_freq_1_Hz_1_ON) != pdPASS)
		printf("LED_0_freq_1_Hz_1_ON was not created!\n");
	if(xTaskCreate(led_blink, "LED_0_freq_2_Hz_4_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_0_freq_2_Hz_4_ON), DISPLAY_TASK_PRI, &led_tsk_0_freq_2_Hz_4_ON) != pdPASS)
		printf("LED_0_freq_2_Hz_4_ON was not created!\n");
	if(xTaskCreate(led_blink, "LED_0_freq_2_Hz_8_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_0_freq_2_Hz_8_ON), DISPLAY_TASK_PRI, &led_tsk_0_freq_2_Hz_8_ON) != pdPASS)
		printf("LED_0_freq_2_Hz_8_ON was not created!\n");
	if(xTaskCreate(led_blink, "LED_1_freq_1_Hz_1_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_1_freq_1_Hz_1_ON), DISPLAY_TASK_PRI, &led_tsk_1_freq_1_Hz_1_ON) != pdPASS)
		printf("LED_1_freq_1_Hz_1_ON was not created!\n");
	if(xTaskCreate(led_blink, "LED_1_freq_2_Hz_4_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_1_freq_2_Hz_4_ON), DISPLAY_TASK_PRI, &led_tsk_1_freq_2_Hz_4_ON) != pdPASS)
		printf("LED_1_freq_2_Hz_4_ON was not created!\n");
	if(xTaskCreate(led_blink, "LED_1_freq_2_Hz_8_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_1_freq_2_Hz_8_ON), DISPLAY_TASK_PRI, &led_tsk_1_freq_2_Hz_8_ON) != pdPASS)
		printf("LED_1_freq_2_Hz_8_ON was not created!\n");

	/* SERIAL RECEIVER TASK */
	xTaskCreate(SerialReceive_Task, "SensorRx_CH0", configMINIMAL_STACK_SIZE, (void*)(&rx_packet_channel_0), TASK_SERIAL_REC_PRI, NULL);
	xTaskCreate(SerialReceive_Task, "SensorRx_CH1", configMINIMAL_STACK_SIZE, (void*)(&rx_packet_channel_1), TASK_SERIAL_REC_PRI, NULL);
	/* SERIAL TRANSMITTER TASK */
	xTaskCreate(SerialSend_Task, "SensorTx_CH0", configMINIMAL_STACK_SIZE, (void*)(&tx_packet_channel_0), TASK_SERIAL_SEND_PRI, NULL);
	xTaskCreate(SerialSend_Task, "SensorTx_CH1", configMINIMAL_STACK_SIZE, (void*)(&tx_packet_channel_1), TASK_SERIAL_SEND_PRI, NULL);
	/* PROCESSING TASKS */
	xTaskCreate(ShowDataOn7Seg, "7SegDisplay", configMINIMAL_STACK_SIZE, (void*)0, DISPLAY_TASK_PRI, NULL);
	

	/* Timers */
	tH0 = xTimerCreate(
		"Timer",
		pdMS_TO_TICKS(5),
		pdTRUE,
		0,
		TimerCallback
	);
	 
	xTimerStart(tH0, 0);
	vTaskStartScheduler();
}