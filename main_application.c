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
#define COM_CH_PC (2)
#define R_BUF_SIZE (16)
#define MAX_QUEUE_SIZE (16)
#define QUEUE_MAX_TICKS_TO_WAIT (10)
#define IDLE_TASK_PRIORITY (0)

/* TASK PRIORITIES */
#define	TASK_SERIAL_SEND_PRI		(1 + IDLE_TASK_PRIORITY)
#define TASK_SERIAL_REC_PRI			(6 + IDLE_TASK_PRIORITY)
#define TASK_SERIAL_REC_PRI_PC	    (5 + IDLE_TASK_PRIORITY)
#define	DISPLAY_TASK_PRI			(2 + IDLE_TASK_PRIORITY)
#define	LED_TASK_PRI				(3 + IDLE_TASK_PRIORITY)
#define	PC_TASK_PRI					(4 + IDLE_TASK_PRIORITY)

/* TASKS: FORWARD DECLARATIONS */
void led_blink(void* pvParameters);
void SerialSend_Task(void* pvParameters);
void SerialSend_Task_PC(void* pvParameters);
void SerialReceive_Task(void* pvParameters);
void SerialReceive_Task_PC(void* pvParameters);
void ShowDataOn7Seg(void* pvParameters);
void PC_control_Task(void* pvParameters);

/* Helper functions */
uint8_t checkInterval(uint8_t value, uint8_t max, uint8_t min);
uint8_t assignValueToFSMState(char* msg);

/* TRASNMISSION DATA - CONSTANT IN THIS APPLICATION */
const char trigger[] = "s";

/* Gloabal variables */
uint8_t left_sensor = 0;
uint8_t right_sensor = 0;
uint8_t start_stop = 0;
uint8_t left_sensor_max_value = 0;
uint8_t left_sensor_min_value = 0;
uint8_t right_sensor_max_value = 0;
uint8_t right_sensor_min_value = 0;

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
SemaphoreHandle_t RX_BinarySemaphore_PC;
TimerHandle_t tH0;
TimerHandle_t tH1;
QueueHandle_t rx2seg7_queue_0;
QueueHandle_t rx2seg7_queue_1;
QueueHandle_t pc_comm_queue;
QueueHandle_t pc_tx_queue;
xTaskHandle led_tsk_0_freq_1_Hz_1_ON;
xTaskHandle led_tsk_0_freq_2_Hz_4_ON;
xTaskHandle led_tsk_0_freq_2_Hz_8_ON;
xTaskHandle led_tsk_1_freq_1_Hz_1_ON;
xTaskHandle led_tsk_1_freq_2_Hz_4_ON;
xTaskHandle led_tsk_1_freq_2_Hz_8_ON;

// ISR - signals that an input LED has been switched off or on
uint32_t OnLED_ChangeInterrupt() {
	uint8_t LED_value;
	get_LED_BAR(0, &LED_value);
	if (LED_value & 0x01) {
		start_stop = 1;
		printf("The system has been started by using an LED\n");
	}
	else {
		start_stop = 0;
		printf("The system has been stopped by using an LED\n");
	}
}

/* ISR - signals that a new message has arrived */
uint32_t prvProcessRXInterrupt() {

	BaseType_t higherPriorityTaskWoken = pdFALSE;

	if (get_RXC_status(COM_CH_0) != 0) {
		if (xSemaphoreGiveFromISR(RX_BinarySemaphore_0, &higherPriorityTaskWoken) != pdTRUE) {
			printf("Error Channel 0\n");
		}
	}
	if (get_RXC_status(COM_CH_1) != 0) {

		if (xSemaphoreGiveFromISR(RX_BinarySemaphore_1, &higherPriorityTaskWoken) != pdTRUE) {
			printf("Error Channel 1\n");
		}
	}
	if (get_RXC_status(COM_CH_PC) != 0) {

		if (xSemaphoreGiveFromISR(RX_BinarySemaphore_PC, &higherPriorityTaskWoken) != pdTRUE) {
			printf("Error Channel 2\n");
		}
	}

	portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

uint8_t checkInterval(uint8_t value, uint8_t max, uint8_t min) {

	/* min = 10
	   max = 80
	   half_val = 35 + 10 = 45
	   value = 68
	*/
	uint8_t half_val = max - min;
	half_val = half_val / 2;
	half_val += min;

	if (value < min) {
		return 0;
	}
	else if (value < half_val) {
		return 1;
	}
	else if (value < max) {
		return 2;
	}
	else
		return 3;
}

uint8_t assignValueToFSMState(char* str) {

	/*
		"START"	= 0 
		"STOP_" = 1
		"CMAX0" = 2
		"CMIN0" = 3
		"CMAX1" = 4
		"CMIN1" = 5
	*/

	char msg[6] = {str[0], str[1], str[2], str[3], str[4], 0};

	if (!strcmp(msg, "START")) {
		return 0;
	}
	if (!strcmp(msg, "STOP_")) {
		return 1;
	}
	if (!strcmp(msg, "CMAX0")) {
		return 2;
	}
	if (!strcmp(msg, "CMIN0")) {
		return 3;
	}
	if (!strcmp(msg, "CMAX1")) {
		return 4;
	}
	if (!strcmp(msg, "CMIN1")) {
		return 5;
	}
	return 1;
}

void PC_control_Task(void* pvParameters) {

	char msg[R_BUF_SIZE];
	uint8_t command = 1;

	while (1) {

		if (xQueueReceive(pc_comm_queue, msg, (TickType_t)QUEUE_MAX_TICKS_TO_WAIT) == pdPASS) {
			command = assignValueToFSMState(msg);

			/*
				"START"	= 0
				"STOP_" = 1
				"CMAX0" = 2
				"CMIN0" = 3
				"CMAX1" = 4
				"CMIN1" = 5
			*/

			switch (command) {
			case 0:
				start_stop = 1;
				printf("Current state is START and the value of start_stop is %d\n", start_stop);
				break;
			case 1:
				start_stop = 0;
				printf("Current state is STOP and the value of start_stop is %d\n", start_stop);
				break;
			case 2:
				left_sensor_max_value = 10 * (msg[5] - 48) + (msg[6] - 48);
				printf("Current state is CMAX0 and the value of left_sensor_max_value is %d\n", left_sensor_max_value);
				break;
			case 3:
				left_sensor_min_value = 10 * (msg[5] - 48) + (msg[6] - 48);
				printf("Current state is CMIN0 and the value of left_sensor_min_value is %d\n", left_sensor_min_value);
				break;
			case 4:
				right_sensor_max_value = 10 * (msg[5] - 48) + (msg[6] - 48);
				printf("Current state is CMAX1 and the value of right_sensor_max_value is %d\n", right_sensor_max_value);
				break;
			case 5:
				right_sensor_min_value = 10 * (msg[5] - 48) + (msg[6] - 48);
				printf("Current state is CMIN1 and the value of right_sensor_min_value is %d\n", right_sensor_min_value);
				break;
			}
		}

	}
}

void led_blink(void* pvParameters) {

	t_LED* led_st = (t_LED*)pvParameters;
	uint8_t LED_value;

	while (1) {

		if (start_stop) {
			get_LED_BAR(led_st->led_select, &LED_value);
			LED_value = (~LED_value) & led_st->mask;
			set_LED_BAR(led_st->led_select, LED_value);
			vTaskDelay(pdMS_TO_TICKS(led_st->time_to_wait));
		}
		else {
			set_LED_BAR(led_st->led_select, 0x00);
		}
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
		xSemaphoreTake(*(packet->sempahore), portMAX_DELAY);	/* waiting for interrupt */
		get_serial_character(packet->channel_num, &cc);			/* recieiving data item */

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

void SerialReceive_Task_PC(void* pvParameters)
{
	uint8_t cc = 0;
	uint8_t pointer = 0;
	char buffer[R_BUF_SIZE];

	for (;;)
	{
		xSemaphoreTake(RX_BinarySemaphore_PC, portMAX_DELAY);	/* waiting for interrupt */
		get_serial_character(COM_CH_PC, &cc);					/* recieiving data item */
		if (cc == 13) {
			buffer[pointer] = 0;
			pointer = 0;
			printf("Received string is : %s\n", buffer);
			xQueueSend(pc_comm_queue, (void*)buffer, (TickType_t)0);
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
		xSemaphoreTake(*(packet->sempahore), portMAX_DELAY);						/* waiting for timer to update the semaphore */

		for (t_point = 0; t_point < sizeof(trigger); t_point++) {					/* Sending the message */
			send_serial_character(packet->channel_num, trigger[t_point++]);
		}
	}
}

void SerialSend_Task_PC(void* pvParameters) {
	
	char msg[8];
	int i;

	while (1) {
		if (xQueueReceive(pc_tx_queue, msg, (TickType_t)QUEUE_MAX_TICKS_TO_WAIT) == pdPASS) {
			for (i = 0; i < 8; i++) {
				send_serial_character(COM_CH_PC, msg[i]);
			}
		}
	}
}

void ShowDataOn7Seg(void* pvParameters) {

	char msg_0[R_BUF_SIZE];
	char msg_1[R_BUF_SIZE];

	for (;;) {
		if(start_stop) {
			if (xQueueReceive(rx2seg7_queue_0, msg_0, (TickType_t)QUEUE_MAX_TICKS_TO_WAIT) == pdPASS) {
				if (xQueueReceive(rx2seg7_queue_1, msg_1, (TickType_t)QUEUE_MAX_TICKS_TO_WAIT) == pdPASS) {

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
		else {
			select_7seg_digit(0);
			set_7seg_digit(0x40);
			select_7seg_digit(1);
			set_7seg_digit(0x40);
			select_7seg_digit(2);
			set_7seg_digit(0x40);
			select_7seg_digit(3);
			set_7seg_digit(0x40);
		}
	}
}

static void TimerCallback( TimerHandle_t xTimer ) {

	static uint8_t timer_0 = 0;
	static uint8_t previous_interval_0, previous_interval_1;
	uint8_t interval_range_0, interval_range_1;
	
	if (timer_0++ > 39) {
		xSemaphoreGive(TX_BinarySemaphore_0);	/* Serial Communication Channel 0 */
		xSemaphoreGive(TX_BinarySemaphore_1);	/* Serial Communication Channel 1 */
		timer_0 = 0;
	}

	interval_range_0 = checkInterval(left_sensor, left_sensor_max_value, left_sensor_min_value);
	interval_range_1 = checkInterval(right_sensor, right_sensor_max_value, right_sensor_min_value);

	if (previous_interval_0 != interval_range_0) set_LED_BAR(1, 0x00);
	if (previous_interval_1 != interval_range_1) set_LED_BAR(2, 0x00);

	/* Diodes for the left sensor */

	switch (interval_range_0) {
	case 0:	/* value < MIN */
		vTaskSuspend(led_tsk_0_freq_1_Hz_1_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_4_ON);
		vTaskResume(led_tsk_0_freq_2_Hz_8_ON);
		break;
	case 1:	/* MIN < value < HALF */
		vTaskSuspend(led_tsk_0_freq_1_Hz_1_ON);
		vTaskResume(led_tsk_0_freq_2_Hz_4_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_8_ON);
		break;
	case 2:	/* HALF < value < MAX */
		vTaskResume(led_tsk_0_freq_1_Hz_1_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_4_ON);
		vTaskSuspend(led_tsk_0_freq_2_Hz_8_ON);
		break;
	case 3: /* MAX < value */
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

	/* Diodes for the right sensor */

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

	previous_interval_0 = interval_range_0;
	previous_interval_1 = interval_range_1;

}

static void TimerCallback_PC(TimerHandle_t xTimer){
	
	char msg_left_sensor[R_BUF_SIZE];
	char msg_right_sensor[R_BUF_SIZE];
	uint8_t interval_left, interval_right;

	msg_left_sensor[0] = 'L';
	msg_left_sensor[1] = ':';
	msg_left_sensor[2] = ' ';
	msg_left_sensor[4] = ' ';
	msg_right_sensor[0] = 'R';
	msg_right_sensor[1] = ':';
	msg_right_sensor[2] = ' ';
	msg_right_sensor[4] = ' ';

	interval_left = checkInterval(left_sensor, left_sensor_max_value, left_sensor_min_value);
	interval_right = checkInterval(right_sensor, right_sensor_max_value, right_sensor_min_value);

	msg_left_sensor[3] = interval_left + 48;
	msg_right_sensor[3] = interval_right + 48;
	msg_left_sensor[5] = (left_sensor / 10) + 48;
	msg_right_sensor[5] = (right_sensor / 10) + 48;
	msg_left_sensor[6] = (left_sensor % 10) + 48;
	msg_right_sensor[6] = (right_sensor % 10) + 48;
	msg_left_sensor[7] = '\n';
	msg_right_sensor[7] = '\n';

	xQueueSend(pc_tx_queue, (void*)msg_left_sensor, (TickType_t)0);
	xQueueSend(pc_tx_queue, (void*)msg_right_sensor, (TickType_t)0);

}

/* MAIN - SYSTEM STARTUP POINT */
void main_demo(void)
{
	
	init_LED_comm();						/* Initializing LED bar */
	set_LED_BAR(1, 0x00);					/* Switching off all of the output LEDs */
	set_LED_BAR(2, 0x00);					/* Switching off all of the output LEDs */
	init_serial_uplink(COM_CH_0);			/* Initializing Tx for channel 0 */
	init_serial_downlink(COM_CH_0);			/* Initializing Rx for channel 0 */
	init_serial_uplink(COM_CH_1);			/* Initializing Tx for channel 1 */
	init_serial_downlink(COM_CH_1);			/* Initializing Rx for channel 1 */
	init_serial_uplink(COM_CH_PC);			/* Initializing Tx for channel 2 */
	init_serial_downlink(COM_CH_PC);		/* Initializing Rx for channel 2 */
	init_7seg_comm();						/* Initializing 7seg display */

	/* ON INPUT CHANGE INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);
	/* SERIAL RECEPTION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXInterrupt);

	/* Create LED interrapt semaphore */
	LED_BinarySemaphore_0 = xSemaphoreCreateBinary();
	if (LED_BinarySemaphore_0 != NULL) printf("Semaphore LED_BinarySemaphore_0 successfully created!\n");
	LED_BinarySemaphore_1 = xSemaphoreCreateBinary();
	if (LED_BinarySemaphore_1 != NULL) printf("Semaphore LED_BinarySemaphore_1 successfully created!\n");
	RX_BinarySemaphore_0 = xSemaphoreCreateBinary();
	if (RX_BinarySemaphore_0 != NULL) printf("Semaphore RX_BinarySemaphore_0 successfully created!\n");
	RX_BinarySemaphore_1 = xSemaphoreCreateBinary();
	if (RX_BinarySemaphore_1 != NULL) printf("Semaphore RX_BinarySemaphore_1 successfully created!\n");
	RX_BinarySemaphore_PC = xSemaphoreCreateBinary();
	if (RX_BinarySemaphore_PC != NULL) printf("Semaphore RX_BinarySemaphore_PC successfully created!\n");
	TX_BinarySemaphore_0 = xSemaphoreCreateBinary();
	if (TX_BinarySemaphore_0 != NULL) printf("Semaphore TX_BinarySemaphore_0 successfully created!\n");
	TX_BinarySemaphore_1 = xSemaphoreCreateBinary();
	if (TX_BinarySemaphore_1 != NULL) printf("Semaphore TX_BinarySemaphore_1 successfully created!\n");

	/* Queues */
	rx2seg7_queue_0 = xQueueCreate(MAX_QUEUE_SIZE, R_BUF_SIZE);
	rx2seg7_queue_1 = xQueueCreate(MAX_QUEUE_SIZE, R_BUF_SIZE);
	pc_comm_queue = xQueueCreate(MAX_QUEUE_SIZE, R_BUF_SIZE);
	pc_tx_queue = xQueueCreate(MAX_QUEUE_SIZE, R_BUF_SIZE);

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
	if (xTaskCreate(led_blink, "LED_0_freq_1_Hz_1_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_0_freq_1_Hz_1_ON), LED_TASK_PRI, &led_tsk_0_freq_1_Hz_1_ON) != pdPASS)
		printf("LED_0_freq_1_Hz_1_ON was not created!\n");
	if(xTaskCreate(led_blink, "LED_0_freq_2_Hz_4_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_0_freq_2_Hz_4_ON), LED_TASK_PRI, &led_tsk_0_freq_2_Hz_4_ON) != pdPASS)
		printf("LED_0_freq_2_Hz_4_ON was not created!\n");
	if(xTaskCreate(led_blink, "LED_0_freq_2_Hz_8_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_0_freq_2_Hz_8_ON), LED_TASK_PRI, &led_tsk_0_freq_2_Hz_8_ON) != pdPASS)
		printf("LED_0_freq_2_Hz_8_ON was not created!\n");
	if(xTaskCreate(led_blink, "LED_1_freq_1_Hz_1_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_1_freq_1_Hz_1_ON), LED_TASK_PRI, &led_tsk_1_freq_1_Hz_1_ON) != pdPASS)
		printf("LED_1_freq_1_Hz_1_ON was not created!\n");
	if(xTaskCreate(led_blink, "LED_1_freq_2_Hz_4_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_1_freq_2_Hz_4_ON), LED_TASK_PRI, &led_tsk_1_freq_2_Hz_4_ON) != pdPASS)
		printf("LED_1_freq_2_Hz_4_ON was not created!\n");
	if(xTaskCreate(led_blink, "LED_1_freq_2_Hz_8_ON", configMINIMAL_STACK_SIZE, (void*)(&led_st_1_freq_2_Hz_8_ON), LED_TASK_PRI, &led_tsk_1_freq_2_Hz_8_ON) != pdPASS)
		printf("LED_1_freq_2_Hz_8_ON was not created!\n");

	/* SERIAL RECEIVER TASK */
	xTaskCreate(SerialReceive_Task, "SensorRx_CH0", configMINIMAL_STACK_SIZE, (void*)(&rx_packet_channel_0), TASK_SERIAL_REC_PRI, NULL);
	xTaskCreate(SerialReceive_Task, "SensorRx_CH1", configMINIMAL_STACK_SIZE, (void*)(&rx_packet_channel_1), TASK_SERIAL_REC_PRI, NULL);
	xTaskCreate(SerialReceive_Task_PC, "PC_Rx", configMINIMAL_STACK_SIZE, (void*)0, TASK_SERIAL_REC_PRI_PC, NULL);
	/* SERIAL TRANSMITTER TASK */
	xTaskCreate(SerialSend_Task, "SensorTx_CH0", configMINIMAL_STACK_SIZE, (void*)(&tx_packet_channel_0), TASK_SERIAL_SEND_PRI, NULL);
	xTaskCreate(SerialSend_Task, "SensorTx_CH1", configMINIMAL_STACK_SIZE, (void*)(&tx_packet_channel_1), TASK_SERIAL_SEND_PRI, NULL);
	xTaskCreate(SerialSend_Task_PC, "PC_Tx", configMINIMAL_STACK_SIZE, (void*)0, TASK_SERIAL_SEND_PRI, NULL);
	/* PROCESSING TASKS */
	xTaskCreate(ShowDataOn7Seg, "7SegDisplay", configMINIMAL_STACK_SIZE, (void*)0, DISPLAY_TASK_PRI, NULL);
	xTaskCreate(PC_control_Task, "PC_Control", configMINIMAL_STACK_SIZE, (void*)0, PC_TASK_PRI, NULL);

	/* Timers */
	tH0 = xTimerCreate(
		"Timer_0",
		pdMS_TO_TICKS(5),
		pdTRUE,
		0,
		TimerCallback
	);

	tH1 = xTimerCreate(
		"Timer_1",
		pdMS_TO_TICKS(5000),
		pdTRUE,
		0,
		TimerCallback_PC
	);
	 
	xTimerStart(tH0, 0);
	xTimerStart(tH1, 0);
	vTaskStartScheduler();
}