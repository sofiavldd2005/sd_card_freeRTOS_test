#include "create_threads.h"

// thread inutil do .ioc
extern osThreadId_t defaultTaskHandle;



const osThreadAttr_t thread1_atrr = {
		.name = "thread1",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityHigh, };

const  	osThreadAttr_t thread_write_sd_attr =  {
		.name = "thread_write_sd",
		.stack_size = 5000 * 4,
		.priority = (osPriority_t) osPriorityLow, };



void create_threads() {
	// NÃO REMOVER - faz funcionar os printf()
	//RetargetInit(UART_DEBUG);

	// remove thread inutil do .ioc
	osThreadTerminate(defaultTaskHandle);

	// create threads
	thread_controller_id = osThreadNew(thread_write_sd_function, NULL,
				&thread_write_sd_attr);

	blinkThreadID = osThreadNew(thread1, NULL, &thread1_attr)

	// kernel will start!
	printf("\r\nkernel started\r\n\r\n");


}
