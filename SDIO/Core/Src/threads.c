#include "main.h"
#include "threads.h"
#include "stdio.h"
#include "string.h"
#include "FreeRTOSConfig.h"

void thread1(void *argument){
	while(1){
		HAL_GPIO_Togglepin(LD2_GPIO_Port, LD2_Pin);
		osDelay(500);
	}
}


void thread_write_sd_function(){

}
