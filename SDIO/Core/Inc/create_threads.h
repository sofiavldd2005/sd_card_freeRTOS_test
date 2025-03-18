#ifndef CREATE_THREADS_H
#define CREATE_THREADS_H


/* user includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>


//  freertos libraries
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "cmsis_os2.h"
#include "ff.h"


osThreadId_t thread1_id;
osThreadId_t write_sd_id;



void create_threads ();





#endif /* CREATE_THREADS_H */
