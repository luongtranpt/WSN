/*
 * Temp.c
 *
 *  Created on: Dec 13, 2023
 *      Author: Xuan Mai, Tran Luong
 */

/* Private includes ----------------------------------------------------------*/
#include "Queue.h"
//#include "main.h"
#include "stdio.h"
#include "string.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
uint8_t Sizeof_Queue_Current(queue_t *queue)
{
	uint8_t i = 0;
	uint8_t total = 0;
	if (queue[0].data == 0U)
	{
		return total;
	}
	total = 0;
	while (queue[i].data != 0U)
	{
		total++;
		i++;
	}
	return total;
}

QueueState Push_Queue(queue_t *queue, EventState State,uint16_t data,uint8_t *array)
{
	int count = Sizeof_Queue_Current(queue);
	if ( count >= Queue_Size)
		return Queue_Full;
	int i = 0;
	if ( count != 0) 
	{
		for (i = count - 1; i >= 0; i--)
        {
			memcpy(&queue[i + 1], &queue[i], sizeof(queue_t));
		}
	}
	//queue[0].Func  = Function;
	queue[0].State = State;
	queue[0].data  = data; 
	ESP_LOGI("hello","fail");

	return PushQueue_Done;
}

QueueState Pop_Queue(queue_t *queue)
{
	if (Sizeof_Queue_Current(queue) == 0)
		return Queue_Empty;
    uint8_t count = Sizeof_Queue_Current(queue);
	queue[count - 1].Func  = NULL;
	queue[count - 1].State = None;
	queue[count - 1].data  = 0;
	strcpy((char*) queue[0].array,"0");
	return PopQueue_Done;
}

bool Check_Queue(queue_t* queue, uint16_t ID)
{
	ESP_LOGI("hello","size: %d",Sizeof_Queue_Current(queue));
   for (uint8_t i = 0; i < Sizeof_Queue_Current(queue); i++)
   {
	if (queue[i].data == ID)
	{
	  return true;
	}
   }
   return false;
   
}