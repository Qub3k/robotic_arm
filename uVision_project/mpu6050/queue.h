/**
 * @addtogroup  DRIVERS Sensor Driver Layer
 * @brief       Hardware drivers to communicate with sensors via I2C.
 * @{
 * 		@file queue.h
 *		@ingroup UART
 * 		Library implementing the queue structures.
 */

#ifndef QUEUE_H
#define QUEUE_H

#define Q_SIZE 100

typedef struct{
  unsigned char Data[Q_SIZE];
  unsigned int Head; // points to oldest data element
  unsigned int Tail; // points to next free space
  unsigned int Size; // quantity of elements in queue
} Q_T;

void q_init(Q_T *q);
int q_empty(Q_T *q);
int q_full(Q_T *q);
int q_enqueue(Q_T *q, unsigned char d);
unsigned char q_dequeue(Q_T *q);

#endif
