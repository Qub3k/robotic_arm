/**
 * @file queue.h
 * Library implementing the queue structures.
 */

#ifndef QUEUE_H
#define QUEUE_H

/** 
 * Macro defining the queues' length
 */
#define Q_SIZE 64

/**
 * Structure storing the queue's data and parameters.
 */
typedef struct{
  unsigned char Data[Q_SIZE]; /**< Data inside the queue */
  unsigned int Head; /**< Pointer to the head of the queue */
  unsigned int Tail; /**< Pointer to the tail of the queue */
  unsigned int Size; /**< Size of the queue */
} Q_T;

/**
 * Initialization function for the queue library.
 */
void q_init(Q_T *q);

/**
 * Function that checks whether the queue is empty or not.
 * @return 1 if empty, 0 if not
 */
int q_empty(Q_T *q);

/**
 * Function that checks whether the queue is full or not.
 * @return 1 if full, 0 if not
 */
int q_full(Q_T *q);

/**
 * Function for pushing the element to the queue
 * @param[in] q pointer to the queue object on which to perform the operation
 * @param[in] d 8-bits long data to push
 * @return 0 if successful, 1 if not
 */
int q_enqueue(Q_T *q, unsigned char d);

/**
 * Function for popint the element from the queue
 * @param[in] q pointer to the queue object on which to perform the operation
 * @return 8-bits long data read from the queue
 */
unsigned char q_dequeue(Q_T *q);

#endif
