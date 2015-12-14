#include "queue.h"

void q_init(Q_T *q){
  unsigned int i;
  for(i = 0; i < Q_SIZE; i++) {
    q->Data[i] = 0;
  }
  q->Head = 0;
  q->Tail = 0;
  q->Size = 0;
}

int q_empty(Q_T *q){
  return q->Size == 0;
}

int q_full(Q_T *q){
  return q->Size == Q_SIZE;
}

int q_enqueue(Q_T *q, unsigned char d){
  if(!q_full(q)) {
    q->Data[q->Tail++] = d; // post-increment the q->Tail 
    q->Tail %= Q_SIZE; // circular buffer behaviour
    q->Size++;
    return 0; // success
  } else
    return 1; // failure
}

unsigned char q_dequeue(Q_T *q){
  unsigned char t = 0;
  if(!q_empty(q)) {
    t = q->Data[q->Head];
    q->Data[q->Head++] = 0; // post-increment the q->Head
    q->Head %= Q_SIZE; // circular buffer behaviour
    q->Size--;
  }
  return t;
}
