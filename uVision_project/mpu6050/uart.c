#include "uart.h"

/* Define the queues for the interrupt-driven communication */
Q_T RxQ;
Q_T TxQ;

void uart_init(void) {
  /* Supply the clock for UART0 and PORTA */
  SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
  
  /* Set PORTA1 as Rx and PORTA2 as Tx */
  PORTA->PCR[1] &= ~PORT_PCR_MUX_MASK; // Make sure that MUX is clear
  PORTA->PCR[1] |= PORT_PCR_MUX(2);
  PORTA->PCR[2] &= ~PORT_PCR_MUX_MASK; // Make sure that MUX is clear
  PORTA->PCR[2] |= PORT_PCR_MUX(2);
  
  /* Choose external 8MHz crystal as a reference clock for UART0 */
  /* Asynch Module Clock = 8 MHz */
  SIM->SOPT2 |= SIM_SOPT2_UART0SRC(2);
  
  /* Disable the reciever and transmitter of UART0 */
  UART0->C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK); // turn off the Tx and Rx
  
  /* Set the oversampling ratio to 31 */
  UART0->C4 &= ~UART0_C4_OSR_MASK; // clear the OSR
  UART0->C4 |= UART0_C4_OSR(31);
  
  /* Set SBr to 26 in order to achieve Baud Rate euqal to 9600 */
  UART0->BDH |= UART0_BDH_SBR(0);
  UART0->BDL &= ~UART0_BDL_SBR_MASK; // clear BDL first
  UART0->BDL |= UART0_BDL_SBR(26);
  
  /* Set 1 Stop Bit */
  UART0->BDH &= ~UART0_BDH_SBNS_MASK;

  /* Choose 8-bits long data */
  UART0->C1 &= ~UART0_C1_M_MASK;
  
  /* Disable hardware parity check */
  UART0->C1 &= ~UART0_C1_PE_MASK;
  
  /* Initialize the queues for the interrupts-driven serial communication */  
  q_init(&TxQ);
  q_init(&RxQ);
  
  /* Configure the interrupts from the UART0 */
  NVIC_ClearPendingIRQ(UART0_IRQn);
  NVIC_EnableIRQ(UART0_IRQn);
  
  /* Enable the interrupt when receiver buffer gets full */
  UART0->C2 |= UART0_C2_RIE_MASK;
 
  /* Turn on the receiver and transmitter */
  UART0->C2 |= UART0_C2_TE_MASK | UART0_C2_RE_MASK; // turn on the Tx and Rx
}

void uart_transmit_poll(uint8_t data){
  /* wait for the transmitter buffer to be empty */
  while(!(UART0->S1 & UART_S1_TDRE_MASK));
  
  /* Send the data */
  UART0->D = data;
}

uint8_t uart_receive_poll(void){
  /* Wait for the receiver buffer to be full */
  while(!(UART0->S1 & UART_S1_RDRF_MASK));
  
  /* Read the data */
  return UART0->D;
}

void uart_transmit(const char *data, ...) {
  char buf[MAX_BUFF_SIZE]; 
  unsigned int i = 0;
  int num_read = 0;
  
  /* Way to read the formated string as the argument */
  va_list args;
  va_start(args, data);
  num_read = vsprintf(buf, data, args);
  
  /* Write the data given as an arugment to the queue */
  while(i < num_read && i < MAX_BUFF_SIZE) {
    if(!q_full(&TxQ)){
      q_enqueue(&TxQ, buf[i]);
      i++;
    }else{
      /* Don't increment the iterator, try to insert the data in the next run */
    }
    /* Each time you insert something, make sure that the interrupt fot transmitter is active */
    UART0->C2 |= UART0_C2_TIE_MASK;
  }
  
  va_end(args);
  
}

void uart_receive(char *data){
  unsigned int i = 0;
  
  /* Read the data from the queue and save it to the "data" variable */
  while(RxQ.Size != 0 && i < strlen(data)) {
    data[i] = q_dequeue(&RxQ);
    i++;
  }
}

void UART0_IRQHandler(void) {
  NVIC_ClearPendingIRQ(UART0_IRQn);
  
  /* Transmitter part */
  if(UART0->S1 & UART_S1_TDRE_MASK) {
    if(!q_empty(&TxQ)){ // there is something to transmit
      UART0->D = q_dequeue(&TxQ);
    }else{ // there is nothing to transmit
      UART0->C2 &= ~UART_C2_TIE_MASK; // clear the interrupt flag
    }
  }
  
  /* Receiver part */
  if(UART0->S1 & UART_S1_RDRF_MASK) {
    if(!q_full(&RxQ)){ // there is still space to store something
      q_enqueue(&RxQ, UART0->D);
    }else{ // error - receiver queue full
      while(1);
    }
  }
}
