#include "MKL46Z4.h"
#include "pit.h"
#include "extern.h"

/* Variable from shared with other files by the "extern.h" */
uint32_t milliseconds = 0;

/**
  * ISR for SysTick - counts how many ms elapsed since the beginning of the application.
  */
void SysTick_Handler(void) {
  milliseconds++;
}

int main() {     
  /* Update the SystemCoreClock variable */
  SystemCoreClockUpdate();
  
  /* Configure the SysTick */
  SysTick_Config(SystemCoreClock / 1000);
  
  /* Initialize the PIT */
  pit_init();
 
  while(1){
  }
   
}
