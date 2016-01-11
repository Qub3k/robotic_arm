#include "pit.h"

#define printf uart_transmit
#define endl() uart_transmit("\r\n")

short gyro_data[3] = {0, 0, 0};
short accel_data[3] = {0, 0, 0};
float gyro_sens = 0;
unsigned short accel_sens = 0;
float tmp_fl = 0;
short int tmp_sh_int = 0;
uint32_t timestamp = 0;

void pit_init() {
  /* Initialize the I2C */
  i2c_init();
  
  /* Initialize the UART */
  uart_init();
  
  /* Initialize the MPU-6050 */
  mpu_init((struct int_param_s *)NULL);
  
  /* Turn on the GYRO and ACCEL */
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

	/* Read the sensitivity of the gyro */
  mpu_get_gyro_sens(&gyro_sens);
	
  /* Read the sensitivity of the accel */
  mpu_get_accel_sens(&accel_sens);
  
  // Enable clock for he PIT module
  SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
  
  // Set period for PIT (keep in mind that Periodic Interrupt Timer is 32-bit long, counts down and operates
  // at frequency equal to 24 MHz (41.67 ns / tic)
  PIT->CHANNEL[0].LDVAL = 0x75300; // Set period to 0.02s = 50 Hz
  
  // Configure Nested Vectored Interrupt Controller
  NVIC_SetPriority(PIT_IRQ_NBR, 2);
  NVIC_ClearPendingIRQ(PIT_IRQ_NBR);
  NVIC_EnableIRQ(PIT_IRQ_NBR);
  
  // Enable interrupts for channel 0 in Periodic Interrupt Timer
  PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;
  
  // Enable channel 0 timer in PIT 
  PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
  
  // Enable PIT module
  PIT->MCR = 0x00;
}

// Define the Interrupt Service Routine for the Periodic Interrupt Timer
void PIT_IRQHandler(void) {
  // Check which channel generated an interrupt
  if(PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
    /* Increase the timestamp value */
    timestamp++;
    
    /* Send the gyro data */
    if(mpu_get_gyro_reg(gyro_data, (unsigned long *)NULL)){
			// do nothing if reading failed
    }else{
      tmp_fl = (float)gyro_data[0]/(float)gyro_sens;
      gyro_data[0] = tmp_fl;
      tmp_fl = (float)gyro_data[1]/(float)gyro_sens;
      gyro_data[1] = tmp_fl;
      tmp_fl = (float)gyro_data[2]/(float)gyro_sens;
      gyro_data[2] = tmp_fl;
			//printf("%+08hd\t%+08hd\t%+08hd", gyro_data[0] , gyro_data[1] , gyro_data[2]);
      printf("%+hd\t%+hd\t%+hd", gyro_data[0] , gyro_data[1] , gyro_data[2]);
    }
    
    /* Send the accelerometer data */
    if(mpu_get_accel_reg(accel_data, (unsigned long *)NULL)){
      // do nothing if reading failed
    }else{
      tmp_fl = (float)accel_data[0]/(float)accel_sens;
      accel_data[0] = tmp_fl*1000;
      tmp_fl = (float)accel_data[1]/(float)accel_sens;
      accel_data[1] = tmp_fl*1000;
      tmp_fl = (float)accel_data[2]/(float)accel_sens;
      accel_data[2] = tmp_fl*1000;
      //printf("\t%+08hd\t%+08hd\t%+08hd\n", accel_data[0], accel_data[1] , accel_data[2]);
      printf("\t%+hd\t%+hd\t%+hd", accel_data[0], accel_data[1] , accel_data[2]);
    }
    
    /* Send the timestamp */
    printf("\t%u\n", timestamp);
    
    // Clear channel 0 interrupt flag
    PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
  }
  
}
