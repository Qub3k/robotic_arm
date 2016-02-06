#include "pit.h"

#define printf uart_transmit
#define endl() uart_transmit("\r\n")

/**
  * Variables for the MPU-6050.
  */
short gyro_data[3] = {0, 0, 0};
short accel_data[3] = {0, 0, 0};
float gyro_sens = 0;
unsigned short accel_sens = 0;
float tmp_fl = 0;

/** 
  * Variable for the wait_ms() function.
  */
bool time_elapsed = false;

/**
  * Variables for the Self-test function.
  */
long gyro_st[3] = {0, 0, 0};
long accel_st[3] = {0, 0, 0};
int self_test_result = 0;

/**
  * Variable for the calibration.
  */
int8_t gyro_bias[3] = {0, 0, 0};

void pit_init() {
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
  
  /* Enable interrupts for channel 1 */
  PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TIE_MASK;
  
  // Enable PIT module
  PIT->MCR = 0x00;
  
  /* Initialize the I2C */
  i2c_init();
  
  /* Initialize the UART */
  uart_init(); 
  
  /* Initialize the MPU-6050 */
  if(mpu_init((struct int_param_s *)NULL))
    printf("Initialization of MPU-6050 failed\n");
  
  /* Turn on the GYRO and ACCEL */
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  
  /* Perform the self-test */
  printf("Self test results:\n");
  self_test_result = mpu_run_self_test(gyro_st, accel_st);
  if(self_test_result & 0x01){
    printf("\tGyro: passed\n");
  }else{
    printf("\tGyro: failed\n");
  }
  if(self_test_result>>1 & 0x01){
    printf("\tAccel: passed\n");
  }else{
    printf("\tAccel: failed\n");
  }

	/* Read the sensitivity of the gyro */
  mpu_get_gyro_sens(&gyro_sens);
	
  /* Read the sensitivity of the accel */
  mpu_get_accel_sens(&accel_sens);
  
  /* Calibrate the mpu-6050 */
  printf("Calibrating...\n");
  if(mpu_calibate_gyro(gyro_bias))
    printf("Calibration failed\n");
  printf("Calibration results:\n");
  printf("\tGyro_x: %d\n", gyro_bias[0]);
  printf("\tGyro_y: %d\n", gyro_bias[1]);
  printf("\tGyro_z: %d\n", gyro_bias[2]);
  
  // Enable channel 0 timer in PIT - reading the results from the MPU-6050
  PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
}

// Define the Interrupt Service Routine for the Periodic Interrupt Timer
void PIT_IRQHandler(void) {
  /* Check which channel generated an interrupt */
  if(PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
    /* Send the gyro data */
    if(mpu_get_gyro_reg(gyro_data, (unsigned long *)NULL)){
			// do nothing if reading failed
    }else{
      tmp_fl = (float)(gyro_data[0] - gyro_bias[0])/(float)gyro_sens;
      gyro_data[0] = tmp_fl;
      tmp_fl = (float)(gyro_data[1] - gyro_bias[1])/(float)gyro_sens;
      gyro_data[1] = tmp_fl;
      tmp_fl = (float)(gyro_data[2] - gyro_bias[2])/(float)gyro_sens;
      gyro_data[2] = tmp_fl;
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
      printf("\t%+hd\t%+hd\t%+hd\n", accel_data[0], accel_data[1] , accel_data[2]);
    }
    
    // Clear channel 0 interrupt flag
    PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;
  }
  if(PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK) {
    /* Disable channel 1 */
    PIT->CHANNEL[1].TCTRL &= ~PIT_TCTRL_TEN_MASK;
    
    /* Set the "time_elapsed" flag */
    time_elapsed = true;
    
    /* Clear the channel 1 interrupt flag */
    PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;
  }
}

void wait_ms(uint32_t ms_to_wait) {
  /* Prevent from asking for too high value */
  if(ms_to_wait > 178956)
    ms_to_wait = 178956;
  
  /* Set the LDVAL for channel 1 of PIT */
  PIT->CHANNEL[1].LDVAL = ms_to_wait*24000;
  
  /* Clear the "time_elapsed" time */
  time_elapsed = false;
  
  /* Turn on the channel 1 */
  PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TEN_MASK;
  
  /* Wait for the interrupt indicating the end of the time */
  while(!time_elapsed);
}
