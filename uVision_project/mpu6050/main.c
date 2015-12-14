#include "MKL46Z4.h"
#include "i2c_mpu6050.h"
#include "uart.h"
#include "mpu6050.h"
#include "float_MKL46Z.h"

#define printf uart_transmit
#define endl() uart_transmit("\r\n")
#define delay_ms(x) delay_mc(5*(x))

float tmp_fl = 0;
short int tmp_sh_int = 0;

int main() {
  short gyro_data[3];
  short accel_data[3];
  float gyro_sens = 0;
  unsigned short accel_sens = 0;
  long accel_bias[3] = {0, 0, 0};
  long gyro_bias[3] = {0, 0, 0};
  int self_test_result = 0;
  
  /* Initialize the I2C */
  i2c_init();
  
  /* Initialize the UART */
  uart_init();
  
  /* Initialize the MPU-6050 */
  endl();
  printf("Initialize the device:\r\n");
  if(mpu_init((struct int_param_s *)NULL))
    printf("mpu_init() has crashed \r\n");
  endl();
  
  /* Turn on the GYRO and ACCEL */
  if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)){
    printf("Failed booting up the gyro or accel\n\r");
  }else{
    printf("Gyro and accel successfully booted!\r\n");
    endl();
  }
  
  /* Set proper FSR before performing self-test */
  if(mpu_set_gyro_fsr(250)){
    printf("Could not set the FSR for gyro");
    endl();
  }else{
    printf("Gyro FSR set to +/- 250");
    endl();
  }
  if(mpu_set_accel_fsr(8)){
    printf("Could not set the FSR for accel");
    endl();
  }else{
    printf("Accel FSR set to +/- 8");
    endl();
  }
  
  /* Run the self-test */
  self_test_result = mpu_run_self_test(gyro_bias, accel_bias);
  printf("Self test result = %i", self_test_result);
  endl();
  if(self_test_result == 0x03){
    printf("Gyro and accel passed the self test!");
    endl();
  }else if(self_test_result == 0x02){
    printf("Only accel passed the test");
    endl();
  }else if(self_test_result == 0x01){
    printf("Only gyro passed the test");
    endl();
  }
  
  /* Read the sensitivity of the gyro */
  if(mpu_get_gyro_sens(&gyro_sens)){
    printf("Failed reading the gyro sensitivity");
    endl();
  }else{
  }
  
  /* Read the sensitivity of the accel */
  if(mpu_get_accel_sens(&accel_sens)){
    printf("Failed reading the accel sensitivity");
    endl();
  }else{
    printf("Accel sensitivity: %d", accel_sens);
    endl();
  }
  
  /* Print the gyro values */
  printf("Sensors readings:");
  endl();
  
  while(1){
    if(mpu_get_gyro_reg(gyro_data, (unsigned long *)NULL)){
      printf("Failed to read the data from the gyro");
      endl();
    }else{
      tmp_fl = (float)gyro_data[0]/(float)gyro_sens;
      gyro_data[0] = tmp_fl;
      tmp_fl = (float)gyro_data[1]/(float)gyro_sens;
      gyro_data[1] = tmp_fl;
      tmp_fl = (float)gyro_data[2]/(float)gyro_sens;
      gyro_data[2] = tmp_fl;
      printf("GYRO_X = %+08hd", gyro_data[0]);
      printf("\tGYRO_Y = %+08hd", gyro_data[1]);
      printf("\tGYRO_Z = %+08hd", gyro_data[2]);
      printf("\t\t");
    }
    if(mpu_get_accel_reg(accel_data, (unsigned long *)NULL)){
      printf("Failed to read the data from the gyro");
      endl();
    }else{
      tmp_fl = (float)accel_data[0]/(float)accel_sens;
      accel_data[0] = tmp_fl*1000;
      tmp_fl = (float)accel_data[1]/(float)accel_sens;
      accel_data[1] = tmp_fl*1000;
      tmp_fl = (float)accel_data[2]/(float)accel_sens;
      accel_data[2] = tmp_fl*1000;
      printf("ACCEL_X = %+08hd", accel_data[0]);
      printf("\tACCEL_Y = %+08hd", accel_data[1]);
      printf("\tACCEL_Z = %+08hd", accel_data[2]);
      printf("\r");
    }
    delay_ms(50);
  }
   
}
