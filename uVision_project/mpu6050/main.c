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
 
  while(1){
    if(mpu_get_gyro_reg(gyro_data, (unsigned long *)NULL)){
			break;
    }else{
      tmp_fl = (float)gyro_data[0]/(float)gyro_sens;
      gyro_data[0] = tmp_fl;
      tmp_fl = (float)gyro_data[1]/(float)gyro_sens;
      gyro_data[1] = tmp_fl;
      tmp_fl = (float)gyro_data[2]/(float)gyro_sens;
      gyro_data[2] = tmp_fl;
			printf("%+08hd\t%+08hd\t%+08hd", gyro_data[0] , gyro_data[1] , gyro_data[2]);
    }
    if(mpu_get_accel_reg(accel_data, (unsigned long *)NULL)){
      break;
    }else{
      tmp_fl = (float)accel_data[0]/(float)accel_sens;
      accel_data[0] = tmp_fl*1000;
      tmp_fl = (float)accel_data[1]/(float)accel_sens;
      accel_data[1] = tmp_fl*1000;
      tmp_fl = (float)accel_data[2]/(float)accel_sens;
      accel_data[2] = tmp_fl*1000;
      printf("\t%+08hd\t%+08hd\t%+08hd\n", accel_data[0], accel_data[1] , accel_data[2]);
    }
    delay_ms(50);
  }
   
}
