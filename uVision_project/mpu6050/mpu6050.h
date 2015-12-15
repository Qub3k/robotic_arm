/**
 * @file mpu6050.h
 * @brief Hardware drivers to communicate with sensors via I2C.
 * @details This header file was created in order to add the support for I2C communication
 * with the MPU-6050 interial measurement unit used in the "Robotic Arm" project.
 */

#ifndef _INV_MPU_H_
#define _INV_MPU_H_

/** 
 * Testing whether doxygen will include this comment.
 */
#include "i2c_mpu6050.h"
#include "extra.h"
#include "uart.h"
#include "MKL46Z4.h"

/**
 * Macro that provides a quick way to pick the x-axis of gyro.
 */
#define INV_X_GYRO      (0x40)

/**
* Macro that provides a quick way to pick the y-axis of gyro.
*/
#define INV_Y_GYRO      (0x20)

/**
* Macro that provides a quick way to pick the z-axis of gyro.
*/
#define INV_Z_GYRO      (0x10)

/**
* Macro that provides a quick way to pick all the axes of gyro.
*/
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)

/**
* Macro that provides a quick way to pick the y-axis of accel.
*/
#define INV_XYZ_ACCEL   (0x08)

/**
* Macro that provides a quick way to pick the y-axis of compass.
*/
#define INV_XYZ_COMPASS (0x01) // ten define nam się nie przyda, bo nie używamy kompasu

/**
* Macro used to inform the compiler that we use the FRDM-KL46Z board.
*/
#define MOTION_DRIVER_TARGET_FRDMKL46Z

/**
 * Structure used for configuration of an interrupt call-back function.
 * This structure is specfic for a certain platform used.
 */
struct int_param_s {
  unsigned long pin; /**< Pin on which to observe the interrupts */
  void (*ISR)(void); /**< Pointer to the ISR servicing the interrrupts */
};

#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)

/**
 * Initialization function for the MPU-6050 sensor.
 *  Initial configuration:\n
 *  Gyro FSR: +/- 2000DPS\n
 *  Accel FSR +/- 2G\n
 *  DLPF: 42Hz\n
 *  FIFO rate: 50Hz\n
 *  Clock source: Gyro PLL\n
 *  FIFO: Disabled.\n
 *  Data ready interrupt: Disabled, active low, unlatched.
 *  @param[in]  int_param   Platform-specific parameters to interrupt API.
 *  @return     0 if successful.
 */
int mpu_init(struct int_param_s *int_param);

/**
 *  Set device to bypass mode.
 * 	This mode is used only when one wants to apply some external sensor to MPU-6050.
 *	For example a compass or any other type of magnetometer. When the bypass mode is
 *	on, the I2C transmission lines in MPU-6050 internally connect the host microcontroller
 *	with the externally connected, additional sensor plugged to MPU-6050.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
int mpu_set_bypass(unsigned char bypass_on);

/**
 *  Enter low-power accel-only mode.
 *  In low-power accel mode, the chip goes to sleep and only wakes up to sample
 *  the accelerometer at one of the following frequencies:
 *  \n MPU6050: 1.25Hz, 5Hz, 20Hz, 40Hz
 *  \n MPU6500: 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
 *  \n If the requested rate is not one listed above, the device will be set to
 *  the next highest rate. Requesting a rate above the maximum supported
 *  frequency will result in an error.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e rate.
 *  @param[in]  rate        Minimum sampling rate, or zero to disable LP
 *                          accel mode.
 *  @return     0 if successful.
 */
int mpu_lp_accel_mode(unsigned char rate);

/**
 *  Enters LP accel motion interrupt mode.
 *  The behaviour of this feature is very different between the MPU6050 and the
 *  MPU6500. Each chip's version of this feature is explained below.
 *
 *  \n The hardware motion threshold can be between 32mg and 8160mg in 32mg
 *  increments.
 *
 *  \n Low-power accel mode supports the following frequencies:
 *  \n 1.25Hz, 5Hz, 20Hz, 40Hz
 *
 *  \n MPU6500:
 *  \n Unlike the MPU6050 version, the hardware does not "lock in" a reference
 *  sample. The hardware monitors the accel data and detects any large change
 *  over a short period of time.
 *
 *  \n The hardware motion threshold can be between 4mg and 1020mg in 4mg
 *  increments.
 *
 *  \n MPU6500 Low-power accel mode supports the following frequencies:
 *  \n 1.25Hz, 2.5Hz, 5Hz, 10Hz, 20Hz, 40Hz, 80Hz, 160Hz, 320Hz, 640Hz
 *
 *  \n\n NOTES:
 *  \n The driver will round down @e thresh to the nearest supported value if
 *  an unsupported threshold is selected.
 *  \n To select a fractional wake-up frequency, round down the value passed to
 *  @e lpa_freq.
 *  \n The MPU6500 does not support a delay parameter. If this function is used
 *  for the MPU6500, the value passed to @e time will be ignored.
 *  \n To disable this mode, set @e lpa_freq to zero. The driver will restore
 *  the previous configuration.
 *
 *  @param[in]  thresh      Motion threshold in mg.
 *  @param[in]  time        Duration in milliseconds that the accel data must
 *                          exceed @e thresh before motion is reported.
 *  @param[in]  lpa_freq    Minimum sampling rate, or zero to disable.
 *  @return     0 if successful.
 */
int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time, unsigned char lpa_freq);

/**
 *  Set interrupt level.
 *  @param[in]  active_low  1 for active low, 0 for active high.
 *  @return     0 if successful.
 */
int mpu_set_int_level(unsigned char active_low);

/**
 *  Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
int mpu_set_int_latched(unsigned char enable);

/**
 *  Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     0 if successful.
 */
int mpu_set_dmp_state(unsigned char enable);

/**
 *  Get DMP state.
 *  @param[out] enabled 1 if enabled.
 *  @return     0 if successful.
 */
int mpu_get_dmp_state(unsigned char *enabled);

/**
 *  Get the current DLPF setting.
 *  @param[out] lpf Current LPF setting.
 *  0 if successful.
 */
int mpu_get_lpf(unsigned short *lpf);

/**
 *  Set digital low pass filter.
 *  The following LPF settings are supported: 188, 98, 42, 20, 10, 5.
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
int mpu_set_lpf(unsigned short lpf);

/**
 *  Get the gyro full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_gyro_fsr(unsigned short *fsr);

/**
 *  Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_gyro_fsr(unsigned short fsr);

/**
 *  Get the accel full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_accel_fsr(unsigned char *fsr);

/**
 *  Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_accel_fsr(unsigned char fsr);

/**
 *  Get the compass full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_compass_fsr(unsigned short *fsr);

/**
 *  Get gyro sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to dps.
 *  @return     0 if successful.
 */
int mpu_get_gyro_sens(float *sens);

/**
 *  Get accel sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to g's.
 *  @return     0 if successful.
 */
int mpu_get_accel_sens(unsigned short *sens);

/**
 *  Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_get_sample_rate(unsigned short *rate);

/**
 *  Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_set_sample_rate(unsigned short rate);

/**
 *  Get compass sampling rate.
 *  @param[out] rate    Current compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_get_compass_sample_rate(unsigned short *rate);

/**
 *  Set compass sampling rate.
 *  The compass on the auxiliary I2C bus is read by the MPU hardware at a
 *  maximum of 100Hz. The actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n WARNING: The new rate may be different than what was requested. Call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    Desired compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_set_compass_sample_rate(unsigned short rate);

/**
 *  Get current FIFO configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[out] sensors Mask of sensors in FIFO.
 *  @return     0 if successful.
 */
int mpu_get_fifo_config(unsigned char *sensors);

/**
 *  Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
int mpu_configure_fifo(unsigned char sensors);

/**
 *  Get current power state.
 *  @param[in]  power_on    1 if turned on, 0 if suspended.
 *  @return     0 if successful.
 */
int mpu_get_power_state(unsigned char *power_on);

/**
 *  Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
int mpu_set_sensors(unsigned char sensors);

/**
 *  Push biases to the accel bias registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values.
 *  @param[in]  accel_bias  New biases.
 *  @return     0 if successful.
 */
int mpu_set_accel_bias(const long *accel_bias);

/**
 *  Read raw gyro data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_gyro_reg(short *data, unsigned long *timestamp);

/**
 *  Read raw accel data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_accel_reg(short *data, unsigned long *timestamp);

/**
 *  Read raw compass data.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_compass_reg(short *data, unsigned long *timestamp); // tej funkcji raczej nie będziemy używać ( a nawet napewno )

/**
 *  Read temperature data directly from the registers.
 *  @param[out] data        Data in q16 format.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_temperature(long *data, unsigned long *timestamp);

/**
 *  Read the MPU interrupt status registers.
 *  @param[out] status  Mask of interrupt bits.
 *  @return     0 if successful.
 */
int mpu_get_int_status(short *status);

/**
 *  Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp, unsigned char *sensors, unsigned char *more);

/**
 *  Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data    FIFO packet.
 *  @param[in]  more    Number of remaining packets.
 */
int mpu_read_fifo_stream(unsigned short length, unsigned char *data, unsigned char *more);

/**
 *  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
int mpu_reset_fifo(void);

/**
 *  Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
int mpu_write_mem(unsigned short mem_addr, unsigned short length, unsigned char *data);

/**
 *  Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
int mpu_read_mem(unsigned short mem_addr, unsigned short length, unsigned char *data);

/**
 *  Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
 */
int mpu_load_firmware(unsigned short length, const unsigned char *firmware, unsigned short start_addr, unsigned short sample_rate);

/**
 *  Register dump for testing.
 *  @return     0 if successful.
 */
int mpu_reg_dump(void);

/**
 *  Read from a single register.
 *  NOTE: The memory and FIFO read/write registers cannot be accessed.
 *  @param[in]  reg     Register address.
 *  @param[out] data    Register data.
 *  @return     0 if successful.
 */
int mpu_read_reg(unsigned char reg, unsigned char *data);

/**
 *  Trigger gyro/accel/compass self-test.
 *  On success/error, the self-test returns a mask representing the sensor(s)
 *  that failed. For each bit, a one (1) represents a "pass" case; conversely,
 *  a zero (0) indicates a failure.
 *
 *  \n The mask is defined as follows:
 *  \n Bit 0:   Gyro.
 *  \n Bit 1:   Accel.
 *  \n Bit 2:   Compass.
 *
 *  \n Currently, the hardware self-test is unsupported for MPU6500. However,
 *  this function can still be used to obtain the accel and gyro biases.
 *
 *  \n This function must be called with the device either face-up or face-down
 *  (z-axis is parallel to gravity).
 *  @param[out] gyro        Gyro biases in q16 format.
 *  @param[out] accel       Accel biases (if applicable) in q16 format.
 *  @return     Result mask (see above).
 */
int mpu_run_self_test(long *gyro, long *accel);

int mpu_register_tap_cb(void (*func)(unsigned char, unsigned char));

#endif  /* #ifndef _INV_MPU_H_ */
