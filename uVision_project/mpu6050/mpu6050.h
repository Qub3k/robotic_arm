/*
 Design Laboratory - Robotic Arm project

 Dzierżewicz Tomasz
 Kwiatosz Michał
 Nawała Jakub

 academic year 2015/2016
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       mpu_6050.h
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 */

#ifndef _INV_MPU_H_
#define _INV_MPU_H_

/* Include the i2c communication header */
#include "i2c_mpu6050.h"

/* Include the header with the mc_delay function */
#include "extra.h"

/* Include the UART communication header */
#include "uart.h"

/* Add the MKL46Z header */
#include "MKL46Z4.h"

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01) // ten define nam się nie przyda, bo nie używamy kompasu

/* Note the compiler that we use the FRDM-KL46Z board */
#define MOTION_DRIVER_TARGET_FRDMKL46Z

struct int_param_s {

// #if defined EMPL_TARGET_MSP430 || defined MOTION_DRIVER_TARGET_MSP430 // nie używamy tych platform więc to makro możemy zakomentować
//     void (*cb)(void);
//     unsigned short pin;
//     unsigned char lp_exit;
//     unsigned char active_low;
// #elif defined EMPL_TARGET_UC3L0 // tej platformy również nie używamy
//     unsigned long pin;
//     void (*cb)(volatile void*);
//     void *arg;

/* W tym miejscu będziemy musieli wstawić pola odpowiednie do zdefiniowania przerwania dla FRDM-MKL46Z
 * Nie wykluczone, że wystarczy jedynie oznaczenie pin'u oraz wskaźnik do funkcji wywoływanej przy przewaniu
 * czyli "callback" lub "ISR" = void (*cb)(void) 
 * Jak zapisać funkcje do takiego wskaźnika?
 * -> musimy najpierw napisać jakąś funkcje:
 *      void my_int_func(int a) {
 *          return a+1;
 *      }
 * -> stowrzyć wskaźnik do funkcji:
 *      void (*callback)(void);
 * -> przypisać funkcje "my_int_func" to wskaźnika funkcji "callback":
 *      callback = &my_int_func; // '&' jest nieobowiązkowy
 * -> w razie problemów ze zrozumieniem polecam przeczytać: http://www.cprogramming.com/tutorial/function-pointers.html
 */

/* Przykładowy kod: */
// #elif defined MKL46Z
  unsigned long pin;
  void (*ISR)(void); // wskaźnik do funkcji "callback"

// #endif
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

/***************************************
 *                                     *
 *           Set up APIs               *
 *                                     *
 ***************************************/
/* Deklaracja funkcji służącej do inicjalizacji urządzenia. */
int mpu_init(struct int_param_s *int_param);
int mpu_init_slave(void);
/* Funkcja ustawiająca urządzenie w tryb "bypass". */
int mpu_set_bypass(unsigned char bypass_on);

/***************************************
 *                                     *
 *        Configuration APIs           *
 *                                     *
 ***************************************/
/* Deklaracja funkcji służącej do wejścia w tryp "lop-power accelerometer-only mode". */
int mpu_lp_accel_mode(unsigned char rate);
/* Enters LP accel motion interrupt mode. */
int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time, unsigned char lpa_freq);
/* Funkcja ustawiająca jaki locziny poziom odpowiada przerwaniu. */
int mpu_set_int_level(unsigned char active_low);
/* Funkcja włączająca tryb "latched interrupts" dla pinu INT. */
int mpu_set_int_latched(unsigned char enable);

/* Enable/disable DMP support. */
int mpu_set_dmp_state(unsigned char enable);
/* Get DMP state. */
int mpu_get_dmp_state(unsigned char *enabled);

/* Funkcja zwracające aktualne ustawienie Digital Low Pass Filter */
int mpu_get_lpf(unsigned short *lpf);
/* Funkcja ustawiająca cut-off filtra dolnoprzepustowego. */
int mpu_set_lpf(unsigned short lpf);

/* Funkcja zwracająca aktualne ustawienia zakresu pracy żyroskopu. */
int mpu_get_gyro_fsr(unsigned short *fsr);
/* Funkcja ustawiająca zakres pracy żyroskopu. */
int mpu_set_gyro_fsr(unsigned short fsr);

/* Funkcja zwracające aktualne ustawienia zakresu pracy akceleromteru. */
int mpu_get_accel_fsr(unsigned char *fsr);
/* Funkcja ustawiająca zakres pracy akcelerometru. */
int mpu_set_accel_fsr(unsigned char fsr);

/* Get the compass full-scale range. */
int mpu_get_compass_fsr(unsigned short *fsr);

/* Funkcja zwracająca ustawienia czułości żyroskopu. */
int mpu_get_gyro_sens(float *sens);
/* Funkcja zwracająca ustawienia czułości akcelerometra. */
int mpu_get_accel_sens(unsigned short *sens);

/* Funkcja zwraca aktualne utawienia "sampling rate". */
int mpu_get_sample_rate(unsigned short *rate);
/* Funkcja ustawiająca sampling rate dla danych pobieranych z czujników. */
int mpu_set_sample_rate(unsigned short rate);
/* Funkcja zwracająca Sampling Rate ustawiony dla kompasu. */
int mpu_get_compass_sample_rate(unsigned short *rate);
/* Funkcja ustawiająca Sampling Rate dla kompasu. */
int mpu_set_compass_sample_rate(unsigned short rate);

/* Funkcja zwracająca ustawienia konfiguracyjne bufora FIFO. */
int mpu_get_fifo_config(unsigned char *sensors);
/* Funkcja ustawiająca, które czujniki mają przekazywać swoje odczyty do bufora First-In-Firsto-Out. */
int mpu_configure_fifo(unsigned char sensors);

/* Funkcja zwracjąca obecny stan zużycia mocy udząrzednia. */
int mpu_get_power_state(unsigned char *power_on);
/* Funkcja dzięki której możemy włączyć konkretne sensory. */
int mpu_set_sensors(unsigned char sensors);

/* Deklaracja funkcji służącej do ustawiania bias'u dla akcelerometra. */
int mpu_set_accel_bias(const long *accel_bias);

/***************************************
 *                                     *
 *      Data getter/setter APIs        *
 *                                     *
 ***************************************/
/* Deklaracja funkcji służącej do sczytywania pomiarów z żrysokpu bezpośrednio z czujnika */
int mpu_get_gyro_reg(short *data, unsigned long *timestamp);
/* Deklaracja funkcji służącej do sczytywania pomiarów z akcelerometru bezpośrednio z czujnika */
int mpu_get_accel_reg(short *data, unsigned long *timestamp);
/* Read raw compass data. */
int mpu_get_compass_reg(short *data, unsigned long *timestamp); // tej funkcji raczej nie będziemy używać ( a nawet napewno )
/* Deklaracja funkcji służącej do sczytywania pomiarów z termometru bezpośrednio z czujnika */
int mpu_get_temperature(long *data, unsigned long *timestamp);

/* Funkcja zwracjąca obecny stan rejestru przerwań. */
int mpu_get_int_status(short *status);
/* Funkcja zwracjąca jeden pakiet z bufora FIFO. */
int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp, unsigned char *sensors, unsigned char *more);
/* Funkcja zwracjąca jeden niestandardowy pakiet z bufora FIFO. */
int mpu_read_fifo_stream(unsigned short length, unsigned char *data, unsigned char *more);
/* Deklaracja funkcji służącej do resetowania bufora FIFO. */
int mpu_reset_fifo(void);

/* Write to the DMP memory. */
int mpu_write_mem(unsigned short mem_addr, unsigned short length, unsigned char *data);
/* Read from the DMP memory. */
int mpu_read_mem(unsigned short mem_addr, unsigned short length, unsigned char *data);
/* Load and verify DMP image. */
int mpu_load_firmware(unsigned short length, const unsigned char *firmware, unsigned short start_addr, unsigned short sample_rate);

/* Deklaracja funkcji służącej do wypisywania zawartości rejstrów urządzenia. */
int mpu_reg_dump(void);
/* Deklaracja funkcji służącej do oczytywania zawartości konkretnego rejestru. */
int mpu_read_reg(unsigned char reg, unsigned char *data);
/* Trigger gyro/accel/compass self-test. */
int mpu_run_self_test(long *gyro, long *accel);
int mpu_register_tap_cb(void (*func)(unsigned char, unsigned char));

#endif  /* #ifndef _INV_MPU_H_ */
