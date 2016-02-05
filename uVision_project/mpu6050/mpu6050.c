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
 *      @file       mpu6050.c
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mpu6050.h"

/**
  * Variable for ISRs preemption.
  */
uint32_t m = 0;

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 
                            
 Poniższy komentarz podkreśla to o czym myślałem już wcześniej. Aby móc korzystać
 z czujnika platforma na której jest on używany (u nas FRDM-KL46Z) musi
 implementować kilka funkcji używanych w tej bibliotece:
 -> i2c_write(unsigned char slave_addr, unsigned char reg_addr,
              unsigned char length, unsigned char const *data)
 -> i2c_read(unsigned char slave_addr, unsigned char reg_addr,
             unsigned char length, unsigned char *data)
 -> delay_ms(unsigned long num_ms)
 -> get_ms(unsigned long *count)
 -> reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin) =>  register the interrupt callback
                                                                            interrupt callback to po prostu ISR
                                                                            czyli Interrup Service Routine
 -> labs(long x) => wartość bezwzględna liczby w formacie "long int"
 -> fabsf(float x) => wartość bezwzględna liczby w formacie "float"
 -> min(int a, int b)
 -> log_i => z tego co zrozumiałem to jest to funkcja służąca do logowanie przebiegu pracy czujnika
             można ją zaimplementować jako nic nie robiącą funkcje: do{} while(0) 
 -> log_e => z tego co zrozumiałem to jest to funkcja służąca do logowanie przebiegu pracy czujnika
             można ją zaimplementować jako nic nie robiącą funkcje: do{} while(0)

  W tej konkretnej implementacji, twórcy posługują się makrami preprocesora,
  aby włączyć kod odpowiedni dla urządzenia.

 ******************************************************************************/


void PORTA_IRQHandler(void) {
  /* Do something */
}
#define delay_ms wait_ms
#define log_i uart_transmit
#define log_e uart_transmit
#define min(a,b) ((a<b)?a:b) 
#define i2c_read i2c_read_registers
#define i2c_write i2c_write_registers
static inline int reg_int_cb(struct int_param_s *int_param) { // pomyśl jak zaimplementować tę funkcje
  /* Configure the interrupt routine for a proper interrupt set at MPU */
  int_param->ISR = PORTA_IRQHandler;
  return 0;
}
unsigned long int get_ms(unsigned long int *count) { 
  /* Read the time */
  m = __get_PRIMASK();
  __disable_irq();
  count[0] = milliseconds;
  __set_PRIMASK(m);
  return 0;
};

/* Zdefiniujmy makro mówiące o tym jakiego używamy czujnika - będzie to potrzebne */ 
/* w dalszych częściach kodu */
#define MPU6050

/* Deklaracja funkcji służącej do włączania przerwania "data ready" */
static int set_int_enable(unsigned char enable);

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 
                            
 Struktura zdefniowana poniżej może być punktem wyjścia dla naszej biblioteki.
 Zawiera ona definicje każdego rejestru jako 8-bitowy typ "unsigned char". Stoi
 to w zgodności z data sheet'em czujnika, gdyż rzeczywiście rejstry urządzenia
 są 8-bitowe.
 Dodatkowo, można od razu zauważyć, że każdy typ zdefiniowany w kodzie jako
 struktura kończy się na "_s"

 ******************************************************************************/
/* Hardware registers needed by driver. */
struct gyro_reg_s {
    unsigned char who_am_i;
    unsigned char rate_div;
    unsigned char lpf;
    unsigned char prod_id;
    unsigned char user_ctrl;
    unsigned char fifo_en;
    unsigned char gyro_cfg;
    unsigned char accel_cfg;
    unsigned char accel_cfg2;
    unsigned char lp_accel_odr;
    unsigned char motion_thr;
    unsigned char motion_dur;
    unsigned char fifo_count_h;
    unsigned char fifo_r_w;
    unsigned char raw_gyro;
    unsigned char raw_accel;
    unsigned char temp;
    unsigned char int_enable;
    unsigned char dmp_int_status;
    unsigned char int_status;
    unsigned char accel_intel;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char int_pin_cfg;
    unsigned char mem_r_w;
    unsigned char accel_offs;
    unsigned char i2c_mst;
    unsigned char bank_sel;
    unsigned char mem_start_addr;
    unsigned char prgm_start_h;
};

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Poniższa struktura to zbiór informacji przypisany do konkretnego urządznenia.
 W tym miejscu jest to tylko deklaracja, zaś definicja konkretnych wartości
 pól z tej struktury znajduje się poniżej.

 ******************************************************************************/
/* Information specific to a particular device. */
struct hw_s {
    unsigned char addr;       // adres I2C urządzenia
    unsigned short max_fifo;  // max. rozmiar bufora FIFO
    unsigned char num_reg;    // liczba rejestrów dostępnych dla czujnika
    unsigned short temp_sens; // stała służąca do wyliczania temperatury dla danego czujnika
    short temp_offset;        // kolejna stała do kalibracji pomiaru temperatury
    unsigned short bank_size;
};

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Poniższa struktura to zbiór danych używanych do przechowywania informacji
 na temat stanu poprzedzającego przejście do przerwania "Motion Interrupt"

 ******************************************************************************/
/* When entering motion interrupt mode, the driver keeps track of the
 * previous state so that it can be restored at a later time.
 * TODO: This is tacky. Fix it.
 */
struct motion_int_cache_s {
    unsigned short gyro_fsr; 
    unsigned char accel_fsr; 
    unsigned short lpf; 
    unsigned short sample_rate; 
    unsigned char sensors_on; 
    unsigned char fifo_sensors; 
    unsigned char dmp_on;
};

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Poniższa struktura służy przechowywaniu aktualnie ustawionej konfiguracji
 czujnika (tak mi się wydaje -> info czeka na potwierdzenie)

 ******************************************************************************/
/* Cached chip configuration data.
 * TODO: A lot of these can be handled with a bitmask.
 */
struct chip_cfg_s {
    /* Matches gyro_cfg >> 3 & 0x03 */
    unsigned char gyro_fsr;
    /* Matches accel_cfg >> 3 & 0x03 */
    unsigned char accel_fsr;
    /* Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2. */
    unsigned char sensors;
    /* Matches config register. */
    unsigned char lpf;
    unsigned char clk_src;
    /* Sample rate, NOT rate divider. */
    unsigned short sample_rate;
    /* Matches fifo_en register. */
    unsigned char fifo_enable;
    /* Matches int enable register. */
    unsigned char int_enable;
    /* 1 if devices on auxiliary I2C bus appear on the primary. */
    unsigned char bypass_mode;
    /* 1 if half-sensitivity.
     * NOTE: This doesn't belong here, but everything else in hw_s is const,
     * and this allows us to save some precious RAM.
     */
    unsigned char accel_half;
    /* 1 if device in low-power accel-only mode. */
    unsigned char lp_accel_mode;
    /* 1 if interrupts are only triggered on motion events. */
    unsigned char int_motion_only;
    struct motion_int_cache_s cache;
    /* 1 for active low interrupts. */
    unsigned char active_low_int;
    /* 1 for latched interrupts. */
    unsigned char latched_int;
    /* 1 if DMP is enabled. */
    unsigned char dmp_on;
    /* Ensures that DMP will only be loaded once. */
    unsigned char dmp_loaded;
    /* Sampling rate used when DMP is enabled. */
    unsigned short dmp_sample_rate;
};

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Poniższa struktura służy przechowywaniu danych używanych do kalibracji czunika

 ******************************************************************************/
/* Information for self-test. */
struct test_s {
    unsigned long gyro_sens;
    unsigned long accel_sens;
    unsigned char reg_rate_div;
    unsigned char reg_lpf;
    unsigned char reg_gyro_fsr;
    unsigned char reg_accel_fsr;
    unsigned short wait_ms;
    unsigned char packet_thresh;
    float min_dps;
    float max_dps;
    float max_gyro_var;
    float min_g;
    float max_g;
    float max_accel_var;
};

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Poniższa struktura łączy wcześniej opisane struktury tworząc spójny kontener
 w pełni opisujący stan naszego urządzenia.

 Co może przez chwilę zastanawiać to struktura wewnątrz sktruktury, ale coś 
 takiego jest dozwolone w C.

 Jako przykład na zrozumienie załóżmy, że tworzymy obiekt struktury "gyro_state_s"
 o nazwie "state":

    struct gyro_state_s state;

 I teraz, żeby dostać się do pola o nazwie "foo" w obiekcie podstruktury "chip_cfg_s"
 znajdującej się wewnątrz struktury "gyro_state_s" używamy następującej składni:

    state.chip_cfg.foo

 Zakładając, że obiekt struktury "chip_cfg_s" został zdefiniowany wewnątrz struktury
 "gyro_state_s" jako "chip_cfg"

 Wracając do poniższej struktury należy zauważyć, że w tym miejscu, pola obiektu
 "chip_cfg" struktury "chip_cfg_s" nie mają określonych wartości (są tylko
 zdefiniowane).

 ******************************************************************************/
/* Gyro driver state variables. */
struct gyro_state_s {
    const struct gyro_reg_s *reg;
    const struct hw_s *hw;
    struct chip_cfg_s chip_cfg;
    const struct test_s *test;
};

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Poniżej znajduje się kilka typów wyliczeniowych wygodnych do konfiguracji
 określonych funkcji wewnątrz niżej zdefiniowanych metod.

 Należy pamiętac, że typy wilczeniowe działają jak zwykły TYP zmiennej w C.
 I tak, każda zmienna o typie "lpf_e" może mieć wartości: INV_FILTER_256HZ_NOLPF2,
 INV_FILTER_188HZ, INV_FILTER_98HZ itd.

 O czym jeszcze należy pamiętać to fakt, iż domyślnie, pierwszy elemnt z listy
 ma wartość 0. Potem, kolejne elemnty mają wartości przypisane ciągowi liczb
 całkowitych rosnącyh co 1.

 Dla przejrzystoći kodu częst wprost pisze się, że pierwszy element ma wartość 0.

 ******************************************************************************/
/* Filter configurations. */
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ, // = 1
    INV_FILTER_98HZ, // = 2
    INV_FILTER_42HZ, // = 3
    INV_FILTER_20HZ, // = 4
    INV_FILTER_10HZ, // = 5
    INV_FILTER_5HZ, // = 6
    INV_FILTER_2100HZ_NOLPF, // = 7
    NUM_FILTER // = 8
};

/* Full scale ranges. */
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS, // = 1
    INV_FSR_1000DPS, // = 2
    INV_FSR_2000DPS, // = 3
    NUM_GYRO_FSR // = 4
};

/* Full scale ranges. */
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G, // = 1
    INV_FSR_8G, // = 2
    INV_FSR_16G, // = 3
    NUM_ACCEL_FSR // = 4
};

/* Clock sources. */
enum clock_sel_e {
    INV_CLK_INTERNAL = 0,
    INV_CLK_PLL, // = 1
    NUM_CLK // = 2
};

/* Low-power accel wakeup rates. */
enum lp_accel_rate_e {
// #if defined MPU6050
    INV_LPA_1_25HZ, // = 0
    INV_LPA_5HZ, // = 1
    INV_LPA_20HZ, // = 2
    INV_LPA_40HZ // = 3
};

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Poniżej znajdują się definicje bitów dla określonych rejestrów. I tak, np., 
 rejestr "PWR_MGMT_1" ma bit o nazwie "DEVICE_RESET" na miejscu 7, co tutaj
 zdefiniowane jest jako "BIT_RESET" = 0x80 = 128 = 0b1000 0000

 Można sie domyślić, iż reguła jest taka, że definicja bitu w danym rejestrze
 zawsze zaczyna się od słowa kluczowego "BIT_"

 ******************************************************************************/
#define BIT_I2C_MST_VDDIO   (0x80)
#define BIT_FIFO_EN         (0x40)
#define BIT_DMP_EN          (0x80)
#define BIT_FIFO_RST        (0x04)
#define BIT_DMP_RST         (0x08)
#define BIT_FIFO_OVERFLOW   (0x10)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)
#define BIT_MOT_INT_EN      (0x40)
#define BITS_FSR            (0x18)
#define BITS_LPF            (0x07)
#define BITS_HPF            (0x07)
#define BITS_CLK            (0x07)
#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_RESET           (0x80)
#define BIT_SLEEP           (0x40)
#define BIT_S0_DELAY_EN     (0x01)
#define BIT_S2_DELAY_EN     (0x04)
#define BITS_SLAVE_LENGTH   (0x0F)
#define BIT_SLAVE_BYTE_SW   (0x40)
#define BIT_SLAVE_GROUP     (0x10)
#define BIT_SLAVE_EN        (0x80)
#define BIT_I2C_READ        (0x80)
#define BITS_I2C_MASTER_DLY (0x1F)
#define BIT_AUX_IF_EN       (0x20)
#define BIT_ACTL            (0x80)
#define BIT_LATCH_EN        (0x20)
#define BIT_ANY_RD_CLR      (0x10)
#define BIT_BYPASS_EN       (0x02)
#define BITS_WOM_EN         (0xC0)
#define BIT_LPA_CYCLE       (0x20)
#define BIT_STBY_XA         (0x20)
#define BIT_STBY_YA         (0x10)
#define BIT_STBY_ZA         (0x08)
#define BIT_STBY_XG         (0x04)
#define BIT_STBY_YG         (0x02)
#define BIT_STBY_ZG         (0x01)
#define BIT_STBY_XYZA       (BIT_STBY_XA | BIT_STBY_YA | BIT_STBY_ZA)
#define BIT_STBY_XYZG       (BIT_STBY_XG | BIT_STBY_YG | BIT_STBY_ZG)

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Kod, który znajduje się poniżej to bardzo przydatna struktura, która przypisuje
 odpowiednim rejestrom MPU-6050 właściwe adresy zapisane w HEX'sie. Sprawdziłem
 to i wszystko zgadza się z data sheet'em czujnika. 
 Żeby zrozumieć co się tutaj dzieje najpierw trzeba zobaczyć deklaracje 
 struktury "gyro_reg_s", które może być znelziona ok. 300 linijek powyżej.

 Metoda przypisywania wartości do pól wewnątrz obiektu "reg" struktury "gyro_reg_s" 
 jest jak najbaridziej poprawna i używana, gdy chcemy podkreślić, które dokładnie
 pola inicjalizujemy.

 ******************************************************************************/
const struct gyro_reg_s reg = {
    .who_am_i       = 0x75,
    .rate_div       = 0x19,
    .lpf            = 0x1A,
    .prod_id        = 0x0C,
    .user_ctrl      = 0x6A,
    .fifo_en        = 0x23,
    .gyro_cfg       = 0x1B,
    .accel_cfg      = 0x1C,
    .motion_thr     = 0x1F,
    .motion_dur     = 0x20,
    .fifo_count_h   = 0x72,
    .fifo_r_w       = 0x74,
    .raw_gyro       = 0x43,
    .raw_accel      = 0x3B,
    .temp           = 0x41,
    .int_enable     = 0x38,
    .dmp_int_status = 0x39,
    .int_status     = 0x3A,
    .pwr_mgmt_1     = 0x6B,
    .pwr_mgmt_2     = 0x6C,
    .int_pin_cfg    = 0x37,
    .mem_r_w        = 0x6F,
    .accel_offs     = 0x06,
    .i2c_mst        = 0x24,
    .bank_sel       = 0x6D,
    .mem_start_addr = 0x6E,
    .prgm_start_h   = 0x70};

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja wartości strutkury opisującej konkretny czujnik. 
 Na pierwszy rzut oka wygląda na to, że wszystko się zgadza.

 ******************************************************************************/
const struct hw_s hw = {
    .addr           = 0x68,
    .max_fifo       = 1024,
    .num_reg        = 118,
    .temp_sens      = 340,
    .temp_offset    = -521,
    .bank_size      = 256
};

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja wartości strutkury używanej do kalibracji czujnika.

 Pierwsze dwie wartości to czułość żyroskopu oraz akceleroemtra wyrażażona jako
 LSB (Least Significatn Bit) po konwersji 16-bitowym ADC (od -32767 do 32768)
 zakładając, że zakres pracy żyroskopu to +/- 250 dps (degrees per second),
 a ackelerometru +/- 16g.

 Pozostałe stałe to wartości niezbędne do odpowiedniej kalibracji.

 ******************************************************************************/
const struct test_s test = {
    .gyro_sens      = 32768/250,
    .accel_sens     = 32768/16,
    .reg_rate_div   = 0,    /* 1kHz. */
    .reg_lpf        = 1,    /* 188Hz. */
    .reg_gyro_fsr   = 0,    /* 250dps. */
    .reg_accel_fsr  = 0x18, /* 16g. */
    .wait_ms        = 50,
    .packet_thresh  = 5,    /* 5% */
    .min_dps        = 10.f,
    .max_dps        = 105.f,
    .max_gyro_var   = 0.14f,
    .min_g          = 0.3f,
    .max_g          = 0.95f,
    .max_accel_var  = 0.14f
};

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 W tym miejscu następuje inicjalizacja obiektu "st" struktury przechowującej stan 
 naszego czujnika ("gyro_state_s").

 &reg to adres obiektu struktury "gyro_reg_s" przechowujący adresy w pamięci
      wszystkich rejestrów

 &hw to adres obiektu struktury "hw_s" zawierający dane sprzętowe naszego czujnika
     takie jak: adres I2C, rozmiar bufora FIFO itd.

 &test to adres obiektu struktury "test_s", który to obiekt zawiera dane 
       niezbędne do poprawnego wykonania funkcji "self-test"

 ******************************************************************************/
static struct gyro_state_s st = {
    .reg = &reg, 
    .hw = &hw,
    .test = &test
};

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja stałej, której mówi ile jednorazowo bajtów można odczytać z bufora
 FIFO.

 ******************************************************************************/
#define MAX_PACKET_LENGTH (12) 

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja funkcji służącej do włączania przerwania "data ready"

 ******************************************************************************/
/**
 *  @brief      Enable/disable data ready interrupt.
 *  If the DMP is on, the DMP interrupt is enabled. Otherwise, the data ready
 *  interrupt is used.
 *  @param[in]  enable      1 to enable interrupt.
 *  @return     0 if successful.
 */
static int set_int_enable(unsigned char enable)
{
    unsigned char tmp;

    if (st.chip_cfg.dmp_on) { // sprawdź czy Digital Motion Processor jest włączony
        if (enable) // jeśli użytkownik chce włączyć ten rodzaj przerwania 
            tmp = BIT_DMP_INT_EN; // to ustaw zmienną tmp na bit 2
        else
            tmp = 0x00;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp)) // ustaw 2 bit w rejestrze INT_ENABLE
            return -1;
        st.chip_cfg.int_enable = tmp; // zmień ustawienia w strukturze przechowującej status czujnika
    } else { // jeśli DMP nie jest włączony
        if (!st.chip_cfg.sensors) 
            return -1;
        if (enable && st.chip_cfg.int_enable) // sprawdź czy przypadkiem to przerwania nie zostało już wcześniej włączone
            return 0;
        if (enable)
            tmp = BIT_DATA_RDY_EN; 
        else
            tmp = 0x00;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &tmp)) // Ustaw bit BIT_DATA_RDY_EN w rejestrze INT_ENABLE aby włączyć przerwanie
            return -1;
        st.chip_cfg.int_enable = tmp;
    }
    return 0;
}

/**
  * Calibrate the gyro by returning the bias that shall be subracted from the
  * readings taken from the unit.
  * @param[out] gyro_bias 3-elements array with the bias for each axis of gyro
  * @return 0 if sucessful.
  */
int mpu_calibate_gyro(int8_t *gyro_bias){
  short gyro_readings[3] = {0, 0, 0};
  int32_t gyro_readings_sum[3] = {0, 0, 0};
  uint8_t i = 0; 
 
  /* Read and sum 20 readings from MPU-6050 */
  for(i = 0; i < 20; i++){
    /* Read the readings */
    if(mpu_get_gyro_reg(gyro_readings, (unsigned long *)NULL))
      return -1;
    
    /* Sum up to readings */
    gyro_readings_sum[0] += gyro_readings[0];
    gyro_readings_sum[1] += gyro_readings[1];
    gyro_readings_sum[2] += gyro_readings[2];
    
    /* Wait for the next result */
    wait_ms(20);
  }
  
  /* Calculate the average value for each axis */
  gyro_bias[0] = gyro_readings_sum[0]/20;
  gyro_bias[1] = gyro_readings_sum[1]/20;
  gyro_bias[2] = gyro_readings_sum[2]/20;

  return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja funkcji służącej do wypisywania zawartości rejstrów urządzenia.

 ******************************************************************************/
/**
 *  @brief      Register dump for testing.
 *  @return     0 if successful.
 */
int mpu_reg_dump(void)
{
    unsigned char ii;
    unsigned char data;

    for (ii = 0; ii < st.hw->num_reg; ii++) { // iteruj po wszystkich rejestrach urządzenia
        if (ii == st.reg->fifo_r_w || ii == st.reg->mem_r_w) // pomiń rejestr FIFO_R_W oraz MEM_R_W
            continue;
        if (i2c_read(st.hw->addr, ii, 1, &data)) // jeśli odczytanie któregokolwiek rejestru się nie powiedzie to...
            return -1; //... przerwij działanie funkcji
        log_i("%#5x: %#5x\r\n", ii, data); // przekaż do funkcji logującej zawartość każdego rejestru
    }
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja funkcji służącej do oczytywania zawartości konkretnego rejestru.

 ******************************************************************************/
/**
 *  @brief      Read from a single register.
 *  NOTE: The memory and FIFO read/write registers cannot be accessed.
 *  @param[in]  reg     Register address.
 *  @param[out] data    Register data.
 *  @return     0 if successful.
 */
int mpu_read_reg(unsigned char reg, unsigned char *data)
{
    if (reg == st.reg->fifo_r_w || reg == st.reg->mem_r_w) // jeśli użytkownik spróbuje odczytać rejstr FIFO_R_W lub MEM_R_W to przerwij działanie funkcji
        return -1;
    if (reg >= st.hw->num_reg) // jeśli użytkonik próbuje odczytać rejestr spoza dostępnej ilości rejestrów to przerwij działanie
        return -1;
    return i2c_read(st.hw->addr, reg, 1, data); // W każdym innym wypadku, oczytaj rejestr "reg" i zapisz jego zawartość do zmiennej "data"
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja funkcji służącej do inicjalizacji urządzenia.

 Domyślna konfiguracja:
    -> żyroskop: +/- 2000 dps
    -> akcelerometr: +/- 2 g
    -> Digital Low Pass Filter Cut-off: 42 Hz
    -> Sample rate: 50 Hz
    -> Clock source: zegar żyroskopu z PLL (Phased Locked Loop)
    -> FIFO: wyłączone.
    -> Data ready interrupt: wyłączony, active when LOW, unlatched.

 ******************************************************************************/
/**
 *  @brief      Initialize hardware.
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
int mpu_init(struct int_param_s *int_param){
    unsigned char data[6], rev;

    /* Reset device. */
    data[0] = BIT_RESET;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
        return -1;
    delay_ms(100);

    /* Wake up chip. */
    data[0] = 0x00;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
        return -1;

    /* Check product revision. */
    if (i2c_read(st.hw->addr, st.reg->accel_offs, 6, data))
        return -1;
    rev = ((data[5] & 0x01) << 2) | ((data[3] & 0x01) << 1) | (data[1] & 0x01);

    if (rev) {
        /* Congrats, these parts are better. */
        if (rev == 1)
            st.chip_cfg.accel_half = 1;
        else if (rev == 2)
            st.chip_cfg.accel_half = 0;
        else {
            log_e("Unsupported software product rev %d.\r\n", rev);
            return -1;
        }
    } else {
        if (i2c_read(st.hw->addr, st.reg->prod_id, 1, data))
            return -1;
        rev = data[0] & 0x0F;
        if (!rev) {
            log_e("Product ID read as 0 -> device is incompatible or an MPU3050.\r\n");
            return -1;
        } else if (rev == 4) {
            log_i("Half sensitivity part found.\n");
            st.chip_cfg.accel_half = 1;
        } else
            st.chip_cfg.accel_half = 0;
    }

    /* Set to invalid values to ensure no I2C writes are skipped. */
    st.chip_cfg.sensors = 0xFF;
    st.chip_cfg.gyro_fsr = 0xFF;
    st.chip_cfg.accel_fsr = 0xFF;
    st.chip_cfg.lpf = 0xFF;
    st.chip_cfg.sample_rate = 0xFFFF; // Sample rate może być większe niż 255 więc używamy typu 16-bitowego
    st.chip_cfg.fifo_enable = 0xFF;
    st.chip_cfg.bypass_mode = 0xFF;

    /* Kolejne linijki nic nie ustawiając w urządzeniu a jednie zmieniają dane struktury, która
     * przechowuje obecną konfigurację czujnika - to co się tu dzieje to zapewne wpisywanie wartości
     * domyślnych, które pojawiają się po każdym resecie urządzenia
     * Zgodnie z data sheet'em, po restarcie:
     *  -> wszystkie rejestry zapisane są wartościami 0x00
     *  -> z wyjątkiem rejestru 107(PWR_MGMT_1), który zainicjalizowany jest wartością 0x40 => domyślnie urządzenie jest w trybie "SLEEP"
     *  -> oraz z wyjątkiem rejestru 117(WHO_AM_I), który zainicjalizowany jest wartością 0x68 => adres I2C urządzenia 
     */

    /* mpu_set_sensors always preserves this setting. */
    st.chip_cfg.clk_src = INV_CLK_PLL;            // pierwsze użycie typu wyliczeniowego "clock_sel_e" mówiące o tym, że będziemy korzystać z bardziej dokładnego zegara PLL żyroskopu
    /* Handled in next call to mpu_set_bypass. */
    st.chip_cfg.active_low_int = 1;               // Ustaw pin "INT" w modelu "Active when LOW"
    st.chip_cfg.latched_int = 0;                  // Ustawienie latch_int = 0 sprawia, że przerwania na pinie "INT" objawiają się jako pulsy o szerokości 50us
    st.chip_cfg.int_motion_only = 0;              // tak producent opisał to ustawienie: "1 if interrupts are only triggered on motion events."
    st.chip_cfg.lp_accel_mode = 0;                // tak producent opisał to ustawienie: "1 if device in low-power accel-only mode."
    memset(&st.chip_cfg.cache, 0, sizeof(st.chip_cfg.cache)); // wyzeruj cały obiekt struktury "motion_int_cache_s"
    st.chip_cfg.dmp_on = 0;                       // domyślnie, wyłącz Digital Motion Processor. Opis producenta: "1 if DMP is enabled."
    st.chip_cfg.dmp_loaded = 0;                   // opis producenta dla tego pola: "Ensures that DMP will only be loaded once."
    st.chip_cfg.dmp_sample_rate = 0;              // opis producenta: "Sampling rate used when DMP is enabled."

    /* Program the unit with the default configuration */
    if (mpu_set_gyro_fsr(2000)){
        return -1;
    }else{
      log_i("\tGyro FSR \t\t= +/- 2000 dps\r\n");
    }
    if (mpu_set_accel_fsr(2)){
        return -1;
    }else{
      log_i("\tAccel FSR \t\t= +/- 2 g\r\n");
    }
    if (mpu_set_lpf(42)){
        return -1;
    }else{
      log_i("\tLow-pass cut-off \t= 42 Hz\r\n");
    }
    if (mpu_set_sample_rate(50)){
        return -1;
    }else{
      log_i("\tSample rate \t\t= 50 Hz\r\n");
    }
    if (mpu_configure_fifo(0)){
        return -1;
    }else{
      log_i("\tFIFO \t\t\t= disabled\r\n");
    }

    if (int_param)              // jeśli adres do struktury konfigurującej przerwania dla danej platformty jest inny niż 0(NULL pointer) to...
        reg_int_cb(int_param);  // ...skonfiguruj odpowiednio przerwania dla danej platformy

    if (mpu_set_bypass(0)) {
        return -1;
    }else{
      log_i("\tBypass mode \t\t= disabled\r\n");
    }

    mpu_set_sensors(0);
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja funkcji służącej do wejścia w tryp "lop-power accelerometer-only mode".

 Podanie tej funkcji argumentu o wartości '0' wyłącza tryb "low-power".

 W trybie "low-power" chip śpi i budzi się tylko gdy przychodzi kolejna próbka 
 danych z akcelerometru (tylko i wyłącznie!).

 Dozwolona częstotliwość dla MPU-6050 to:
 -> 1.25 Hz,
 -> 5 Hz,
 -> 20 Hz
 -> oraz 40 Hz.

 Według data sheet'a, procedura wprowadzania urządzenia w stan "low power" jest
 następująca:
 1. set CYCLE bit in PRW_MGMT_1 register to '1',
 2. set SLEEP bit in PWR_MGMT_1 register to '0',
 3. set TEMP_DIS bit in PWR_MGMT_1 register to '1', => wyłącza czujnik temperatury
 4. set STBY_XG, STBY_YG, STBY_ZG bits to 1 in PWR_MGMT_2 register to '1'.

 ******************************************************************************/
/**
 *  @brief      Enter low-power accel-only mode.
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
int mpu_lp_accel_mode(unsigned char rate)
{
    unsigned char tmp[2]; // należy zwrócić uwagę, że zmienna "tmp" jest 2 bajtowa

    if (rate > 40) // jeśli arugment wykracza poza największą dozwoloną częstotliwość pomiarów to przerwij działanie
        return -1;

    if (!rate) { // jeśli rate ma wartość '0'
        mpu_set_int_latched(0); // wyłącz tryb "latched interrupts"
        tmp[0] = 0; // wyzeruj pierwsze 8 bajtów zmiennej "tmp"
        tmp[1] = BIT_STBY_XYZG; // wyłącz żyroskop
        if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp)) // wpisz dwa bajty danych z "tmp" zaczynając od rejestru PWR_MGMT_1
            return -1;
        st.chip_cfg.lp_accel_mode = 0; // zapisz nowe ustawienia w naszej strukturze konfiguracyjnej
        return 0; // wyjdź z funkcji
    }
    /* For LP accel, we automatically configure the hardware to produce latched
     * interrupts. In LP accel mode, the hardware cycles into sleep mode before
     * it gets a chance to deassert the interrupt pin; therefore, we shift this
     * responsibility over to the MCU.
     *
     * Any register read will clear the interrupt.
     */
    mpu_set_int_latched(1); // jeśli chcemy włączyć "low-power mode" to włączamy najpierw "latched interrupts"
// #if defined MPU6050
    tmp[0] = BIT_LPA_CYCLE; // pierwsze krok procedury = ustaw CYCLE mode
    if (rate == 1) { // wybierz odpowiedni sample rate na podstawie podanego argumentu
        tmp[1] = INV_LPA_1_25HZ;
        mpu_set_lpf(5); // razem z "sampling rage" ustaw również filtr dolnoprzepustowy
    } else if (rate <= 5) {
        tmp[1] = INV_LPA_5HZ;
        mpu_set_lpf(5);
    } else if (rate <= 20) {
        tmp[1] = INV_LPA_20HZ;
        mpu_set_lpf(10);
    } else {
        tmp[1] = INV_LPA_40HZ;
        mpu_set_lpf(20);
    }
    tmp[1] = (tmp[1] << 6) | BIT_STBY_XYZG; // przesuń ustawienia z poprzednich kilku linijek 0 6 bitów w lewo i ustaw bity wyłączające żyroskop
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, tmp)) // zapisz dwa bajty "tmp" do dwóch rejestrów, zaczynając oc "PWR_MGMT_1"
        return -1;
    
    st.chip_cfg.sensors = INV_XYZ_ACCEL; // zapisz nowe ustawienia w strukturze konfiguracyjnej
    st.chip_cfg.clk_src = 0;
    st.chip_cfg.lp_accel_mode = 1;
    mpu_configure_fifo(0); // nie przekierowuj żadnych danych do bufora FIFO

    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja funkcji służącej do sczytywania pomiarów z żrysokpu bezpośrednio
 z czujnika - bez żadnej dodatkowej obróbki lub obliczeń.

 Dodatkowo, funkcja zwraca znacznik czasu pomariu (po to potrzebna jest
 definicja funkcji "get_ms()").

 ******************************************************************************/
/**
 *  @brief      Read raw gyro data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_gyro_reg(short *data, unsigned long *timestamp)
{
    unsigned char tmp[6]; // utwórz 6 bajtową zmienną

    if (!(st.chip_cfg.sensors & INV_XYZ_GYRO)) // jeśli żyroskop nie jest włączony to zakończ działanie i ustaw kod błędu
        return -1;

    if (i2c_read(st.hw->addr, st.reg->raw_gyro, 6, tmp)) // oczytaj 6 bajtów danych zaczynając od rejestru RAW_GYRO
        return -1;
    data[0] = (tmp[0] << 8) | tmp[1]; // przez to, że rejestry są 8-bitowe, a dane 16-bitowe to trzeba górny bajt przesunać o 8 bitów w lewo
    data[1] = (tmp[2] << 8) | tmp[3];
    data[2] = (tmp[4] << 8) | tmp[5];
    if (timestamp) // jeśli adres wskaźnika "timestamp" jest inny niż zero to:
        get_ms(timestamp); // zapisz również znacznik czasu pomiaru
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja funkcji służącej do sczytywania pomiarów z akcelerometru bezpośrednio
 z czujnika - bez żadnej dodatkowej obróbki lub obliczeń.

 Dodatkowo, funkcja zwraca znacznik czasu pomariu (po to potrzebna jest
 definicja funkcji "get_ms()").

 ******************************************************************************/
/**
 *  @brief      Read raw accel data directly from the registers.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_accel_reg(short *data, unsigned long *timestamp)
{
    unsigned char tmp[6]; // utwórz 6 bajtową zmienną

    if (!(st.chip_cfg.sensors & INV_XYZ_ACCEL)) // jeśli akcelerometr jest wyłączony to przerwij działanie
        return -1;

    if (i2c_read(st.hw->addr, st.reg->raw_accel, 6, tmp)) // odczytaj 6 bajtów zaczynać od rejestu "RAW_ACCEL"
        return -1;
    data[0] = (tmp[0] << 8) | tmp[1]; // procedura odczytu jest taka sama jak w żyroskopie (patrz funkcja wyżej)
    data[1] = (tmp[2] << 8) | tmp[3];
    data[2] = (tmp[4] << 8) | tmp[5];
    if (timestamp)
        get_ms(timestamp);
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja funkcji służącej do sczytywania pomiarów z termometru bezpośrednio
 z czujnika - bez żadnej dodatkowej obróbki lub obliczeń.

 Co jest nakbardziej przyjemne to to, że funkcja sama przetwarza dane 
 i zwraca wynik w stopniach celcjusza.

 Dodatkowo, funkcja zwraca znacznik czasu pomariu (po to potrzebna jest
 definicja funkcji "get_ms()").

 ******************************************************************************/
/**
 *  @brief      Read temperature data directly from the registers.
 *  @param[out] data        Data in q16 format.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_temperature(long *data, unsigned long *timestamp)
{
    unsigned char tmp[2]; // utwórz 2 bajtową zmienną
    short raw; // utwórz 2 bajtową zmienną (16-bitów)

    if (!(st.chip_cfg.sensors)) // jeśli wszystkie czujniki sa wyłączone to przerwij działanie
        return -1;

    if (i2c_read(st.hw->addr, st.reg->temp, 2, tmp)) // oczytaj 2 bajty zaczynając od rejestru "TEMP"
        return -1;
    raw = (tmp[0] << 8) | tmp[1]; // wpisz odczytane dane do zmiennej "raw"
    if (timestamp)
        get_ms(timestamp); // zapisz znacznik czasu w parametrze "timestamp"

    data[0] = (long)((35 + ((raw - (float)st.hw->temp_offset) / st.hw->temp_sens)) * 65536L); // zapisz temperature w zmiennej podanej jako paramter, po ówczesnym zastosowaniu wzoru do kalibracji
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja funkcji służącej do ustawiania bias'u dla akcelerometra.

 Jest to funkcja, która służy do ustawienia odpowiedniej kalibracji akcelerometra.

 Funkcja sczytuje najpierw wartości kalibracyjne ustawione przez producenta podczas
 fabrykacji urządzenia.

 Tej funkcji używa się podczas kalibracji urządzenia po wykonaniu i zapisaniu 
 wyników "self-test".

 ******************************************************************************/
/**
 *  @brief      Push biases to the accel bias registers.
 *  This function expects biases relative to the current sensor output, and
 *  these biases will be added to the factory-supplied values.
 *  @param[in]  accel_bias  New biases.
 *  @return     0 if successful.
 */
int mpu_set_accel_bias(const long *accel_bias)
{
    unsigned char data[6]; // utwórz 6 bajtową zmienną
    short accel_hw[3]; // utwórz 6 bajtową zmienną = 3 komórki po 2 bajty każda
    short got_accel[3]; // utwórz 6 bajtową zmienną = 3 komórki po 2 bajty każda
    short fg[3]; // utwórz 6 bajtową zmienną = 3 komórki po 2 bajty każda

    if (!accel_bias) // jeśli wskaźnik do "accel_bias" to zero 
        return -1; // to zakończ działanie
    if (!accel_bias[0] && !accel_bias[1] && !accel_bias[2]) // jeśli wszystkie wartości w "accel_bias" to zera to zakończ działanie
        return 0;

    if (i2c_read(st.hw->addr, 3, 3, data)) // przeczytaj 3 bajty zaczynając od rejstru 3 (nie ma go w datasheet'cie) = "factory trim", parametry kalibracyjne ustawione przy fabrykacji chip'u
        return -1;
    fg[0] = ((data[0] >> 4) + 8) & 0xf;
    fg[1] = ((data[1] >> 4) + 8) & 0xf;
    fg[2] = ((data[2] >> 4) + 8) & 0xf;

    accel_hw[0] = (short)(accel_bias[0] * 2 / (64 + fg[0])); // używając zadanego bias'u oraz wartości fabrycznych, zapisza kalibracje bias'ingu akcelerometra
    accel_hw[1] = (short)(accel_bias[1] * 2 / (64 + fg[1]));
    accel_hw[2] = (short)(accel_bias[2] * 2 / (64 + fg[2]));

    if (i2c_read(st.hw->addr, 0x06, 6, data)) // przeczytaj 6 bajtów zaczynając od rejestru 6 (nie ma go w datasheet'cie) = obecny bias urządzenia
        return -1;

    got_accel[0] = ((short)data[0] << 8) | data[1]; // używając sczytanych danych 
    got_accel[1] = ((short)data[2] << 8) | data[3];
    got_accel[2] = ((short)data[4] << 8) | data[5];

    accel_hw[0] += got_accel[0]; // utwórz nową konfigurację bias'ingu
    accel_hw[1] += got_accel[1];
    accel_hw[2] += got_accel[2];

    data[0] = (accel_hw[0] >> 8) & 0xff;
    data[1] = (accel_hw[0]) & 0xff;
    data[2] = (accel_hw[1] >> 8) & 0xff;
    data[3] = (accel_hw[1]) & 0xff;
    data[4] = (accel_hw[2] >> 8) & 0xff;
    data[5] = (accel_hw[2]) & 0xff;

    if (i2c_write(st.hw->addr, 0x06, 6, data)) // zapisz konfigurację do rejestru przechowującego bias urzadzenia 
        return -1;
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Definicja funkcji służącej do resetowania bufora FIFO.

 ******************************************************************************/
/**
 *  @brief  Reset FIFO read/write pointers.
 *  @return 0 if successful.
 */
int mpu_reset_fifo(void)
{
    unsigned char data; // utwórz 8 bitową zmienną

    if (!(st.chip_cfg.sensors)) // jeśli wszystkie czujniki są wyłączone to przerwij działanie
        return -1;

    data = 0; // wyzeruj zmienną "data"
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data)) // wyzeruj rejestr "INT_ENABLE"
        return -1;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data)) // wyzeruj rejestr "FIFO_EN"
        return -1;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data)) // wyzeruj rejestr "USER_CTRL"
        return -1;

    if (st.chip_cfg.dmp_on) { // jeśli włączony jest Digital Motion Processor (na razie to pominę bo zapewne nie będziemy go używać)
        data = BIT_FIFO_RST | BIT_DMP_RST;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
            return -1;
        delay_ms(50);
        data = BIT_DMP_EN | BIT_FIFO_EN;
        if (st.chip_cfg.sensors & INV_XYZ_COMPASS)
            data |= BIT_AUX_IF_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data))
            return -1;
        if (st.chip_cfg.int_enable)
            data = BIT_DMP_INT_EN;
        else
            data = 0;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data))
            return -1;
        data = 0;
        if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &data))
            return -1;
    } else { // jeśli DMP jest wyłączony
        data = BIT_FIFO_RST; // ustaw w zmiennej "data" bit BIT_FIFO_RST
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data)) // rozpocznij procedurę "reset FIFO buffer"
            return -1;
        if (st.chip_cfg.bypass_mode || !(st.chip_cfg.sensors & INV_XYZ_COMPASS)) // jeśli działamy w "bypass_mode" lub kompas jest wyłączony
            data = BIT_FIFO_EN; // to ustaw jedynie bit BIT_FIFO_EN => włącz FIFO buffer
        else
            data = BIT_FIFO_EN | BIT_AUX_IF_EN; // w innym wypadku ustaw dodatkowo bit BIT_AUX_IF_EN
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &data)) // zapisz konfigurację do rejestru USER_CTRL
            return -1;
        delay_ms(50); // poczekaj 50 ms na stabilizacje FIFO
        if (st.chip_cfg.int_enable) // jeśli wcześniej ustawiony było jakie kolwiek przerwanie w rejestrze "INT_ENABLE"
            data = BIT_DATA_RDY_EN; // to ustaw bit BIT_DATA_RDY_EN
        else
            data = 0; // w innym wypadku, wyzeruj zmienną "data"
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, &data)) // wpisz konfigurację do rejestru INT_ENABLE
            return -1;
        if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, &st.chip_cfg.fifo_enable)) // zapisz w rejestrze FIFO_EN obecną konfigurację urządzenia przechowywaną w strukturze "st" (state)
            return -1;
    }
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwracająca aktualne ustawienia zakresu pracy żyroskopu.

 ******************************************************************************/
/**
 *  @brief      Get the gyro full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_gyro_fsr(unsigned short *fsr)
{
    switch (st.chip_cfg.gyro_fsr) { // w zależnośi od ustawienia zapisanego w naszej strukturze konfiguracyjnej, przypisz do zmiennej "fsr" odpowiednią czułość
    case INV_FSR_250DPS:
        fsr[0] = 250;
        break;
    case INV_FSR_500DPS:
        fsr[0] = 500;
        break;
    case INV_FSR_1000DPS:
        fsr[0] = 1000;
        break;
    case INV_FSR_2000DPS:
        fsr[0] = 2000;
        break;
    default:
        fsr[0] = 0;
        break;
    }
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja ustawiająca zakres pracy żyroskopu. 

 Należy zwrócić uwagę jak sprytnie używa ona typu wyliczeniowego "gyro_fsr_e"
 przesuniętego o 3 bity w lewo. Przesunięcie wynika z tego, że to od 3 bitu
 w rejestrze GRYO_CFG ustawia się zakres pracy żyroskopu.

 Ustawiany zakres jest wartością 2 bitową (0 - 3)

 ******************************************************************************/
/**
 *  @brief      Set the gyro full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_gyro_fsr(unsigned short fsr)
{
    unsigned char data;

    if (!(st.chip_cfg.sensors)) // jeśli wszystkie podzespoły są wyłączone to przerwji działanie funkcji
        return -1;

    switch (fsr) { // wykonaj konkretną czynność w zależności od wartości parametru "fsr" (full scale range)s
    case 250:
        data = INV_FSR_250DPS << 3; 
        break;
    case 500:
        data = INV_FSR_500DPS << 3;
        break;
    case 1000:
        data = INV_FSR_1000DPS << 3;
        break;
    case 2000:
        data = INV_FSR_2000DPS << 3;
        break;
    default:
        return -1;
    }

    if (st.chip_cfg.gyro_fsr == (data >> 3)) // jeśli ustawiona konfiguracja jest już obecie wybrana to zakończ działanie funkcji
        return 0;
    if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, &data)) // Wpisz rządaną konfigurację do rejestru GYRO_CFG
        return -1;
    st.chip_cfg.gyro_fsr = data >> 3; // Zapisz ustawienia w strukturze przechowującej obecny stan urządzenia.
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwracające aktualne ustawienia zakresu pracy akceleromteru.

 To czy aktywny jest tryb "half accelerometer precision" zależy od numeru
 seryjnego urządzenia sprawdzanego w funkcji inicjalizującej.

 ******************************************************************************/
/**
 *  @brief      Get the accel full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_accel_fsr(unsigned char *fsr)
{
    switch (st.chip_cfg.accel_fsr) { // w zależności od wartości ustawionej w naszej strukturze konifiguracyjnej, wpisz prawidłową wartość do zmiennej podanej jako paramter
    case INV_FSR_2G:
        fsr[0] = 2;
        break;
    case INV_FSR_4G:
        fsr[0] = 4;
        break;
    case INV_FSR_8G:
        fsr[0] = 8;
        break;
    case INV_FSR_16G:
        fsr[0] = 16;
        break;
    default:
        return -1;
    }
    if (st.chip_cfg.accel_half) // jeśli włączony jest tryb "halp accelerometer precision" to zwróc czułość podzieloną przez 2
        fsr[0] <<= 1;
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja ustawiająca zakres pracy akcelerometru.

 Należy zwrócić uwagę jak sprytnie używa ona typu wyliczeniowego "accel_fsr_e"
 przesuniętego o 3 bity w lewo. Przesunięcie wynika z tego, że to od 3 bitu
 w rejestrze ACCEL_CFG ustawia się zakres pracy akcelerometru.

 Ustawiany zakres jest wartością 2 bitową (0 - 3)

 ******************************************************************************/
/**
 *  @brief      Set the accel full-scale range.
 *  @param[in]  fsr Desired full-scale range.
 *  @return     0 if successful.
 */
int mpu_set_accel_fsr(unsigned char fsr)
{
    unsigned char data;

    if (!(st.chip_cfg.sensors)) // jeśli wszyskite sensory są wyłączone to przerwij działanie
        return -1;

    switch (fsr) { // w zależności od wybranej czułośći wykonaj odpowiednią operacje
    case 2:
        data = INV_FSR_2G << 3;
        break;
    case 4:
        data = INV_FSR_4G << 3;
        break;
    case 8:
        data = INV_FSR_8G << 3;
        break;
    case 16:
        data = INV_FSR_16G << 3;
        break;
    default:
        return -1;
    }

    if (st.chip_cfg.accel_fsr == (data >> 3)) // sprawdź czy przypadkiem rządane ustawienie nie zostału już wcześniej wybrane
        return 0;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, &data)) // wpisz konfiguracje do odpowiedniego rejestru
        return -1;
    st.chip_cfg.accel_fsr = data >> 3; // zapisz wykonaną operacje w strukturze przechowującej obecną konfiguracje
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwracające aktualne ustawienie Digital Low Pass Filter

 ******************************************************************************/
/**
 *  @brief      Get the current DLPF setting.
 *  @param[out] lpf Current LPF setting.
 *  0 if successful.
 */
int mpu_get_lpf(unsigned short *lpf)
{
    switch (st.chip_cfg.lpf) { // w zależności od wartości zapisanej w strukturze konfiguracyjnej, zapisz prawdiłową wartość w zmiennej podanej jako parametr
    case INV_FILTER_188HZ:
        lpf[0] = 188;
        break;
    case INV_FILTER_98HZ:
        lpf[0] = 98;
        break;
    case INV_FILTER_42HZ:
        lpf[0] = 42;
        break;
    case INV_FILTER_20HZ:
        lpf[0] = 20;
        break;
    case INV_FILTER_10HZ:
        lpf[0] = 10;
        break;
    case INV_FILTER_5HZ:
        lpf[0] = 5;
        break;
    case INV_FILTER_256HZ_NOLPF2: // ten i kolejny przypadek nie są używane w MPU 6050
    case INV_FILTER_2100HZ_NOLPF:
    default:
        lpf[0] = 0;
        break;
    }
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja ustawiająca cut-off filtra dolnoprzepustowego.

 Dostępne wartości [Hz]: 188, 98, 42, 20, 10, 5

 Ponownie należy zwrócić uwagę jak sprytnie używane są tutaj typy wyliczeniowe
 zdefiniowane wcześniej.

 Ustawienia filtra znajdują się w rejestrze CONFIG i są to pierwsze 3 bity [0:2]

 Trzeba paiętać, że im niższy cut-off tym większe opóźnienie wprowowadzone przez
 użycie filtra.

 Dokładnie jest to rozpisane w dokumentacji "MPU-6000 and MPU-6050 Register Map 
 and Descriptions Revision 4.0" w opisie rejestru "CONFIG"

 Należy pamiętać, że jeśli włączymy Low Pass Filter to output rate żyroskopu
 domyślnie jest ograniczony do 1 kHz!

 Dodatkowo, można ustawić filtr 

 ******************************************************************************/
/**
 *  @brief      Set digital low pass filter.
 *  The following LPF settings are supported: 188, 98, 42, 20, 10, 5.
 *  @param[in]  lpf Desired LPF setting.
 *  @return     0 if successful.
 */
int mpu_set_lpf(unsigned short lpf)
{
    unsigned char data;

    if (!(st.chip_cfg.sensors)) // jeśli wszystkie sensory są wyłączone to przerwij operacje
        return -1;

    if (lpf >= 188) // dopasuj wpisany parametr to obsługiwanyczh cut-off'ów
        data = INV_FILTER_188HZ;
    else if (lpf >= 98)
        data = INV_FILTER_98HZ;
    else if (lpf >= 42)
        data = INV_FILTER_42HZ;
    else if (lpf >= 20)
        data = INV_FILTER_20HZ;
    else if (lpf >= 10)
        data = INV_FILTER_10HZ;
    else
        data = INV_FILTER_5HZ;

    if (st.chip_cfg.lpf == data) // sprawdź czy przypdkiem rządane ustawienia nie są już ustawione
        return 0;
    if (i2c_write(st.hw->addr, st.reg->lpf, 1, &data)) // wpisz ustawienia do odpowiedniego rejestru
        return -1;
    st.chip_cfg.lpf = data;
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwraca aktualne utawienia "sampling rate".

 ******************************************************************************/
/**
 *  @brief      Get sampling rate.
 *  @param[out] rate    Current sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_get_sample_rate(unsigned short *rate)
{
    if (st.chip_cfg.dmp_on) // jeśli włączony jest Digital Moion Processor to przerwji działanie i zwróć kod błędu
        return -1;
    else
        rate[0] = st.chip_cfg.sample_rate; // w innym wypadku, zapisz dane ze struktury konfiguracyjnej do zmiennej podanej jako argument
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja ustawiająca sampling rate dla danych pobieranych z czujników.

 Zgodnie z data sheet'em, jeśli używamy zarówno żyroskopu, jak i akcelerometru
 to sampling rate musi mieścić się między 4 Hz a 1 kHz.

 Należy pamiętać, że zgodnie z data sheet'em, sampling rate ustawia się jako:

    Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)

 Więc jedynym modyfikowalnym przez nas paramterem jest tutaj "SMPLRT_DIV" i to
 ten parametr musimy wpisać do rejestru.

 ******************************************************************************/
/**
 *  @brief      Set sampling rate.
 *  Sampling rate must be between 4Hz and 1kHz.
 *  @param[in]  rate    Desired sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_set_sample_rate(unsigned short rate)
{
    unsigned char data;

    if (!(st.chip_cfg.sensors)) // jeśli wszystkie czujniki są wyłączone to przerwji działanie
        return -1;

    if (st.chip_cfg.dmp_on) // jeśli włączony jest Digital Motion Processor to przerwij działanie
        return -1;
    else {
        if (st.chip_cfg.lp_accel_mode) { // sprawdź czy włączony jest tryp "low-power", domyślnie jest wylączony
            if (rate && (rate <= 40)) { // jeśli paramter "rate" jest różny od zera oraz mniejszy bądź równy 40 to ustaw ten "rate" w trybie "low-power"
                /* Just stay in low-power accel mode. */
                mpu_lp_accel_mode(rate);
                return 0;
            }
            /* Requested rate exceeds the allowed frequencies in LP accel mode,
             * switch back to full-power mode.
             */
            mpu_lp_accel_mode(0); // jeśli "rate" jest powyżej 40 a jesteśmy w "low-power" mode to wyjdź z trybu "low-power", aby móc ustawić rządany sampling rate
        }
        if (rate < 4) // ogranicz minimalny możliwy sampling rate do 4 Hz
            rate = 4;
        else if (rate > 1000) // ogranicz maksymalny możliwy sampling rate do 1000 Hz
            rate = 1000;

        data = 1000 / rate - 1; // przekształć parametr "rete" w SMPLRT_DIV, które będzie wpisane do rejestru
        if (i2c_write(st.hw->addr, st.reg->rate_div, 1, &data)) // wpisz odpowiednią wartość do rejestru obłusugjącego Sample Rate
            return -1;

        st.chip_cfg.sample_rate = 1000 / (1 + data); // zapisz w naszej strukturze konfiguracyjniej zadane sample rate

        /* Automatically set LPF to 1/2 sampling rate. */
        mpu_set_lpf(st.chip_cfg.sample_rate >> 1); // automatycznie ustaw cut-off'a filtra dolnoprzepustowego na połowę zadengo sampling rate
        return 0;
    }
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwracająca Sampling Rate ustawiony dla kompasu.

 W projekcie robotic arm raczej nam się to nie przyda.

 ******************************************************************************/
/**
 *  @brief      Get compass sampling rate.
 *  @param[out] rate    Current compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_get_compass_sample_rate(unsigned short *rate)
{
    rate[0] = 0; // nie używamy kompasu więc ustaw zmienną na 0
    return -1; // oraz zwróc kod błedu.
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja ustawiająca Sampling Rate dla kompasu.

 W projekcie robotic arm raczej nam się to nie przyda.

 ******************************************************************************/
/**
 *  @brief      Set compass sampling rate.
 *  The compass on the auxiliary I2C bus is read by the MPU hardware at a
 *  maximum of 100Hz. The actual rate can be set to a fraction of the gyro
 *  sampling rate.
 *
 *  \n WARNING: The new rate may be different than what was requested. Call
 *  mpu_get_compass_sample_rate to check the actual setting.
 *  @param[in]  rate    Desired compass sampling rate (Hz).
 *  @return     0 if successful.
 */
int mpu_set_compass_sample_rate(unsigned short rate)
{
    return -1; // zwróc -1 ponieważ nie używamy w projekcie kompasu
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwracająca ustawienia czułości żyroskopu.

 Funkcja ta tłumaczy Full Scale Range na LSB/dps czyli ile kroków konwertera
 analogowo-cyfrowego odpowiada jednepu stopniowi na sekundę.

 Te wartości można odczytać bezpośrednio z data sheet'a.

 ******************************************************************************/

/**
 *  @brief      Get gyro sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to dps.
 *  @return     0 if successful.
 */
int mpu_get_gyro_sens(float *sens)
{
    switch (st.chip_cfg.gyro_fsr) { // na podstawie danych zapisanych w strukturze konfiguracyjnej...
    case INV_FSR_250DPS:
        sens[0] = 131.f; // ... przetłumasz Full Scale Range na LSB/dps
        break;
    case INV_FSR_500DPS:
        sens[0] = 65.5f;
        break;
    case INV_FSR_1000DPS:
        sens[0] = 32.8f;
        break;
    case INV_FSR_2000DPS:
        sens[0] = 16.4f;
        break;
    default:
        return -1;
    }
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwracająca ustawienia czułości akcelerometra.

 Funkcja ta tłumaczy Full Scale Range na LSB/g czyli ile kroków konwertera
 analogowo-cyfrowego odpowiada jednemu g.

 Te wartości można odczytać bezpośrednio z data sheet'a.

 ******************************************************************************/
/**
 *  @brief      Get accel sensitivity scale factor.
 *  @param[out] sens    Conversion from hardware units to g's.
 *  @return     0 if successful.
 */
int mpu_get_accel_sens(unsigned short *sens)
{
    switch (st.chip_cfg.accel_fsr) { // odczytaj full scale range i...
    case INV_FSR_2G:
        sens[0] = 16384; // wylicz na jego podstawie LSB/g
        break;
    case INV_FSR_4G:
        sens[0] = 8092;
        break;
    case INV_FSR_8G:
        sens[0] = 4096;
        break;
    case INV_FSR_16G:
        sens[0] = 2048;
        break;
    default:
        return -1;
    }
    if (st.chip_cfg.accel_half) // jeśli jest włączony tryb "accel_halt" to podziel wynik przez 2.
        sens[0] >>= 1;
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwracająca ustawienia konfiguracyjne bufora FIFO.

 Jest to po prostu zwrócenie wartości ze struktury konfiguracyjnej.

 ******************************************************************************/
/**
 *  @brief      Get current FIFO configuration.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[out] sensors Mask of sensors in FIFO.
 *  @return     0 if successful.
 */
int mpu_get_fifo_config(unsigned char *sensors)
{
    sensors[0] = st.chip_cfg.fifo_enable; // zapisz w zmiennej podanej jako argument obecną konfigurację FIFO
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja ustawiająca, które czujniki mają przekazywać swoje odczyty do bufora
 First-In-Firsto-Out.

 Jeśli przekażemy funkcji parametr o wartości 0 to dane z żadnego z czujników
 nie zostaną przekierowane do FIFO.

 ******************************************************************************/
/**
 *  @brief      Select which sensors are pushed to FIFO.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  @param[in]  sensors Mask of sensors to push to FIFO.
 *  @return     0 if successful.
 */
int mpu_configure_fifo(unsigned char sensors)
{
    unsigned char prev;
    int result = 0;

    /* Compass data isn't going into the FIFO. Stop trying. */
    sensors &= ~INV_XYZ_COMPASS; // powstrzymaj użytkownika przed próba przekierowania danych z kompasu do FIFO

    if (st.chip_cfg.dmp_on) // jeśli DMP jest włączony to przerwij działanie
        return 0;
    else { // jeśli DMP jest wyłączony to:
        if (!(st.chip_cfg.sensors)) // Sprawdź czy sensory są właczone, jeśli nie to przerwij działanie
            return -1;
        prev = st.chip_cfg.fifo_enable; // Zapisz poprzednią konfigurację bufora FIFO
        st.chip_cfg.fifo_enable = sensors & st.chip_cfg.sensors; // Ustaw bity w strukturze konfiguracyjnej tylko dla sensorów, które są używane
        if (st.chip_cfg.fifo_enable != sensors) // jeśli spróbujesz przekieować wyniki wyłączonej sensora do FIFO to funkcja zwróci wynik w postaci "-1"
            /* You're not getting what you asked for. Some sensors are
             * asleep.
             */
            result = -1;
        else // jeśli wszystkie sensory, których ustawienia chcesz modyfikować, są włączone to zapisz wynik jako poprawny (czyli '0')
            result = 0;
        if (sensors || st.chip_cfg.lp_accel_mode) // jeśli jakiekolwiek czujniki działają lub działasz w trybie "low-power"
            set_int_enable(1); // to wlącz przerwania "data ready"
        else
            set_int_enable(0); // W innym wypadku, wyłącz przerwanie "data ready"
        if (sensors) { // jeśli jakiekolwiek sensory działają to
            if (mpu_reset_fifo()) { // zresetuj działanie bufora FIFO 
                st.chip_cfg.fifo_enable = prev; // w wypadku niepowodzenia resetu bufora FIFO, przywróc poprzednie ustawienia i zapisz kod błędu ("-1")
                return -1;
            }
        }
    }

    return result;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwracjąca obecny stan zużycia mocy udząrzednia.

 Zwraca:
  -> 0 => sensory wyłączone, urządzenie wstrzymane,
  -> 1 => sensory włączone, urządzenie pracuje.

 ******************************************************************************/
/**
 *  @brief      Get current power state.
 *  @param[in]  power_on    1 if turned on, 0 if suspended.
 *  @return     0 if successful.
 */
int mpu_get_power_state(unsigned char *power_on)
{
    if (st.chip_cfg.sensors) // jeśli którekolwiek sensory są włączone
        power_on[0] = 1; // to zwróc informację, że urządzenie działa
    else // jeśli wszystkie są wyłączone
        power_on[0] = 0; // to zwróc informacje, że urządzenie jest wstrzymane.
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja dzięki której możemy włączyć konkretne sensory.

 Pamiętaj, aby posługiwać się, zdefiniowanym w nagłówku "mpu6050.h", makrami
 odpowiadającymi konrketnym podezposłom:
 -> INV_X_GYRO      żyroskop tylko w osi x
 -> INV_Y_GYRO      żyroskop tylko w osi y
 -> INV_Z_GYRO      żyroskop tylko w osi z
 -> NV_XYZ_GYRO     żyroskop we wszystkich 3 osiach
 -> INV_XYZ_ACCEL   akcelerometr we wszystkich 3 osiach

 Co jest ważne w tej funkcji to fakt, iż jesli nie ustawimy któregokolwiek
 z czujników w "stanby mode" w rejestrze PWR_MGMT_2 to, domyślnie, jest on
 włączony.

 Dodatkowo, jeśli wcześniej wybraliśmy żyroskop jako źródło zegara to jeśli go
 wyłączymy w rejestrze PWR_MGMT_2, czujnik automatucznie przełączy się na
 wewnętrzny zegar 8MHz.

 ******************************************************************************/
/**
 *  @brief      Turn specific sensors on/off.
 *  @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_XYZ_COMPASS
 *  @param[in]  sensors    Mask of sensors to wake.
 *  @return     0 if successful.
 */
int mpu_set_sensors(unsigned char sensors)
{
    unsigned char data;

    if (sensors & INV_XYZ_GYRO) // jeśli poprosiliśmy o używanie żyroskopu we wszystkich 3 osiach
        data = INV_CLK_PLL; // to zainicjalizujmy naszą zmienną "data" bitem INV_CLK_PLL
    else if (sensors) // jeśli używamy jakiejkolwiek innej konfiguracji
        data = 0; // to zainicjalizujmy naszą zmienną wartością '0'
    else
        data = BIT_SLEEP; // jeśli nie wybraliśmy żadnych czujników to ustawmy czujnik w SLEEP MODE
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, &data)) { // zapiszmy zawartość zmiennej "data" w rejestrze PWR_MGMT_1
        st.chip_cfg.sensors = 0;  // w przypadku niepowodzenia zapisu do rejestru PWR_MGMT_1, zapiszmy w naszej funkcji konfiguracyjnej, iż wszystkie sensory są wyłączone
        return -1;
    }
    st.chip_cfg.clk_src = data & ~BIT_SLEEP; // zapiszmy wybraną konfigurację zegara do naszej struktury przechowującej bieżący stan czujnika

    data = 0; // wyzerujmy naszą zmienną "data"
    if (!(sensors & INV_X_GYRO))
        data |= BIT_STBY_XG; // jeśli nie używamy osi x żyroskopu to ustawmy w zmiennej "data" bit BIT_STBY_XG
    if (!(sensors & INV_Y_GYRO))
        data |= BIT_STBY_YG;  // jeśli nie używamy osi y żyroskopu to ustawmy w zmiennej "data" bit BIT_STBY_YG
    if (!(sensors & INV_Z_GYRO))
        data |= BIT_STBY_ZG; // jeśli nie używamy osi z żyroskopu to ustawmy w zmiennej "data" bit BIT_STBY_ZG
    if (!(sensors & INV_XYZ_ACCEL))
        data |= BIT_STBY_XYZA; // jeśli nie używamy akcelerometru to ustawmy w zmiennej "data" bit BIT_STBY_XYZA
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_2, 1, &data)) { // wpiszmy wartość zmiennej "data" do rejstru PWR_MGMT_2
        st.chip_cfg.sensors = 0;
        return -1;
    }

    if (sensors && (sensors != INV_XYZ_ACCEL)) // jeśli włączyliśmy jakikolwiek sensor i nie jest to akcelerometr to:
        /* Latched interrupts only used in LP accel mode. */
        mpu_set_int_latched(0); // musimy wyłączyć funkcje "latched interrupts", która sprawia, że pin INT utrzymuje swój stan dopóki odpowiednia flaga przerwania nie zostanie wyczyszczona

    st.chip_cfg.sensors = sensors; // zapisz zadane ustawienia w naszej strukturze konfiguracyjnej
    st.chip_cfg.lp_accel_mode = 0; // zapisz w naszej strukturze konfiguracyjnej, że nie używamy trybu "low-power"
    delay_ms(50); // poczekaj 50 ms na ustabilizowanie się sensorów
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwracjąca obecny stan rejestru przerwań.

 Ważne jest to, aby zauważyć, że funkcja zwraca dwa pełne rejestry:
 -> statusy przerwań dla DMP (Digital Motino Processor),
 -> statusy przerwań dla reszty czujnika (bez DMP).

 ******************************************************************************/
/**
 *  @brief      Read the MPU interrupt status registers.
 *  @param[out] status  Mask of interrupt bits.
 *  @return     0 if successful.
 */
int mpu_get_int_status(short *status)
{
    unsigned char tmp[2]; // utwórz 2 bajtową zmienną
    if (!st.chip_cfg.sensors) // jeśli wszystkie sensory są wyłączone to...
        return -1; // ...przerwij działanie i zwróc kod błędu
    if (i2c_read(st.hw->addr, st.reg->dmp_int_status, 2, tmp)) // Odczytaj zwartość statusu przerwań zarówno dla DMP, jak i dla reszty czujnika.
        return -1;
    status[0] = (tmp[0] << 8) | tmp[1];
    return 0;
}


/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwracjąca jeden pakiet z bufora FIFO.

 Należy pamiętać o tym, że wszystko dziala tutaj w trybie First-in-First-Out 
 queue czyli jeśli chcemy odczytać zawartość 12 rejestrów to musimy 12 razy
 odczytać dane z FIFO i po każdym odczycie, najstarsze dane są odrzucane i podczas
 kolejnego odczytu, mamy dostęp do danych z kolejnego rejestru.

 Z data sheet'a wynika, że dane z czujników są wpisywane do FIFO w kolejności
 odpowiadającej adresowi ich rejestrów (od najmniejszego do największego).

 ******************************************************************************/
/**
 *  @brief      Get one packet from the FIFO.
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
int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
        unsigned char *sensors, unsigned char *more)
{
    /* Assumes maximum packet size is gyro (6) + accel (6). */
    unsigned char data[MAX_PACKET_LENGTH];
    unsigned char packet_size = 0; // zainicjalizuj rozmiar pakietu na 0
    unsigned short fifo_count, index = 0;

    if (st.chip_cfg.dmp_on) // jeśli włączony jest DMP to przerwij działanie
        return -1;

    sensors[0] = 0; // ustaw zmienną "sensors", podaną jako paramter na '0'
    if (!st.chip_cfg.sensors) // jeśli żadne sensory nie są włączone to...
        return -1;  // ...zwróć błąd
    if (!st.chip_cfg.fifo_enable) // jeśli FIFO jest wyłączone to...
        return -1; // ...zwróć błąd

    if (st.chip_cfg.fifo_enable & INV_X_GYRO) // jeśli używamy osi X żyroskopu to...
        packet_size += 2; //...powiększ rozmiar pakietu o 2 bajty
    if (st.chip_cfg.fifo_enable & INV_Y_GYRO) // jeśli używamy osi y żyroskopu to...
        packet_size += 2; //...powiększ rozmiar pakietu o 2 bajty
    if (st.chip_cfg.fifo_enable & INV_Z_GYRO) // jeśli używamy osi z żyroskopu to...
        packet_size += 2; //...powiększ rozmiar pakietu o 2 bajty
    if (st.chip_cfg.fifo_enable & INV_XYZ_ACCEL) // jeśli używamy wszystkich osi akcelerometra to...
        packet_size += 6; //...powiększ rozmiar pakietu o 6 bajtów

    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data)) // odczytaj ile próbek danych jest obecnie dostępnych w buforze FIFO
        return -1;
    fifo_count = (data[0] << 8) | data[1]; // zapisz w zminnej "fifo_count" ile jest dostępnych do sczytania danych
    if (fifo_count < packet_size) // jeśli ilośc danych do sczytania jest mniejsza niż potrzebny rozmiar pakietu to zakończ dzianie bez błęd -> wywołaj tę funkcję póxniej, żeby sczytać dane
        return 0;
//    log_i("FIFO count: %hd\n", fifo_count);
    if (fifo_count > (st.hw->max_fifo >> 1)) { // jeśli FIFO jest w połowie pełne to...
        /* FIFO is 50% full, better check overflow bit. */
        if (i2c_read(st.hw->addr, st.reg->int_status, 1, data)) // ...prezczytaj dane z rejestru INT_STATUS
            return -1;
        if (data[0] & BIT_FIFO_OVERFLOW) { // jeśi nastąpiło przepełnienie FIFO to go zresetuj
            mpu_reset_fifo();
            return -2;
        }
    }
    get_ms((unsigned long*)timestamp); // sczytaj obecny znacznik czasu

    if (i2c_read(st.hw->addr, st.reg->fifo_r_w, packet_size, data)) // oczytaj rejestr FIFO_R_W tyle razy, ile bajtów jest w potrzebnym dla nas pakiecie.
        return -1;
    more[0] = fifo_count / packet_size - 1; // zapisz do zmiennej podanej jako parametr funkcji, ile jeszcze zostało pakietów do odczytania
    sensors[0] = 0; // wyzeruj zmienną "sensors"

    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_XYZ_ACCEL) { // odczytaj dane z akcelerometra
        accel[0] = (data[index+0] << 8) | data[index+1];
        accel[1] = (data[index+2] << 8) | data[index+3];
        accel[2] = (data[index+4] << 8) | data[index+5];
        sensors[0] |= INV_XYZ_ACCEL; // zapisz w zmiennej "sensors", że odczytałeś dane dla akcelerometra
        index += 6;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_X_GYRO) { // odczytaj dane z osi x żyroskopu
        gyro[0] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_X_GYRO; // zapisz w zmiennej "sensors", że odczytałeś dane z osi x żyroskopu
        index += 2;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Y_GYRO) { // odczytaj dane z osi y żyroskopu
        gyro[1] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_Y_GYRO; // zapisz w zmiennej "sensors", że odczytałeś dane z osi x żyroskopu
        index += 2;
    }
    if ((index != packet_size) && st.chip_cfg.fifo_enable & INV_Z_GYRO) { // odczytaj dane z osi y żyroskopu
        gyro[2] = (data[index+0] << 8) | data[index+1];
        sensors[0] |= INV_Z_GYRO; // zapisz w zmiennej "sensors", że odczytałeś dane z osi x żyroskopu
        index += 2;
    }

    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja zwracjąca jeden niestandardowy pakiet z bufora FIFO.

 Niestandardowy oznacza tutaj, że jego długość i wielkośc nie odpowiadają
 zdefiniowanym makrom lub włączonym sensorom.

 Co ciekawe, funkcja działa tylko wtedy gdy włączony jest Digial Motion Processor.

 ******************************************************************************/
/**
 *  @brief      Get one unparsed packet from the FIFO.
 *  This function should be used if the packet is to be parsed elsewhere.
 *  @param[in]  length  Length of one FIFO packet.
 *  @param[in]  data    FIFO packet.
 *  @param[in]  more    Number of remaining packets.
 */
int mpu_read_fifo_stream(unsigned short length, unsigned char *data,
    unsigned char *more)
{
    unsigned char tmp[2]; // utwórz 1 bajtową zmienną "tmp"
    unsigned short fifo_count; // utwórz 2 bajtów (16-bitową) zniemmną "fifo_count"
    if (!st.chip_cfg.dmp_on) // jeśli DMP jest wyłączony to...
        return -1; //...przerwij działanie
    if (!st.chip_cfg.sensors) // jeśli żaden sensor nie jest włączony to...
        return -1; //...przerwij działanie

    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, tmp)) // odczytaj ile danych jest dostępnych do dczytu z FIFO
        return -1;
    fifo_count = (tmp[0] << 8) | tmp[1]; // zapisz w zmiennej "fifo_count" ile zmiennych dostępnych jest do odczytu z FIFO
    if (fifo_count < length) { // jeśli liczba danych do odczytu jest mniejsza niż rzadana długość do oczytu to...
        more[0] = 0; //...poinforumuj, że nie ma nic więcej do oczytu oraz...
        return -1; //...zwróc kod błędu
    }
    if (fifo_count > (st.hw->max_fifo >> 1)) { // jeśli FIFO jest już w połowie zapełniony to...
        /* FIFO is 50% full, better check overflow bit. */
        if (i2c_read(st.hw->addr, st.reg->int_status, 1, tmp)) //...sprawdź czy FIFO się nie przepełnił...
            return -1;
        if (tmp[0] & BIT_FIFO_OVERFLOW) { //...jeśli tak to...
            mpu_reset_fifo(); //...zresetuj FIFO
            return -2;
        }
    }

    if (i2c_read(st.hw->addr, st.reg->fifo_r_w, length, data)) // odczytaj rządanę liczbę danych z FIFO
        return -1;
    more[0] = fifo_count / length - 1; // zapisz ile jeszcze zostało danych w FIFO do oczytania
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja ustawiająca urządzenie w tryb "bypass".

 Tryb "bypass" pozwala nam bezpośrednio konfigurować zewnętrzny czujnik podłączony
 do MPU-6050.

 Ze względu na to, że nie używamy żadnego zewnętrznego czujnika to ta funkcja
 nigdy nie będzie przez nas używana.

 Pole "bypass_mode" opisane jest przez producenta w następujący sposób:

    "1 if devices on auxiliary I2C bus appear on the primary."

 Dość ciekawe jest to, że funkcja musi też modyfikować rejestr INT_PIN_CFG,
 ale dzieje się tak ze względu na to, że to tam właśnie znajduje się bit,
 który jest najważniejszy w konfiguracji trybu "bypass". Jeśli bit ten (I2C_BYPASS_EN)
 jest ustawiony na '0' to żaden inny rejestr nie będzie w stanie włączyć
 trybu "bypass".

 ******************************************************************************/
/**
 *  @brief      Set device to bypass mode.
 *  @param[in]  bypass_on   1 to enable bypass mode.
 *  @return     0 if successful.
 */
int mpu_set_bypass(unsigned char bypass_on)
{
    unsigned char tmp;

    if (st.chip_cfg.bypass_mode == bypass_on) // sprawdź czy przypadkiem urządzenie nie jest już skonfigurowane w tryb "bypass"
        return 0;

    if (bypass_on) { // co robić jeśli funkcja ma włączyć tryb "bypass"
        if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        tmp &= ~BIT_AUX_IF_EN;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp))
            return -1;
        delay_ms(3);
        tmp = BIT_BYPASS_EN;
        if (st.chip_cfg.active_low_int)
            tmp |= BIT_ACTL;
        if (st.chip_cfg.latched_int)
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR;
        if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp))
            return -1;
    } else { // co robić jeśli chcemy wyłączyć tryb "bypass" - to co nas interesuje
        /* Enable I2C master mode if compass is being used. */
        if (i2c_read(st.hw->addr, st.reg->user_ctrl, 1, &tmp)) // sczytaj zawartość rejestru "USER_CTRL"
            return -1;
        if (st.chip_cfg.sensors & INV_XYZ_COMPASS) // jeśli właczony jest kompas
            tmp |= BIT_AUX_IF_EN; // ustaw bit, który spawi, że MPU-6050 może komunikować się z kompasem
        else
            tmp &= ~BIT_AUX_IF_EN; // jeśli nie używamy kopmasu to wyczyść bit BIT_AUX_IF_EN
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, &tmp)) // wpisz wybrane ustawienia do odpowiedniego rejestru
            return -1;
        delay_ms(3); // Odczekaj 3 ms
        if (st.chip_cfg.active_low_int) // jeśli używamy przerwań w trybie active-when-LOW
            tmp = BIT_ACTL; // to ustaw w zmiennej "tmp" tylko bit BIT_ACTL czyli tmp = 0b1000 0000
        else
            tmp = 0; // w innym wypadku wyzeruj zawartość zmiennej "tmp"
        if (st.chip_cfg.latched_int) // jeśli używamy funkcji "latched interrupts" to:
            tmp |= BIT_LATCH_EN | BIT_ANY_RD_CLR; // ustaw w zmiennej "tpm" bity BIT_LATCH_EN oraz BIT_ANY_RD_CLR
        if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp)) // wpisz zawartość zmiennej "tmp" do rejestru INT_PIN_CFG
            return -1;
    }
    st.chip_cfg.bypass_mode = bypass_on; // zapisz w naszej strukturze konfiguracyjnej zadane ustawienia
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja ustawiająca jaki locziny poziom odpowiada przerwaniu.

 Dostępne opcje:
 -> Active-when-Low,
 -> Active-when-High.

 Nie do końca rozumiem czemu ma służyć ta funkcja skoro, w gruncie rzeczy,
 nie zapisuje nic do rejestrów czujnika.

 Najprawdopodniej, wartość przypisane w tej funkcji używana jest potem 
 przy inicjalizacji urządzenia.

 ******************************************************************************/
/**
 *  @brief      Set interrupt level.
 *  @param[in]  active_low  1 for active low, 0 for active high.
 *  @return     0 if successful.
 */
int mpu_set_int_level(unsigned char active_low)
{
    st.chip_cfg.active_low_int = active_low; // wpisz zadaną konfigurację do struktury konfiguracyjnej
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 

 Funkcja włączająca tryb "latched interrupts" dla pinu INT.

 Gdy ten tryb jest włączony to pin "INT" utrzymuje status przerwania tak długo
 jak długo ustawiona jest flaga któregokowliek przerwania w odpowiednim rejestrze.

 Dopiero po wyczeszczeniu tej flagi, pin "INT" wraca to stanu jałowego.

 Może dziwić, że pojawiają sie tu linijki dotyczące bypass mode czy ustawienia
 związane active-when-low. Dzieje się tak dlatego, że funkcja musi to zweryfikować
 aby zachować te ustawienia takie jak były przed jej wywołaniem.

 Paramtr: enable
   -> 1 => włącz latched interrupts,
   -> 0 => wyłącz latched interrupts.

 ******************************************************************************/
/**
 *  @brief      Enable latched interrupts.
 *  Any MPU register will clear the interrupt.
 *  @param[in]  enable  1 to enable, 0 to disable.
 *  @return     0 if successful.
 */
int mpu_set_int_latched(unsigned char enable)
{
    unsigned char tmp;
    if (st.chip_cfg.latched_int == enable) // sprawdź czy przypadkiem zadane ustawienie nie jest już skonfigurowane
        return 0;

    if (enable) // jeśli chcemy włączyć ten tryb to:
        tmp = BIT_LATCH_EN | BIT_ANY_RD_CLR; // ustaw w zmiennej "tmp" bity BIT_LATCH_EN oraz BIT_ANY_RD_CLR
    else
        tmp = 0; // w innym wypadku wyzeruj zmienną "tmp"
    if (st.chip_cfg.bypass_mode) // jeśli aktywny jest tryb "bypass"
        tmp |= BIT_BYPASS_EN; // to ustaw odpowieni bit
    if (st.chip_cfg.active_low_int) // jeśli aktywny jest tryb "active when low" 
        tmp |= BIT_ACTL; // to ustaw odpowiedni bit
    if (i2c_write(st.hw->addr, st.reg->int_pin_cfg, 1, &tmp)) // zapisz zmienną "tmp" w rejestrze INT_PIN_CFG
        return -1;
    st.chip_cfg.latched_int = enable; // zapisz zmianę do naszej struktury konfiguracyjnej
    return 0;
}

/* Tutaj rozpoczyna się blok funkcji zdefiniowany jedynia dla MPU-6050 */
#ifdef MPU6050
/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 
 
 Funckja służąca do odczytania wartości "Factory Trim" zapisanych na płyce
 prez producenta.

 ******************************************************************************/
static int get_accel_prod_shift(float *st_shift)
{
    unsigned char tmp[4], shift_code[3], ii;

    if (i2c_read(st.hw->addr, 0x0D, 4, tmp)) // odczytaj zawartość 4 kolejnych rejestrów zaczynając od rejestru o adresie 0x0D (Self-test registers)
        return 0x07;

    shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4); // szczytaj wyniki testu dla osi x akcelerometra
    shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2); // sczytaj wyniki testu dla osi y akcelerometra
    shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03); // sczytaj wyniki testu dla osi z akcelerometra
    for (ii = 0; ii < 3; ii++) {
        if (!shift_code[ii]) { 
            st_shift[ii] = 0.f; 
            continue; // ropocznij kolejną iteracje pętli
        }
        /* Equivalent to..
         * st_shift[ii] = 0.34f * powf(0.92f/0.34f, (shift_code[ii]-1) / 30.f)
         */
        st_shift[ii] = 0.34f;
        while (--shift_code[ii])
            st_shift[ii] *= 1.034f;
    }
    return 0;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 
 
 Funckja służąca do analizy danych z funkcji "self-test" dla ackelerometra.

 bias_regular = accelerometer output with Self-test Enabled
 biar_st      = accelerometer output with Self-test Disabled

 Self Test Response = biar_regular - bias_st

 ******************************************************************************/
static int accel_self_test(long *bias_regular, long *bias_st)
{
    int jj, result = 0; // utwórz dwie 32-bitowe zmienne
    float st_shift[3], st_shift_cust, st_shift_var; 

    get_accel_prod_shift(st_shift); // sczytaj wartości "Factory Trim" dla akcelerometra
    for(jj = 0; jj < 3; jj++) {
        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f; // Self Test Response = accelerometer output with Self-test Enabled - accelerometer output with Self-test Disabled
        if (st_shift[jj]) { // jeśli Factory trim jest większy od zera to...
            st_shift_var = st_shift_cust / st_shift[jj] - 1.f; // ...wylicz odchylenie Self Test Response od Factory Trim
            if (fabs(st_shift_var) > test.max_accel_var) // jeśli odchylenie to przekroczy maksymalną dopuszczalną wartość to...
                result |= 1 << jj; // zapisz kod w błędu w miejscu odpowiadającym testowanej osi
        } else if ((st_shift_cust < test.min_g) ||
            (st_shift_cust > test.max_g)) // jeśli Self Test reponse wychodzi poza dopuszczalny zakres to...
            result |= 1 << jj; //...ustaw kod błędu w odpowiedniej osi
    }

    return result;
}

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 
 
 Funckja służąca do analizy danych z funkcji "self-test" dla żyroskopu.

 bias_regular = gyro output with Self-test Enabled
 bias_st      = gyro output with Self-test Disabled
 
 Self Test Response = biar_regular - bias_st

 tmp[0] = XG_TEST
 tmp[1] = YG_TEST
 tmp[2] = ZG_TEST

 ******************************************************************************/
static int gyro_self_test(long *bias_regular, long *bias_st)
{
    int jj, result = 0;
    unsigned char tmp[3];
    float st_shift, st_shift_cust, st_shift_var;

    if (i2c_read(st.hw->addr, 0x0D, 3, tmp)) // odczytaj 3 rejestry zaczynając od rejestru 0x0D (Self-test)
        return 0x07;

    tmp[0] &= 0x1F; // oczytaj tylko 5 najniższych bitów = XG_TEST
    tmp[1] &= 0x1F; // oczytaj tylko 5 najniższych bitów = YG_TEST
    tmp[2] &= 0x1F; // oczytaj tylko 5 najniższych bitów = ZG_TEST

    for (jj = 0; jj < 3; jj++) {
        st_shift_cust = labs(bias_regular[jj] - bias_st[jj]) / 65536.f; // wylicz Self Test Response na podstawie gyro output with Self-test Enabled i gyro output with Self-test Disabled
        if (tmp[jj]) { // jeśli XG_TEST, YG_TEST lub ZG_TEST jest różny od zera to...
            st_shift = 3275.f / test.gyro_sens; 
            while (--tmp[jj])
                st_shift *= 1.046f; //...wylicz factory trim oraz...
            st_shift_var = st_shift_cust / st_shift - 1.f; // ...wylicz odchylenie Self Test Response od Factory Trim
            if (fabs(st_shift_var) > test.max_gyro_var) // jeśli wynik wychodzi poza zakres to...
                result |= 1 << jj; //... ustaw kod błędu dla konkretnej osi
        } else if ((st_shift_cust < test.min_dps) || 
            (st_shift_cust > test.max_dps)) // jeśli Self Test Response wychodzi poza zakres to...
            result |= 1 << jj; //...ustaw błąd w odpowiednim rejestrze
    }
    return result;
}

#endif 
/* koniec makra MPU6050 */

/******************************************************************************

                            ROBOTIC ARM DESIGN LAB 
 
 Funckja służąca do wyliczania wyników wyjściowych sensorów gdy funkcja
 Self-test jest włączona/wyłączona.

 ******************************************************************************/
static int get_st_biases(long *gyro, long *accel, unsigned char hw_test)
{
    unsigned char data[MAX_PACKET_LENGTH];
    unsigned char packet_count, ii;
    unsigned short fifo_count;

    data[0] = 0x01;
    data[1] = 0;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 2, data))
        return -1;
    delay_ms(200);
    data[0] = 0;
    if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->i2c_mst, 1, data))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;
    data[0] = BIT_FIFO_RST | BIT_DMP_RST;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;
    delay_ms(15);
    data[0] = st.test->reg_lpf;
    if (i2c_write(st.hw->addr, st.reg->lpf, 1, data))
        return -1;
    data[0] = st.test->reg_rate_div;
    if (i2c_write(st.hw->addr, st.reg->rate_div, 1, data))
        return -1;
    if (hw_test)
        data[0] = st.test->reg_gyro_fsr | 0xE0;
    else
        data[0] = st.test->reg_gyro_fsr;
    if (i2c_write(st.hw->addr, st.reg->gyro_cfg, 1, data))
        return -1;
    
    if (hw_test)
        data[0] = st.test->reg_accel_fsr | 0xE0;
    else
        data[0] = test.reg_accel_fsr;
    if (i2c_write(st.hw->addr, st.reg->accel_cfg, 1, data))
        return -1;
    if (hw_test)
        delay_ms(200);

    /* Fill FIFO for test.wait_ms milliseconds. */
    data[0] = BIT_FIFO_EN;
    if (i2c_write(st.hw->addr, st.reg->user_ctrl, 1, data))
        return -1;

    data[0] = INV_XYZ_GYRO | INV_XYZ_ACCEL;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;
    delay_ms(test.wait_ms);
    data[0] = 0;
    if (i2c_write(st.hw->addr, st.reg->fifo_en, 1, data))
        return -1;

    if (i2c_read(st.hw->addr, st.reg->fifo_count_h, 2, data))
        return -1;

    fifo_count = (data[0] << 8) | data[1];
    packet_count = fifo_count / MAX_PACKET_LENGTH;
    gyro[0] = gyro[1] = gyro[2] = 0;
    accel[0] = accel[1] = accel[2] = 0;

    for (ii = 0; ii < packet_count; ii++) {
        short accel_cur[3], gyro_cur[3];
        if (i2c_read(st.hw->addr, st.reg->fifo_r_w, MAX_PACKET_LENGTH, data))
            return -1;
        accel_cur[0] = ((short)data[0] << 8) | data[1];
        accel_cur[1] = ((short)data[2] << 8) | data[3];
        accel_cur[2] = ((short)data[4] << 8) | data[5];
        accel[0] += (long)accel_cur[0];
        accel[1] += (long)accel_cur[1];
        accel[2] += (long)accel_cur[2];
        gyro_cur[0] = (((short)data[6] << 8) | data[7]);
        gyro_cur[1] = (((short)data[8] << 8) | data[9]);
        gyro_cur[2] = (((short)data[10] << 8) | data[11]);
        gyro[0] += (long)gyro_cur[0];
        gyro[1] += (long)gyro_cur[1];
        gyro[2] += (long)gyro_cur[2];
    }
#ifdef EMPL_NO_64BIT
    gyro[0] = (long)(((float)gyro[0]*65536.f) / test.gyro_sens / packet_count);
    gyro[1] = (long)(((float)gyro[1]*65536.f) / test.gyro_sens / packet_count);
    gyro[2] = (long)(((float)gyro[2]*65536.f) / test.gyro_sens / packet_count);
    if (has_accel) {
        accel[0] = (long)(((float)accel[0]*65536.f) / test.accel_sens /
            packet_count);
        accel[1] = (long)(((float)accel[1]*65536.f) / test.accel_sens /
            packet_count);
        accel[2] = (long)(((float)accel[2]*65536.f) / test.accel_sens /
            packet_count);
        /* Don't remove gravity! */
        accel[2] -= 65536L;
    }
#else
    gyro[0] = (long)(((long long)gyro[0]<<16) / test.gyro_sens / packet_count);
    gyro[1] = (long)(((long long)gyro[1]<<16) / test.gyro_sens / packet_count);
    gyro[2] = (long)(((long long)gyro[2]<<16) / test.gyro_sens / packet_count);
    accel[0] = (long)(((long long)accel[0]<<16) / test.accel_sens /
        packet_count);
    accel[1] = (long)(((long long)accel[1]<<16) / test.accel_sens /
        packet_count);
    accel[2] = (long)(((long long)accel[2]<<16) / test.accel_sens /
        packet_count);
    /* Don't remove gravity! */
    if (accel[2] > 0L)
        accel[2] -= 65536L;
    else
        accel[2] += 65536L;
#endif

    return 0;
}

/**
 *  @brief      Trigger gyro/accel/compass self-test.
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
int mpu_run_self_test(long *gyro, long *accel)
{
#ifdef MPU6050
    const unsigned char tries = 2;
    long gyro_st[3], accel_st[3];
    unsigned char accel_result, gyro_result;
#ifdef AK89xx_SECONDARY
    unsigned char compass_result;
#endif
    int ii;
#endif
    int result;
    unsigned char accel_fsr, fifo_sensors, sensors_on;
    unsigned short gyro_fsr, sample_rate, lpf;
    unsigned char dmp_was_on;

    if (st.chip_cfg.dmp_on) {
        mpu_set_dmp_state(0);
        dmp_was_on = 1;
    } else
        dmp_was_on = 0;

    /* Get initial settings. */
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    mpu_get_lpf(&lpf);
    mpu_get_sample_rate(&sample_rate);
    sensors_on = st.chip_cfg.sensors;
    mpu_get_fifo_config(&fifo_sensors);

    /* For older chips, the self-test will be different. */
#if defined MPU6050
    for (ii = 0; ii < tries; ii++)
        if (!get_st_biases(gyro, accel, 0))
            break;
    if (ii == tries) {
        /* If we reach this point, we most likely encountered an I2C error.
         * We'll just report an error for all three sensors.
         */
        result = 0;
        goto restore;
    }
    for (ii = 0; ii < tries; ii++)
        if (!get_st_biases(gyro_st, accel_st, 1))
            break;
    if (ii == tries) {
        /* Again, probably an I2C error. */
        result = 0;
        goto restore;
    }
    accel_result = accel_self_test(accel, accel_st);
    gyro_result = gyro_self_test(gyro, gyro_st);    

    result = 0;
    if (!gyro_result)
        result |= 0x01;
    if (!accel_result)
        result |= 0x02;

#ifdef AK89xx_SECONDARY
    compass_result = compass_self_test();
    if (!compass_result)
        result |= 0x04;
#endif
restore:
#elif defined MPU6500
    /* For now, this function will return a "pass" result for all three sensors
     * for compatibility with current test applications.
     */
    get_st_biases(gyro, accel, 0);
    result = 0x7;
#endif
    /* Set to invalid values to ensure no I2C writes are skipped. */
    st.chip_cfg.gyro_fsr = 0xFF;
    st.chip_cfg.accel_fsr = 0xFF;
    st.chip_cfg.lpf = 0xFF;
    st.chip_cfg.sample_rate = 0xFFFF;
    st.chip_cfg.sensors = 0xFF;
    st.chip_cfg.fifo_enable = 0xFF;
    st.chip_cfg.clk_src = INV_CLK_PLL;
    mpu_set_gyro_fsr(gyro_fsr);
    mpu_set_accel_fsr(accel_fsr);
    mpu_set_lpf(lpf);
    mpu_set_sample_rate(sample_rate);
    mpu_set_sensors(sensors_on);
    mpu_configure_fifo(fifo_sensors);

    if (dmp_was_on)
        mpu_set_dmp_state(1);

    return result;
}

/**
 *  @brief      Write to the DMP memory.
 *  This function prevents I2C writes past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to write.
 *  @param[in]  data        Bytes to write to memory.
 *  @return     0 if successful.
 */
int mpu_write_mem(unsigned short mem_addr, unsigned short length,
        unsigned char *data)
{
    unsigned char tmp[2];

    if (!data)
        return -1;
    if (!st.chip_cfg.sensors)
        return -1;

    tmp[0] = (unsigned char)(mem_addr >> 8);
    tmp[1] = (unsigned char)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > st.hw->bank_size)
        return -1;

    if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
        return -1;
    if (i2c_write(st.hw->addr, st.reg->mem_r_w, length, data))
        return -1;
    return 0;
}

/**
 *  @brief      Read from the DMP memory.
 *  This function prevents I2C reads past the bank boundaries. The DMP memory
 *  is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Bytes read from memory.
 *  @return     0 if successful.
 */
int mpu_read_mem(unsigned short mem_addr, unsigned short length,
        unsigned char *data)
{
    unsigned char tmp[2];

    if (!data)
        return -1;
    if (!st.chip_cfg.sensors)
        return -1;

    tmp[0] = (unsigned char)(mem_addr >> 8);
    tmp[1] = (unsigned char)(mem_addr & 0xFF);

    /* Check bank boundaries. */
    if (tmp[1] + length > st.hw->bank_size)
        return -1;

    if (i2c_write(st.hw->addr, st.reg->bank_sel, 2, tmp))
        return -1;
    if (i2c_read(st.hw->addr, st.reg->mem_r_w, length, data))
        return -1;
    return 0;
}

/**
 *  @brief      Load and verify DMP image.
 *  @param[in]  length      Length of DMP image.
 *  @param[in]  firmware    DMP code.
 *  @param[in]  start_addr  Starting address of DMP code memory.
 *  @param[in]  sample_rate Fixed sampling rate used when DMP is enabled.
 *  @return     0 if successful.
 */
int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate)
{
    unsigned short ii;
    unsigned short this_write;
    /* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
#define LOAD_CHUNK  (16)
    unsigned char cur[LOAD_CHUNK], tmp[2];

    if (st.chip_cfg.dmp_loaded)
        /* DMP should only be loaded once. */
        return -1;

    if (!firmware)
        return -1;
    for (ii = 0; ii < length; ii += this_write) {
        this_write = min(LOAD_CHUNK, length - ii);
        if (mpu_write_mem(ii, this_write, (unsigned char*)&firmware[ii]))
            return -1;
        if (mpu_read_mem(ii, this_write, cur))
            return -1;
        if (memcmp(firmware+ii, cur, this_write))
            return -2;
    }

    /* Set program start address. */
    tmp[0] = start_addr >> 8;
    tmp[1] = start_addr & 0xFF;
    if (i2c_write(st.hw->addr, st.reg->prgm_start_h, 2, tmp))
        return -1;

    st.chip_cfg.dmp_loaded = 1;
    st.chip_cfg.dmp_sample_rate = sample_rate;
    return 0;
}

/**
 *  @brief      Enable/disable DMP support.
 *  @param[in]  enable  1 to turn on the DMP.
 *  @return     0 if successful.
 */
int mpu_set_dmp_state(unsigned char enable)
{
    unsigned char tmp;
    if (st.chip_cfg.dmp_on == enable)
        return 0;

    if (enable) {
        if (!st.chip_cfg.dmp_loaded)
            return -1;
        /* Disable data ready interrupt. */
        set_int_enable(0);
        /* Disable bypass mode. */
        mpu_set_bypass(0);
        /* Keep constant sample rate, FIFO rate controlled by DMP. */
        mpu_set_sample_rate(st.chip_cfg.dmp_sample_rate);
        /* Remove FIFO elements. */
        tmp = 0;
        i2c_write(st.hw->addr, 0x23, 1, &tmp);
        st.chip_cfg.dmp_on = 1;
        /* Enable DMP interrupt. */
        set_int_enable(1);
        mpu_reset_fifo();
    } else {
        /* Disable DMP interrupt. */
        set_int_enable(0);
        /* Restore FIFO settings. */
        tmp = st.chip_cfg.fifo_enable;
        i2c_write(st.hw->addr, 0x23, 1, &tmp);
        st.chip_cfg.dmp_on = 0;
        mpu_reset_fifo();
    }
    return 0;
}

/**
 *  @brief      Get DMP state.
 *  @param[out] enabled 1 if enabled.
 *  @return     0 if successful.
 */
int mpu_get_dmp_state(unsigned char *enabled)
{
    enabled[0] = st.chip_cfg.dmp_on;
    return 0;
}

/**
 *  @brief      Read raw compass data.
 *  @param[out] data        Raw data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds. Null if not needed.
 *  @return     0 if successful.
 */
int mpu_get_compass_reg(short *data, unsigned long *timestamp)
{
#ifdef AK89xx_SECONDARY
    unsigned char tmp[9];

    if (!(st.chip_cfg.sensors & INV_XYZ_COMPASS))
        return -1;

#ifdef AK89xx_BYPASS
    if (i2c_read(st.chip_cfg.compass_addr, AKM_REG_ST1, 8, tmp))
        return -1;
    tmp[8] = AKM_SINGLE_MEASUREMENT;
    if (i2c_write(st.chip_cfg.compass_addr, AKM_REG_CNTL, 1, tmp+8))
        return -1;
#else
    if (i2c_read(st.hw->addr, st.reg->raw_compass, 8, tmp))
        return -1;
#endif

#if defined AK8975_SECONDARY
    /* AK8975 doesn't have the overrun error bit. */
    if (!(tmp[0] & AKM_DATA_READY))
        return -2;
    if ((tmp[7] & AKM_OVERFLOW) || (tmp[7] & AKM_DATA_ERROR))
        return -3;
#elif defined AK8963_SECONDARY
    /* AK8963 doesn't have the data read error bit. */
    if (!(tmp[0] & AKM_DATA_READY) || (tmp[0] & AKM_DATA_OVERRUN))
        return -2;
    if (tmp[7] & AKM_OVERFLOW)
        return -3;
#endif
    data[0] = (tmp[2] << 8) | tmp[1];
    data[1] = (tmp[4] << 8) | tmp[3];
    data[2] = (tmp[6] << 8) | tmp[5];

    data[0] = ((long)data[0] * st.chip_cfg.mag_sens_adj[0]) >> 8;
    data[1] = ((long)data[1] * st.chip_cfg.mag_sens_adj[1]) >> 8;
    data[2] = ((long)data[2] * st.chip_cfg.mag_sens_adj[2]) >> 8;

    if (timestamp)
        get_ms(timestamp);
    return 0;
#else
    return -1;
#endif
}

/**
 *  @brief      Get the compass full-scale range.
 *  @param[out] fsr Current full-scale range.
 *  @return     0 if successful.
 */
int mpu_get_compass_fsr(unsigned short *fsr)
{
#ifdef AK89xx_SECONDARY
    fsr[0] = st.hw->compass_fsr;
    return 0;
#else
    return -1;
#endif
}

/**
 *  @brief      Enters LP accel motion interrupt mode.
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
int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time,
    unsigned char lpa_freq)
{
    // unsigned char data[3]; // variable used only for MPU6500

    if (lpa_freq) {
        // unsigned char thresh_hw; // variable used only for MPU6500 board

        if (!time) {
            /* Minimum duration must be 1ms. */
            time = 1;
        }

        if (!st.chip_cfg.int_motion_only) {
            /* Store current settings for later. */
            if (st.chip_cfg.dmp_on) {
                mpu_set_dmp_state(0);
                st.chip_cfg.cache.dmp_on = 1;
            } else
                st.chip_cfg.cache.dmp_on = 0;
            mpu_get_gyro_fsr(&st.chip_cfg.cache.gyro_fsr);
            mpu_get_accel_fsr(&st.chip_cfg.cache.accel_fsr);
            mpu_get_lpf(&st.chip_cfg.cache.lpf);
            mpu_get_sample_rate(&st.chip_cfg.cache.sample_rate);
            st.chip_cfg.cache.sensors_on = st.chip_cfg.sensors;
            mpu_get_fifo_config(&st.chip_cfg.cache.fifo_sensors);
        }

#if defined MPU6500
        /* Disable hardware interrupts. */
        set_int_enable(0);

        /* Enter full-power accel-only mode, no FIFO/DMP. */
        data[0] = 0;
        data[1] = 0;
        data[2] = BIT_STBY_XYZG;
        if (i2c_write(st.hw->addr, st.reg->user_ctrl, 3, data))
            goto lp_int_restore;

        /* Set motion threshold. */
        data[0] = thresh_hw;
        if (i2c_write(st.hw->addr, st.reg->motion_thr, 1, data))
            goto lp_int_restore;

        /* Set wake frequency. */
        if (lpa_freq == 1)
            data[0] = INV_LPA_1_25HZ;
        else if (lpa_freq == 2)
            data[0] = INV_LPA_2_5HZ;
        else if (lpa_freq <= 5)
            data[0] = INV_LPA_5HZ;
        else if (lpa_freq <= 10)
            data[0] = INV_LPA_10HZ;
        else if (lpa_freq <= 20)
            data[0] = INV_LPA_20HZ;
        else if (lpa_freq <= 40)
            data[0] = INV_LPA_40HZ;
        else if (lpa_freq <= 80)
            data[0] = INV_LPA_80HZ;
        else if (lpa_freq <= 160)
            data[0] = INV_LPA_160HZ;
        else if (lpa_freq <= 320)
            data[0] = INV_LPA_320HZ;
        else
            data[0] = INV_LPA_640HZ;
        if (i2c_write(st.hw->addr, st.reg->lp_accel_odr, 1, data))
            goto lp_int_restore;

        /* Enable motion interrupt (MPU6500 version). */
        data[0] = BITS_WOM_EN;
        if (i2c_write(st.hw->addr, st.reg->accel_intel, 1, data))
            goto lp_int_restore;

        /* Enable cycle mode. */
        data[0] = BIT_LPA_CYCLE;
        if (i2c_write(st.hw->addr, st.reg->pwr_mgmt_1, 1, data))
            goto lp_int_restore;

        /* Enable interrupt. */
        data[0] = BIT_MOT_INT_EN;
        if (i2c_write(st.hw->addr, st.reg->int_enable, 1, data))
            goto lp_int_restore;

        st.chip_cfg.int_motion_only = 1;
        return 0;
#endif
    } else {
        /* Don't "restore" the previous state if no state has been saved. */
        int ii;
        char *cache_ptr = (char*)&st.chip_cfg.cache;
        for (ii = 0; ii < sizeof(st.chip_cfg.cache); ii++) {
            if (cache_ptr[ii] != 0)
                goto lp_int_restore;
        }
        /* If we reach this point, motion interrupt mode hasn't been used yet. */
        return -1;
    }
lp_int_restore:
    /* Set to invalid values to ensure no I2C writes are skipped. */
    st.chip_cfg.gyro_fsr = 0xFF;
    st.chip_cfg.accel_fsr = 0xFF;
    st.chip_cfg.lpf = 0xFF;
    st.chip_cfg.sample_rate = 0xFFFF;
    st.chip_cfg.sensors = 0xFF;
    st.chip_cfg.fifo_enable = 0xFF;
    st.chip_cfg.clk_src = INV_CLK_PLL;
    mpu_set_sensors(st.chip_cfg.cache.sensors_on);
    mpu_set_gyro_fsr(st.chip_cfg.cache.gyro_fsr);
    mpu_set_accel_fsr(st.chip_cfg.cache.accel_fsr);
    mpu_set_lpf(st.chip_cfg.cache.lpf);
    mpu_set_sample_rate(st.chip_cfg.cache.sample_rate);
    mpu_configure_fifo(st.chip_cfg.cache.fifo_sensors);

    if (st.chip_cfg.cache.dmp_on)
        mpu_set_dmp_state(1);

#ifdef MPU6500
    /* Disable motion interrupt (MPU6500 version). */
    data[0] = 0;
    if (i2c_write(st.hw->addr, st.reg->accel_intel, 1, data))
        goto lp_int_restore;
#endif

    st.chip_cfg.int_motion_only = 0;
    return 0;
}

/**
 *  @}
 */

