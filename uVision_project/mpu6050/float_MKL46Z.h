/**
 * @addtogroup  DRIVERS Sensor Driver Layer
 * @brief       Hardware drivers to communicate with sensors via I2C.
 * @{
 * 		@file float_MKL46Z.h
 * 		The library for parsing any float-realted operations performed on FRDM-KL46Z board.
 */

#ifndef FLOAT_MKL46Z_H
#define FLOAT_MKL46Z_H

#include<stdio.h>
#include<math.h>

/** 
 * @brief Reverses a string 'str' of length 'len'.
 * @param[in, out] str string to reverse
 * @param[in] len length of the string to reverse
 */
void reverse(char *str, int len);
  
/** 
 * @brief Converts a given integer x to string str[]. d is the number
 * of digits required in output. If d is more than the number
 * of digits in x, then 0s are added at the beginning.
 * @param[in] 	x integer to convert
 * @param[out] 	str[] string array to store the result
 * @param[in] 	d number of digits that must be generated
 * @return length of the string after the conversion
 */
int intToStr(int x, char str[], int d);

/** 
 * @brief Converts a floating point number to string.
 * @param[in] n floating point number to convert
 * @param[out] buffer buffer to which output the result
 * @param[in] afterpoint number of decimal places to take into account
 */
void ftoa(float n, char *buffer, int afterpoint);
  
#endif

/**@}*/
