#ifndef FLOAT_MKL46Z_H
#define FLOAT_MKL46Z_H

#include<stdio.h>
#include<math.h>

/** 
 * @brief Reverses a string 'str' of length 'len'.
 */
void reverse(char *str, int len);
  
/** 
 * @brief Converts a given integer x to string str[]. d is the number
 * of digits required in output. If d is more than the number
 * of digits in x, then 0s are added at the beginning.
 * @param[in] x		given integer x 
 * @param[in] str   string array
 * @param[in] d		control the right representation
 */
int intToStr(int x, char str[], int d);

/** 
 * @brief Converts a floating point number to string.
 * @param[in] n				given floating point
 * @param[in] buffer		simple buffer
 * @param[in] afterpoint	parameter used to handle with more complex notation for e.g 233.007
 */
void ftoa(float n, char *buffer, int afterpoint);
  
#endif
