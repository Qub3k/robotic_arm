#ifndef FLOAT_MKL46Z_H
#define FLOAT_MKL46Z_H

#include<stdio.h>
#include<math.h>

/* 
 * Reverses a string 'str' of length 'len' 
 */
void reverse(char *str, int len);
  
/* 
 * Converts a given integer x to string str[].  d is the number
 * of digits required in output. If d is more than the number
 * of digits in x, then 0s are added at the beginning.
 */
int intToStr(int x, char str[], int d);

/* 
 * Converts a floating point number to string.
 */
void ftoa(float n, char *res, int afterpoint);
  
#endif
