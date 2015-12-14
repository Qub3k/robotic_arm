#include "float_MKL46Z.h"
 
/* 
 * Reverses a string 'str' of length 'len' 
 */
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
 
/* 
 * Converts a given integer x to string str[].  d is the number
 * of digits required in output. If d is more than the number
 * of digits in x, then 0s are added at the beginning.
 */
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while(x){
        str[i++] = (x%10) + '0'; // automatically converts to char
        x = x/10;
    }
 
    // If number of digits required is more, then
    // add 0s at the beginning
    while(i < d)
        str[i++] = '0';
 
    reverse(str, i);
    str[i] = '\0';
    return i;
}

/* 
 * Converts a floating point number to string.
 */
void ftoa(float n, char *res, int afterpoint){
  int ipart = 0;
  float fpart = .0f;
  int i = 0;
  
  /* Extract integer part */
  ipart = (int)n;
  
  /* Extract floating point part */
  fpart = n - (float)ipart;
  
  /* Convert integer part to string */
  i = intToStr(ipart, res, 0);
  
  /* Check for the display options after point */
  if(afterpoint != 0) {
    res[i] = '.';
    
    // Get the value of fraction part upto given no.
    // of points after dot. The third parameter is needed
    // to handle cases like 233.007
    fpart = fpart * pow(10, afterpoint);

    intToStr((int)fpart, res + i + 1, afterpoint);
  }
}
