#ifndef _DATACONVERT_H_
#define _DATACONVERT_H_

#include "stm32f2xx.h"

#define ABS(x)                  (x < 0) ? (-x) : x
#define MAX(a,b)                (a < b) ? (b) : a
#define MIN(a,b)                (a > b) ? (b) : a
#define IS_AF(c) 				 ((c >= 'A') && (c <= 'F'))
#define IS_af(c)                ((c >= 'a') && (c <= 'f'))
#define IS_09(c)                ((c >= '0') && (c <= '9'))
#define ISVALIDHEX(c)           IS_AF(c) || IS_af(c) || IS_09(c)
#define ISVALIDDEC(c)           IS_09(c)
#define CONVERTDEC(c)           (c - '0')

#define CONVERTHEX_alpha(c)     (IS_AF(c) ? (c - 'A'+10) : (c - 'a'+10))
#define CONVERTHEX(c)           (IS_09(c) ? (c - '0') : CONVERTHEX_alpha(c))

void Int2Str(uint8_t * str, int32_t intnum);
uint32_t Str2Int(uint8_t *inputstr, int32_t *intnum);
int M2Mhtos(char* dest,char* src,int n);
int M2Mstoh(char* dest,char* src,int n);
int Convertu8ArrTou32(uint32_t *pu32DstValue,uint8_t *pu8SrcArr);
int Convertu32Tou8Arr(uint8_t *pu8DstArr,uint32_t u32SrcValue);
int Convertu8ArrTou32Big(uint32_t *pu32DstValue,uint8_t *pu8SrcArr);
int Convertu32Tou8ArrBig(uint8_t *pu8DstArr,uint32_t u32SrcValue);

#endif

