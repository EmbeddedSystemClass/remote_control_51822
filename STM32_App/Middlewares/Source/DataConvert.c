#include "dataconvert.h"
#include "includes.h"

/**
  * @brief  Convert an Integer to a string
  * @param  str: The string
  * @param  intnum: The integer to be converted
  * @retval None
  */
void Int2Str(uint8_t * str, int32_t intnum)
{
    uint32_t i, Div = 1000000000, j = 0, Status = 0;

    for (i = 0; i < 10; i++)
    {
        str[j++] = (intnum / Div) + 48;
        intnum = intnum % Div;
        Div /= 10;
        if ((str[j-1] == '0') & (Status == 0))
        {
            j = 0;
        }
        else
        {
            Status++;
        }
    }
}

/**
  * @brief  Convert a string to an integer
  * @param  inputstr: The string to be converted
  * @param  intnum: The integer value
  * @retval 1: Correct
  *         0: Error
  */
uint32_t Str2Int(uint8_t *inputstr, int32_t *intnum)
{
  uint32_t i = 0, res = 0;
  uint32_t val = 0;

  if (inputstr[0] == '0' && (inputstr[1] == 'x' || inputstr[1] == 'X'))
  {
    if (inputstr[2] == '\0')
    {
      return 0;
    }
    for (i = 2; i < 11; i++)
    {
      if (inputstr[i] == '\0')
      {
        *intnum = val;
        /* return 1; */
        res = 1;
        break;
      }
      if (ISVALIDHEX(inputstr[i]))
      {
        val = (val << 4) + CONVERTHEX(inputstr[i]);
      }
      else
      {
        /* Return 0, Invalid input */
        res = 0;
        break;
      }
    }
    /* Over 8 digit hex --invalid */
    if (i >= 11)
    {
      res = 0;
    }
  }
  else /* max 10-digit decimal input */
  {
    for (i = 0;i < 11;i++)
    {
      if (inputstr[i] == '\0')
      {
        *intnum = val;
        /* return 1 */
        res = 1;
        break;
      }
      else if ((inputstr[i] == 'k' || inputstr[i] == 'K') && (i > 0))
      {
        val = val << 10;
        *intnum = val;
        res = 1;
        break;
      }
      else if ((inputstr[i] == 'm' || inputstr[i] == 'M') && (i > 0))
      {
        val = val << 20;
        *intnum = val;
        res = 1;
        break;
      }
      else if (ISVALIDDEC(inputstr[i]))
      {
        val = val * 10 + CONVERTDEC(inputstr[i]);
      }
      else
      {
        /* return 0, Invalid input */
        res = 0;
        break;
      }
    }
    /* Over 10 digit decimal --invalid */
    if (i >= 11)
    {
      res = 0;
    }
  }

  return res;
}

int M2Mstoh(char* dest,char* src,int n)
{
   int i=0;

   for(i=0;i<n;i++)
   {
      if((src[i]-'0')<=9&&(src[i]-'0')>=0) //0-9
         dest[i/2]+=(src[i]-'0');
      else if((src[i]-'A')<=5&&(src[i]-'A')>=0) //A-F
         dest[i/2]+=(src[i]-'A'+10);
      else if((src[i]-'a')<=5&&(src[i]-'a')>=0) //a-f
         dest[i/2]+=(src[i]-'a'+10);
      else
         return (i+1)/2;

      if(0==(i%2))
         dest[i/2]=dest[i/2]<<4; //High 4 bit
   }

   return (n+1)/2;
}

int M2Mhtos(char* dest,char* src,int n)
{
   int i=0;
   char ch1,ch2;

   for(i=0;i<n;i++)
   {
      ch1 = (( src[i]&0xF0)>>4);
      ch2 = ( src[i]&0x0F);

      dest[2*i] = (ch1<10)?('0'+ch1):('A'+ch1-10);
      dest[2*i+1] = (ch2<10)?('0'+ch2):('A'+ch2-10);
   }

   return 2*n;
}

int Convertu8ArrTou32(uint32_t *pu32DstValue,uint8_t *pu8SrcArr)
{
    int ret = 0;
    uint32_or_uint8_u a;

    for(uint8_t i = 0;i < 4;i ++)
    {
        a.arr[i] = pu8SrcArr[i];
    }

    if(pu32DstValue != NULL)
    {
        *pu32DstValue = a.l;
        ret = 0;
    }
    else
    {
        ret = -1;
    }

    return (ret);
}

int Convertu32Tou8Arr(uint8_t *pu8DstArr,uint32_t u32SrcValue)
{
    int ret = 0;
    uint32_or_uint8_u a;

    a.l = u32SrcValue;
    if(pu8DstArr != NULL)
    {
        for(uint8_t i = 0;i < 4;i ++)
        {
            *(pu8DstArr + i) = *(a.arr + i);
        }
        ret = 0;
    }
    else
    {
        ret = -1;
    }

    return (ret);
}

int Convertu8ArrTou32Big(uint32_t *pu32DstValue,uint8_t *pu8SrcArr)
{
    int ret = 0;
    uint32_t value = 0;

    value = ((pu8SrcArr[0] << 24) | (pu8SrcArr[1] << 16) | (pu8SrcArr[3] << 8) | (pu8SrcArr[4]));
    if(pu32DstValue != NULL)
    {
        *pu32DstValue = value;
        ret = 0;
    }
    else
    {
        ret = -1;
    }

    return (ret);
}

int Convertu32Tou8ArrBig(uint8_t *pu8DstArr,uint32_t u32SrcValue)
{
    int ret = 0;

    if(pu8DstArr != NULL)
    {
        *(pu8DstArr) = (u32SrcValue >> 24);
        *(pu8DstArr + 1) = (u32SrcValue >> 16);
        *(pu8DstArr + 2) = (u32SrcValue >> 8);
        *(pu8DstArr + 3) = u32SrcValue;

        ret = 0;
    }
    else
    {
        ret = -1;
    }

    return (ret);

}


