/*
	This library's purpose to create simple-filter of data series using data array
	simply way to filtering data-series on mcu ( arduino or stm32 board)

	Lib_name 	: FILTER
	AUTHOR		: Andri Setyabudi ;

*/

#pragma once
#include <Arduino.h>
#ifndef ARRAYFILTER_H
#define ARRAYFILTER_H


#define MAX_DATA_POINTS		10

#define MAF_type  1
#define AF_type   2
#define HF_type   3

template<typename T, size_t N> size_t ArraySize(T(&)[N]){ return N; }
#define Array_Size(x) sizeof(x)/sizeof(x[0])

template<typename T> float SumOfArray(T arrVar)
{
  float tmp = 0;
  uint8_t arrLen = (uint8_t)Array_Size(arrVar);
  for (uint8_t loc = 0; loc<arrLen; loc++)
  {tmp+=arrVar[loc];};
  return tmp;
}


class ArrayFilter
{
	protected:
		float dataSeries[MAX_DATA_POINTS];
    float currValue;
    uint8_t filterType:3;
    boolean setData;
	public:
		ArrayFilter(){}
    void setType(uint8_t type, int orderVal)
    {
      order = (orderVal>MAX_DATA_POINTS)?MAX_DATA_POINTS:orderVal;
      order = (order<1)?1:order;
      filterType=(type>3)?3:(type<1)?1:type;
      switch(filterType)
      {
        case MAF_type:
          break;
        case AF_type:
          break;
        default:    // default is hanning - filter
          order = 3;
          break;
      };
    }
    void ShiftOnce()
    {
      for(n_loop=order; n_loop>0; n_loop--)
      {
        dataSeries[n_loop]=dataSeries[n_loop-1];
      }
    }
		float MovingAverageFilter( float newData)
		{
      if ( filterType != MAF_type) return 0;
			ShiftOnce();
      currValue=0;
			dataSeries[0]=newData;
			for( n_loop=0; n_loop<order; n_loop++)
			{
			  currValue+=dataSeries[n_loop];
			}
			currValue/=(float)order;
      n_loop=0;
			return currValue;
		}
   template<typename T> float AverageOf(T dataArray)
   {
      return SumOfArray(dataArray)/Array_Size(dataArray);
   }
   float AverageFilter()
   {
      if ( filterType != AF_type) return 0;
      if ( setData ) return SumOfArray(dataSeries)/order;
   }
   float HanningFilter(float newData)
   {
      if(filterType !=HF_type)  return 0;
      ShiftOnce();
      dataSeries[0]=newData;
      return (dataSeries[0]+2*dataSeries[1]+dataSeries[2])/4;
   }

   float getValue() const{return currValue;}
   void  setValue(float dataValue, uint8_t loc)
   {
      loc=(loc>MAX_DATA_POINTS)?MAX_DATA_POINTS:((loc<0)?0:loc);
      dataSeries[loc]=dataValue;
      setData = true;
   }
	private:
		uint8_t order:5;
    uint8_t n_loop;

};
#endif
