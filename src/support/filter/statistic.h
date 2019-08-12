#ifndef _STATISTIC_H_
#define _STATISTIC_H_
#include "Arduino.h"

#define arrayLength(x)    sizeof(x)/sizeof(x[0])

template <typename T, size_t size_>
float getSum( T (&series)[size_])
{
  float sum = 0;
  for(size_t i = 0; i < size_; i++)
  {
    sum+=series[i];
  }
  return sum;
}


template <typename T, size_t size_>
T getAverage(const T (&series)[size_])
{
   return ( getSum(series) / size_ ) ;
}

template <typename T, size_t size_>
float getGeoMean(const T (&series)[size_])
{
  float sum = 0;
  for ( size_t i = 0; i< size_; i++)
  {
    sum += log(series[i]);
  }
  return exp(sum/size_);
}

template <typename T, size_t size_>
void copyArrayTo(const T (&from)[size_], T (&to)[size_])
{
//  size_t num = (size_From < size_To) ?size_From:size_To;
  for(size_t i = 0; i < size_; i++)
  {
    to[i] = from[i];
  }
}

template <typename T, size_t size_>
T getLowest(const T (&series)[size_])
{
  T tmpSeries[size_];
  copyArrayTo(series,tmpSeries);
  bubbleSort(tmpSeries);
  return tmpSeries[0];
}

template <typename T, size_t size_>
T getHighest(const T (&series)[size_])
{
  T tmpSeries[size_];
  copyArrayTo(series,tmpSeries);
  bubbleSort(tmpSeries);
  return tmpSeries[size_ - 1];
}

template <typename T, size_t size_>
void bubbleSort( T (&series)[size_])
{
  T tmp = 0;
  for(size_t i = 0; i < size_; i++)
  {
    for ( size_t j = 0; j < (size_ - 1); j++)
    {
      if ( series[j] > series[j+1] )
      {
        tmp         = series[j];
        series[j]   = series[j+1];
        series[j+1] = tmp;
      }
    }
  }
}

template <typename T, size_t size_>
float getDeviasion( T (&series)[size_])
{
  float sum = getAverage(series);
  float std = 0;
  for(size_t id = 0; id<size_; id++)
  {
    std+=pow((series[id]-sum),2);
  }
  return (sqrt(std/(size_)));
}

template <typename T, size_t size_>
T getMedian( T (&series)[size_])
{
  T tmpSeries[size_];
  copyArrayTo(series,tmpSeries);
  bubbleSort(tmpSeries);
  if (bitRead(size_,0)==1)
    return tmpSeries[size_/2];
  else

  return (tmpSeries[(size_/2)-1]+tmpSeries[size_/2])/2;
}

template <typename T, size_t size_>
T getMode(const T (&series)[size_],float epsilon = 0.00001){ //calculate the mode.
                                                              //epsilon is the tolerance for two measurements to be equivalent.
  T sorted[size_]; //Temporary array to sort values.
  float unique[size_]; //Temporary array to store unique values
  int uniquect[size_]; //Temporary array to store unique counts
  copyArrayTo(series,sorted);

  bubbleSort(sorted); // Sort the values

  // Now count the number of times each unique number appears in the sorted array.
  unique[0]=sorted[0];
  uniquect[0]=1;
  int p=0; // counter for # unique numbers
  int maxp=0;
  int maxidx=0;
  for(int i=1;i<size_;i++)
  {
    if(fabs(sorted[i]-sorted[p])<epsilon)
    {
      uniquect[p]++; //if same number again, add to count
      if(uniquect[p]>maxp)
      {
        maxp=uniquect[p];
        maxidx=p;
      }
    }
    else
    {
      p++;
      unique[p]=sorted[i];
      uniquect[p]=1;
    }
  }
  if (maxp>1)
  {
    return unique[maxidx]; //If there is more than one mode, return the lowest one.
  }
  else
  {
    return 0; //If there is no mode, return a zero.
  }
}

template <typename T, size_t size_>
void roundF( T (&series)[size_], int p) //round float variable to a given # decimals, p
{
  float precision=pow(10.0,p);
  for(int i=0;i<size_;i++){
  series[i]=round(series[i]*precision)/precision;
  }
}

template <typename T>
float roundF( const T al, int p) //round float variable to a given # decimals, p
{
  float precision=pow(10.0,p);
  return round(al*precision)/precision;
}


#endif
