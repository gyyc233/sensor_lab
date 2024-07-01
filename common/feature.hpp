#include <iostream>
#include <cassert>
#include <cmath>

template <typename Type, typename Iterator> Type 
getFuture(Iterator begin, Iterator end){
  Type ret;
  size_t num=0;
  for(Iterator it=begin;it!=end;it++){
    ret+=*it;
    num++;
  }
  assert(num!=0);

  return ret/static_cast<Type>(num);
}

template <typename Type, typename Iterator> Type
getVariance(Iterator begin, Iterator end){
  Type sum = 0;
  Type average = getFuture<Type,Iterator>(begin,end);
  size_t num=0;
  for(Iterator it=begin;it!=end;it++){
    sum+=pow(*it-average,2);
    num++;
  }

  assert(num!=0);
  sum=sum/static_cast<Type>(num);

  return (sum);
}

template <typename Type, typename Iterator> Type 
getStandardDeviation(Iterator begin, Iterator end){
  Type ret = getVariance <Type,Iterator>(begin,end);
  return sqrt(ret);
}
