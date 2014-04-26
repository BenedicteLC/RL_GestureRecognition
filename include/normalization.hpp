#ifndef normalization_hpp
#define normalization_hpp

#include <vector>
#include <stdint.h>
#include <stdlib.h>  

using namespace std;

void normalize(vector<uint8_t> *rightFeaturesXY, vector<uint8_t> *rightFeaturesZ,
  vector<uint8_t> *leftFeaturesXY, vector<uint8_t> *leftFeaturesZ);
  
#endif      
