#ifndef sample_hpp
#define sample_hpp

#include <vector>
#include <stdio.h> 
#include <stdint.h>

using namespace std;

class Sample{

public:
    vector<uint8_t> featureVector;
    int label;
    
    Sample(vector<uint8_t> afeatureVector, int alabel);

	void setLabel(int alabel);
      
};
#endif      
