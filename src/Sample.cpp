#include <Sample.hpp>
    
Sample::Sample(vector<uint8_t> afeatureVector, int alabel) 
{       
	featureVector = afeatureVector;
	label = alabel;
}
  
/*Sample(int featureVectorSize) {
    featureVector = new int[featureVectorSize]();
}*/
void Sample::setLabel(int alabel) {
        label = alabel;
}

