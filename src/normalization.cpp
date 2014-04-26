#include "normalization.hpp"

void normalize(vector<uint8_t> *rightFeaturesXY, vector<uint8_t> *rightFeaturesZ,
  vector<uint8_t> *leftFeaturesXY, vector<uint8_t> *leftFeaturesZ)
{
	
	int size = (*rightFeaturesXY).size();
	int zeroCounter = 0; 
	
	//Count the number of 0s that need to be replaced.
	for(int i = size - 1 ; i > 30 ; i--) //End at 30 because the minimum vector length is 30.
	{	
		if((*rightFeaturesXY)[i] != 0)
		{
			break;			
		}
		zeroCounter++;
	}
		
	//Number of non-zero entries in the vector.
	int numberEntries = size - zeroCounter;
	
	//Find how many holes minimum must be filled for each vector entry.
	//-11 because we're not duplicating the vector borders.
	int holesPerEntry = zeroCounter / (numberEntries - 11); 
	//Find how many holes are left.
	int holesLeft = zeroCounter - holesPerEntry * (numberEntries - 11);
		
	//Where to replace the next zero entry.
	int vectorPointer = size - 1;
	
	for(int i = numberEntries - 1 ; i >= numberEntries - 3  ; i--)
	{
		(*rightFeaturesXY)[vectorPointer] = (*rightFeaturesXY)[i];
		(*rightFeaturesZ)[vectorPointer] = (*rightFeaturesZ)[i];
		(*leftFeaturesXY)[vectorPointer] = (*leftFeaturesXY)[i];
		(*leftFeaturesZ)[vectorPointer] = (*leftFeaturesZ)[i];
		vectorPointer--; 
	}
	
	
	//Fill the zero entries.
	for(int i = numberEntries - 4 ; i >= 8; i--) //-4 & 8: ignore start and end of vector.
	{	
	
		//Fill the minimum number of holes for this entry.
		for(int j = 0 ;  j <= holesPerEntry ; j++)
		{			
			(*rightFeaturesXY)[vectorPointer] = (*rightFeaturesXY)[i];
			(*rightFeaturesZ)[vectorPointer] = (*rightFeaturesZ)[i];
			(*leftFeaturesXY)[vectorPointer] = (*leftFeaturesXY)[i];
			(*leftFeaturesZ)[vectorPointer] = (*leftFeaturesZ)[i];
			vectorPointer--; 
		}
		
		//Fill an extra hole if need be.
		if(holesLeft != 0)
		{
			(*rightFeaturesXY)[vectorPointer] = (*rightFeaturesXY)[i];
			(*rightFeaturesZ)[vectorPointer] = (*rightFeaturesZ)[i];
			(*leftFeaturesXY)[vectorPointer] = (*leftFeaturesXY)[i];
			(*leftFeaturesZ)[vectorPointer] = (*leftFeaturesZ)[i];
			vectorPointer--; 
			holesLeft--;
		}
	}
	
	return;
}

/**Testing.**/
/*int main ()
{
	//Random number generator.
	srand (time(NULL));
	int r; 
	vector<uint8_t> rightFeaturesXY(100);
	
	for(int i = 0; i < 35; i++)
	{
		r = rand() % 9 + 1;
		rightFeaturesXY[i] = r;
	}
	for(int i = 35; i < 100; i++)
	{
		rightFeaturesXY[i] = 0;		
	}
	
	for(int i = 0; i < 100; i++){
		cout << +rightFeaturesXY[i];
	}
	cout << "\n";
	
	normalize(&rightFeaturesXY);
	
	for(int i = 0; i < 100; i++){
		cout << +rightFeaturesXY[i];
	}
	cout << "\n";	
	
}*/ /**TESTING**/
	
