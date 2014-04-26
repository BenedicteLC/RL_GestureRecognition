#include "filter.hpp"

void filter(vector<uint8_t> *rightFeaturesXY, vector<uint8_t> *leftFeaturesXY)
{
	int size = (*rightFeaturesXY).size();
	int threshold = 5;
	
	//Applying directional equivalence filter.
	for(int index = 1 ; index < size - 1 ; index++)
	{
		if( abs((*rightFeaturesXY)[index] - (*rightFeaturesXY)[index - 1]) +
			abs((*rightFeaturesXY)[index] - (*rightFeaturesXY)[index + 1]) > threshold )
		{
			(*rightFeaturesXY)[index] = (*rightFeaturesXY)[index + 1];
		}
		if( abs((*leftFeaturesXY)[index] - (*leftFeaturesXY)[index - 1]) + 
			abs((*leftFeaturesXY)[index] - (*leftFeaturesXY)[index + 1 ])  > threshold )
		{
			(*leftFeaturesXY)[index] = (*leftFeaturesXY)[index + 1];
		}
	}
}

//Old filter backup. Just removes middles. Was not optimal.
/*	//Count total amount of MIDDLEs.
	for(int index=1; index<size; index++)
	{
		if((*rightFeaturesXY)[index] == 9)
		{
			count++;
			
		}
	}	
	
	//If first direction is 9 for right XY.
	//Find the first non-9 entry.
	while((*rightFeaturesXY)[position] == 9 && position < size)
	{
		position++;
	}
	if(position < size)
		(*rightFeaturesXY)[0] = (*rightFeaturesXY)[position];
		
	
	//replacing 9s from XY vectors.
	for(int index=1; index<size; index++)
	{
		if((*rightFeaturesXY)[index] == 9)
		{
			(*rightFeaturesXY)[index] =(*rightFeaturesXY)[index-1];	
		}
	}	
	
	position = 0;	
	
	//If first direction is 9 for right XY.
	while((*leftFeaturesXY)[position] == 9 && position < size)
	{
		position++;
	}
	if(position < size)
		(*leftFeaturesXY)[0] = (*leftFeaturesXY)[position];
		
	
	//replacing 9s from XY vectors.
	for(int index=1; index<size; index++)
	{
		if((*leftFeaturesXY)[index] == 9)
		{
			(*leftFeaturesXY)[index] =(*leftFeaturesXY)[index-1];	
		}
	}*/
	
