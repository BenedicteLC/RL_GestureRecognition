#include <CvRTreesMultiClass.hpp>
						                    
int CvRTreesMultiClass::predict_multi_class( const Mat& sample) const
{
	int MaxVote = 0;
	int MaxLabel = 999;
	int k;
	int labelVoteArray[10] = {0,0,0,0,0,0,0,0,0,0}; 
	
	if( nclasses > 0 ) //classification
	{
		int valuePredicted;
		for( k = 0; k < ntrees; k++ )
		{
			CvDTreeNode* predicted_node = trees[k]->predict( sample);
			valuePredicted = (int)predicted_node->value;
			labelVoteArray[valuePredicted]++;
		}
		
		//get max of array
		for (int i= 0; i < 10; i++)
		{ 
			//cout << "label at "<< i << " has " << labelVoteArray[i] << " votes\n";	
			if(labelVoteArray[i] > MaxVote)
			{
				MaxVote = labelVoteArray[i];
				MaxLabel = i;
			}
		}
	}
	
	//cout<< "the result from the tree is " << MaxLabel <<"\n";
	if(MaxVote <  35) 
	{
		MaxLabel = 0; //Votes number is not enough.	
	}
	
	return MaxLabel;
}                             
