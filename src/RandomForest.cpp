#include <RandomForest.hpp>
  
RandomForest::RandomForest( vector<Sample> samples) 
{
	trainingSamples = samples; // training samples vector intialized 	
}

void RandomForest::train()
{	     	
	Mat trainingMat(trainingSamples.size(), MATRIX_SIZE, CV_32FC1);
	Mat labelMat(trainingSamples.size(), 1, CV_32FC1);
	
	uint j;
	for (uint i = 0; i < trainingSamples.size(); i++)
	{	         
		Sample trainingSample = trainingSamples[i];

		for( j = 0; j < trainingSample.featureVector.size(); j++)
		{
		  trainingMat.at<float>(i, j) = trainingSample.featureVector[j];		 
		}	       
			  
		labelMat.at<float>(i, 0) =  trainingSample.label;
   }	   
	   
   Mat varType(MATRIX_SIZE+1, 1, CV_8U);
   varType.setTo(Scalar(CV_VAR_NUMERICAL) ); // all inputs are numerical
   varType.at<uchar>(MATRIX_SIZE, 0) = CV_VAR_CATEGORICAL;
   float priors[] = {1,1,1,1,1,1,1,1,1};  // weights of each classification for classes (all equal as equal samples of each digit)
   CvRTParams params = CvRTParams(60, // max depth
									2,//min sample count: IS A SMALL % OF THE TOTAL AMOUNT OF SAMPLES
									0,//regression accuracy: N/A here
									false,//compute surrogate split,no missing data
									15,// max number of categories (use sub-optimal algorithm for larger numbers)
									priors,// the array of priors
									false,// calculate variable importance
									6, // number of variables randomly selected at node and used to find the best split(s). 
									//4 = sqrt of number of features (initially 4)
									100, // max number of trees in the forest
									0.01f,// forest accuracy
									CV_TERMCRIT_ITER |CV_TERMCRIT_EPS);
		  
   forest = new CvRTrees;      
	   
   //  forest->train(trainingMat, CV_ROW_SAMPLE, labelMat, Mat(), Mat(), varType, mask, params);
   forest->train(trainingMat, CV_ROW_SAMPLE, labelMat, Mat(), Mat(), varType, Mat(), params);
   forest->save("treeState.xml");
   cout << "Forest built!\n" ;
}
