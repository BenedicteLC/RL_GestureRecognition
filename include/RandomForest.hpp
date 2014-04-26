#ifndef RandomForest_hpp
#define RandomForest_hpp

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/opencv.hpp>

#include <ctype.h> 

#include <Sample.hpp>

#define MATRIX_SIZE 180*4

using namespace cv;

class RandomForest{
 public:
	CvRTrees *forest;
	vector<Sample> trainingSamples;
  
    RandomForest( vector<Sample> samples); 
    void train();  
   
};
#endif      
