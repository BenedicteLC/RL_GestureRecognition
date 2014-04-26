#ifndef CvRTreesMultiClass_hpp
#define CvRTreesMultiClass_hpp

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h> 
#include <opencv2/highgui/highgui.hpp>

#include <ctype.h> 
#include <stdio.h>

using namespace cv;
using namespace std;


class CvRTreesMultiClass : public CvRTrees
{
    public:
    int predict_multi_class(const Mat& sample) const;
};          
#endif                    
