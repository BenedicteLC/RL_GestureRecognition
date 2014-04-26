/*
   Compute the training error on the random forest.
*/
#include <RandomForest.hpp>
#include <CvRTreesMultiClass.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <stdlib.h>

#include <ros/ros.h>
#include <ros/package.h>

const std::string samplesdbName = "TestingSamples.csv"; 

int main(int argc, char **argv) {
	
	int count = 0;
	int keptCount = 0;
	int errorCount = 0;
	double error = 0.0;
	
	//Load the forest.

	CvRTrees *aforest;
	CvRTreesMultiClass *anotherForest;	
	anotherForest = new CvRTreesMultiClass;
	anotherForest->load("treeState.xml");
	
	//Read the .csv file containing all training samples.    
    ifstream data; //Input stream to read from beginning of file.
    data.open (samplesdbName.c_str(), ios::in);
	if (!data.is_open())
    { 
		 printf("ERROR: cannot read file!!!");
		 return -1; 
	}

    string line;
    
    while (true)
    {		        
        vector<uint8_t> featureVector;
        std::string cell;
        getline(data, line, ','); //Get all directions into line.
        std::stringstream lineStream(line);
        
        //transform string line into a feature vector       
        uint8_t num;
        while (lineStream >> num) featureVector.push_back(num - '0');
                
        getline(data, cell); //Get label.      
        int label = atoi(cell.c_str());
        
        if(!data.good()) break; //Because good() return false only after it fails!    
		
		count++;
		keptCount++;
		//Predict with the forest.

        Mat predictionTraits(1, 180*4, CV_32FC1);
        uint i;
       
        for(i = 0; i < featureVector.size(); i++){
			predictionTraits.at<float>(0, i) = featureVector[i];
		}		

		/*voting confidence part */
		int result = anotherForest->predict_multi_class(predictionTraits);
		//cout<< "The number of votes is: " << votes <<", ";
		if(result == 0)
		{
			keptCount--;

			//Classification failed
		}else{
			cout << "Real label: " << label << " Predicted label: " << result << "\n";
			if(result != label) errorCount++;// count it in errors
		}

    }
	data.close();
	
	if (count == 0){
		cout << "There is no sample in the csv file.";
	}
	else{
		error = 100.0*(double)errorCount / count;
		cout << "The initial number of samples is " << count << ".\n";
		cout << "The number of samples kept is " << keptCount << ".\n";
		cout << "The number of errors is " << errorCount << ".\n";
		cout << "The error is " << error << "%.\n";
	}
	return 0;
}
