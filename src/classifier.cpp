#include <RandomForest.hpp>
#include <CvRTreesMultiClass.hpp>

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <stdlib.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <gesture_recognition/Feat.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

vector<uint8_t> rightFeaturesXY;
vector<uint8_t> rightFeaturesZ;
vector<uint8_t> leftFeaturesXY;
vector<uint8_t> leftFeaturesZ;

ros::Publisher chairCommand;
//CvRTrees *aforest;
CvRTreesMultiClass *anotherForest;

void vectorCallback(const gesture_recognition::Feat& msg){        
	rightFeaturesXY = msg.rightXY;
	rightFeaturesZ = msg.rightZ;
	leftFeaturesXY = msg.leftXY;
	leftFeaturesZ = msg.leftZ;
    
	std_msgs::Int32 command; //Wheelchair commands.

	vector<uint8_t> featureInput;       
	uint i;     
    
	 //Create the input vector to be classified.
	 for(i = 0 ; i < rightFeaturesXY.size() ; i++)
	 {
			featureInput.push_back(rightFeaturesXY[i]);
			featureInput.push_back(rightFeaturesZ[i]);
			featureInput.push_back(leftFeaturesXY[i]);
			featureInput.push_back(leftFeaturesZ[i]);
	 } 
			 
	//Prediction input   
	Mat predictionTraits(1, MATRIX_SIZE, CV_32FC1);
	uint k;
	for(k = 0; k < MATRIX_SIZE; k++)
	{		
		predictionTraits.at<float>(0, k) = featureInput[k];		
	}	
		
	//int result = (int)aforest->predict(predictionTraits/*,mask*/);  
	int result = anotherForest->predict_multi_class(predictionTraits);
	
	//cout<< "The number of votes for this label is: "<< votes << "\n";

    if( result == 0)
    {
		//Wheelchair.		
		cout << "Gesture discarded\n";
	}

    command.data = 0 ;
	if(result == 1) { //Stop
	
		cout << "Stop\n";
		command.data = 0;
	}
	
	if(result == 2) { //Invite:follow
	
		cout << "Invite\n";
		//command.data = 30;
	}

	if(result == 3) { //Up: forward
	
		cout << "Up\n";
		command.data = 11;
	}
	
	if(result == 4) { //Down: backward
	
		cout << "Down\n";
		command.data = 15;
	}
	
	if(result == 5) { //Demo 1 hand:park left
	
		cout << "Demo 1 hand\n";
		//command.data = 20;
	}
	
	if(result == 6) { //Demo 2 hands
	
		cout << "Demo 2 hands\n";
		//command.data = No mapping yet
	}
	
	if(result == 7) { //CW circle: Turn left 90.		
	
		cout << "CW circle\n";
		//command.data = 10;
	}
	
	if(result == 8) { //CCW circle: Turn right 90.
		
		cout << "CCW circle\n";
		//command.data = 12;
	}
	
	if(result == 9) { //2 circles: Turn 180.
		
		cout << "2 circles\n";
		//command.data = 21;
	}	
		
	chairCommand.publish(command);	
	
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "classifier");  
	ros::NodeHandle nodeH;
	
	chairCommand = nodeH.advertise<std_msgs::Int32>("GUIControl", 1000);
	
    //Subscribing to topic "features".
    ros::Subscriber sub = nodeH.subscribe("features", 100, vectorCallback);
    ros::Rate r(30);   
	
	anotherForest = new CvRTreesMultiClass;
	anotherForest->load("treeState.xml");
	
	while(ros::ok()){		
		ros::spinOnce();
	}
	return 0;
}
