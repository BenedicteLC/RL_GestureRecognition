/*
    The trainer receives a feature vector from the feature extractor
    asks the user for its expected output
    adds the feature vector and the label to an .csv file 
    creates a random forest object
    reads the .csv file containing all training samples 
    adds them to the training samples of the forest
    trains the forest with these samples
    saves the state model of the tree 
*/
#include <RandomForest.hpp>

#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include <ros/package.h>
#include <gesture_recognition/Feat.h>

using namespace std; // loads the sample database from file (which is a CSV text file)
vector<uint8_t> rightFeaturesXY;
vector<uint8_t> rightFeaturesZ;
vector<uint8_t> leftFeaturesXY;
vector<uint8_t> leftFeaturesZ;
ofstream samplesdb; //Output stream to write to file.
static vector<Sample> mySamples;
const std::string samplesdbName = "TrainingSamples.csv";  

int read_data_from_csv(const char* filename)
{	
	ifstream inputFile; // Input stream to read from beginning of file.
	string myString;
	
	inputFile.open (samplesdbName.c_str(), ios::in);
	if (!inputFile.is_open())
    { 
		 printf("ERROR: cannot read file.");
		 return -1; 
	}
	
	while (!inputFile.eof())
	{	
		vector<uint8_t> featureVector;		
        std::string cell;
        uint8_t num;
        
        //Get feature vector.
        getline(inputFile, myString, ',');
        if(myString.compare("") == 0) break; //Safety measure.
		
        std::stringstream lineStream(myString); 
        //- '0' is for ASCII to int conversion.
        while (lineStream >> num) featureVector.push_back(num - '0');
        
        getline(inputFile, cell); //Get label.        
        int label = atoi(cell.c_str());
        
        //Add to samples.
       	Sample *aSample = new Sample(featureVector, label);		
		mySamples.push_back(*aSample);		
    }
    inputFile.close();
    return 1; // all OK
}

void vectorCallback(const gesture_recognition::Feat& msg){    
		int label;
        uint i;
		rightFeaturesXY = msg.rightXY;
		rightFeaturesZ = msg.rightZ;
		leftFeaturesXY = msg.leftXY;
		leftFeaturesZ = msg.leftZ;
       
        cout << "\n"; 
        cout << "Please enter the correct label for this vector.\n";
        cin >> label;
		
		//File will contain rightXY, rightZ, leftXY, leftZ.
        for(i = 0 ; i < rightFeaturesXY.size(); i++){
            samplesdb << (int)rightFeaturesXY[i];     
            samplesdb << (int)rightFeaturesZ[i];     
            samplesdb << (int)leftFeaturesXY[i];     
            samplesdb << (int)leftFeaturesZ[i];  
            /**TESTING*/        
           /*  cout << +rightFeaturesXY[i];   
                cout << +rightFeaturesZ[i];  
                cout << +leftFeaturesXY[i];  
                cout << +leftFeaturesZ[i];    *//**TESTING*/      
        }
      
        samplesdb << "," ;
        samplesdb << label << endl;   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "trainer");  
	ros::NodeHandle nodeH;
    
    //Subscibing to topic "features".    
    ros::Subscriber sub = nodeH.subscribe("features", 100, vectorCallback);
	ros::Rate r(30);   
    cout << "Ready.\n";
    
    //Enable write/append to file.
    samplesdb.open (samplesdbName.c_str(), ios::out /*| ios::ate*/ | ios::app/* | ios::binary*/) ;
    if (!samplesdb.is_open())
    { 
		 printf("ERROR: cannot read file.");
		 return -1; 
	}
    
	while(ros::ok())
	{	
		ros::spinOnce();
	}	
	
	samplesdb.close(); //Close file writer.
	
    //read the .csv file containing all training samples      
    read_data_from_csv(samplesdbName.c_str());
	cout << "The number of samples in the csv file is " << mySamples.size() << ".\n";
	
	/**TESTING**/
	/*for (int i = 0 ; i < (int)mySamples.size() ; i++){
		cout << i << ") " << mySamples[i].toString() << "\n"; 
	}*/ /**TESTING**/	
	
	cout << "Do you want to train the forest? 1: yes, 0: no\n";
	int answer = 0;
	cin >> answer;
	
	if(answer == 1){
		RandomForest *myForest = new RandomForest(mySamples);	
		myForest->train();
		cout << "Forest trained. " << mySamples.size() <<"\n";
	}
	
	return 0;
}
