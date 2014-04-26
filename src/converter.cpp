/*
   Normalize a csv file.
*/

#include "filter.hpp"
#include "normalization.hpp"

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

ofstream convertedFile;

int convertCSV()
{
	//Read the .csv file containing all samples.    
    ifstream inputFile; //Input stream to read from beginning of file.
    inputFile.open ("TestingSamples.csv", ios::in);
	if (!inputFile.is_open())
    { 
		 printf("ERROR: cannot read file!!!");
		 return -1; 
	}
    
    while (!inputFile.eof())
	{	
		vector<uint8_t> rightFeaturesXY; //Feature vector representing right hand XY movements.
		vector<uint8_t> rightFeaturesZ; //Feature vector representing right hand Z movements.
		vector<uint8_t> leftFeaturesXY; //Feature vector representing left hand XY movements.
		vector<uint8_t> leftFeaturesZ; //Feature vector representing left hand Z movements.
		vector<uint8_t> featureVector;		
		string cell, myString;
        uint8_t num;
               
        //Get feature vector.
        getline(inputFile, myString, ',');
        if(myString.compare("") == 0) break; //Safety measure.
		//cout << "The line is " << line << "\n";
		
        stringstream lineStream(myString); 
        //- '0' is for ASCII to int conversion.
        while (lineStream >> num) featureVector.push_back(num-'0');
        
        getline(inputFile, cell); //Get label.        
        int label = atoi(cell.c_str());        
        
        //Convert the line back to 4 feature vectors.
		for(uint i = 0; i < featureVector.size(); i+=4)
		{
			
			rightFeaturesXY.push_back(featureVector[i]);			
			rightFeaturesZ.push_back(featureVector[i + 1]);
			leftFeaturesXY.push_back(featureVector[i + 2]);
			leftFeaturesZ.push_back(featureVector[i + 3]);
		}
		
		//normalize(&rightFeaturesXY, &rightFeaturesZ, &leftFeaturesXY, &leftFeaturesZ);
		filter(&rightFeaturesXY, &leftFeaturesXY); //Filter MIDDLEs out.		
		
		//Write back vectors to new file.
        for(uint i = 0 ; i < rightFeaturesXY.size(); i++){
			
            convertedFile << (int)rightFeaturesXY[i];     
            convertedFile << (int)rightFeaturesZ[i];     
            convertedFile << (int)leftFeaturesXY[i];     
            convertedFile << (int)leftFeaturesZ[i];           
        }
		
        convertedFile << "," ;
        convertedFile << label << endl;   
       		
    }
   convertedFile.close(); 
   
	return 0;
}

int main(int argc, char **argv) {
	
	//Enable write/append to file.
    convertedFile.open ("converted.csv", ios::out /*| ios::ate*/ | ios::app/* | ios::binary*/) ;
    if (!convertedFile.is_open())
    { 
		 printf("ERROR: cannot read file.");
		 return -1; 
	}    
	
	convertCSV();
	
	return 0;
}
