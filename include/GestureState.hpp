#ifndef GestureState_hpp
#define GestureState_hpp

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>

using namespace std;
enum DirectionXY{VOID, RIGHT, UP_RIGHT, UP, UP_LEFT, LEFT, DOWN_LEFT, DOWN, DOWN_RIGHT, MIDDLE};
string testResults[] = {"VOID", "RIGHT", "UP_RIGHT", "UP","UP_LEFT","LEFT","DOWN_LEFT", "DOWN", "DOWN_RIGHT" ,"MIDDLE"};

enum DirectionZ{EMPTY, BACKWARD, CENTER, FORWARD};
string testResultsZ[] = {"VOID", "BACKWARD", "CENTER", "FORWARD"};

class GestureState
{  
	public:
		static bool canGatherData; //Directions can be stored in vector.
		static bool canPublish;	//Vector ready to be published on ROS topic.	
		static int dataAmount; //Counter for vector elements.		
		static int sound; //Status of sound to be played.
		static void update(DirectionXY newDirection); //Update state machine.
		static void reset();
	
    private: 
    	static DirectionXY direction;	
    	static bool isOngoingGesture; //Is gesture being captured.
		static int middleCount; //Counter for consecutive middle directions.
    	
        GestureState();  
        ~GestureState(){/*TODO?*/};
        static bool isStartOrEnd(void);
};

//Defining static variables outside of class.
bool GestureState::canGatherData; 
bool GestureState::canPublish;
int GestureState::dataAmount; 
DirectionXY GestureState::direction;	
bool GestureState::isOngoingGesture;
int GestureState::middleCount;
int GestureState::sound;

#endif      
