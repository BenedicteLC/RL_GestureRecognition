//State machine for gesture capture.

#include <GestureState.hpp>

GestureState::GestureState()
{
	canGatherData = false;
	canPublish = false;
	isOngoingGesture = false;
	dataAmount = 0;
	middleCount = 0;
	sound == 0;
	direction = VOID;
}

//Update the state of gesture capture. 
void GestureState::update(DirectionXY newDirection)
{	
	direction = newDirection;
	
	if(isOngoingGesture) //Gesture capture in progress.
	{ 	
		//std::cout << testResults[direction] + "\n";	//TESTING print directions		
		if(canGatherData) //Just wait for first sequence of MIDDLEs to stop.
		{ 
			if(dataAmount >= 180 || isStartOrEnd() ) //End of a gesture.
			{ 					
				isOngoingGesture = false; 
				canGatherData = false;				
				
				if(dataAmount > 30 && dataAmount < 180) //Data vector big enough, but not too long.	
				{ 			
					std::cout << "Gesture captured!\n";	/**TESTING**/
					sound = 2;	
					canPublish = true; //Gesture can be published.
				}
				else
				{ //Gesture had bad length. Discarded.
					
					canPublish = false; //Gesture cannot be published.
					sound = 3;
					std::cout << "Gesture not captured.\n";	/**TESTING**/	
					
				} /**TESTING**/
				
				dataAmount = 0; //Reset counter.
				middleCount = -8; //Add a delay to prevent next gesture to start too fast.
			}			
		}
		else if (direction != MIDDLE) //Wait until the MIDDLEs stop.
		{ 
			canGatherData = true;
		}
	}
	else //Waiting for gesture capture to begin.
	{ 
		if(direction != VOID) //Ignore the VOIDs.
		{ 
			isOngoingGesture = isStartOrEnd();
			
			if(isOngoingGesture)
			{				
				sound = 1;
				std::cout << "Gesture begins!\n";	
			}
		}	
	}	
}

/*The initiation and termination gestures correspond to a series of approx
 * 10 MIDDLE directions. This method determines if a gesture is beginning 
 * or ending.*/
bool GestureState::isStartOrEnd()
{		
	if(direction == MIDDLE)
	{
		middleCount++;
	}
	else if (middleCount > 0) //Do not want negative values.
	{ 
		middleCount--;
	}	
	
	if(middleCount == 10 ) //Gesture started or ended.
	{ 
		middleCount = 0;
		return true;		
	}
	else 
	{
		return false;
	}	
}

void GestureState::reset(){
	canGatherData = false;
	canPublish = false;
	isOngoingGesture = false;
	dataAmount = 0;
	middleCount = 0;
}
