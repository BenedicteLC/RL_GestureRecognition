#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <iterator>
#include <unistd.h>

#include <kdl/frames.hpp>
#include <gesture_recognition/Feat.h>
#include <sound_play/sound_play.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include <GestureState.hpp>
#include <normalization.hpp>
#include <filter.hpp>

using std::string;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;
XnUserID user;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

//Right hand. (Left = right for Kinect)
enum DirectionXY directionXYLeft = VOID;
enum DirectionZ directionZLeft = EMPTY;

//Left hand. (Right = left for Kinect)
enum DirectionXY directionXYRight = VOID;
enum DirectionZ directionZRight = EMPTY;

//Past movement of hands. 0:left hand, 1:right hand.
double pastXMov[2] = {0.0};
double pastYMov[2] = {0.0};
double pastZMov[2] = {0.0};



//Create a new user.
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

//Lost a user.
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

//Start user calibration.
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

//Finish user calibration.
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}				
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void getdirection(XnSkeletonJoint const& joint, bool leftHand){
	
	DirectionXY tempDirXY = VOID;
	DirectionZ tempDirZ = EMPTY;
	double pastX, pastY, pastZ;
	
	if(leftHand){
		pastX = pastXMov[0];
		pastY = pastYMov[0];
		pastZ = pastZMov[0];
	}
		
	else{
		pastX = pastXMov[1];
		pastY = pastYMov[1];
		pastZ = pastZMov[1];
	}
	
	XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);
    user = users[0]; //User 1.
    
    if (g_UserGenerator.GetSkeletonCap().IsTracking(user)){			
		XnSkeletonJointPosition joint_position;    
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
		
		//Get the joint confidence value.
		//Low confidence means lost joint.
		XnFloat confidence = joint_position.fConfidence;
		if(confidence <= 0.5){
			tempDirXY = VOID;
			tempDirZ = EMPTY;
			return;
		}
		
		double x = -joint_position.position.X / 1000.0; //Convert to meters.
		double y = joint_position.position.Y / 1000.0;	//Convert to meters.
		double z = -joint_position.position.Z / 1000.0;	//Convert to meters.		
		double noMvmtThres = 0.002; //In meters.
		
		//X-Y plane.
		if(pastX != 0.0 && pastY != 0.0)
		{
			double deltaX = x - pastX;
			double deltaY = y - pastY;			
			double theta = atan(fabs(deltaY) / fabs(deltaX));				
			
			//Convert angle between 0 and 2PI.
			if(deltaX <= 0.0 && deltaY >= 0.0){
				theta = M_PI - theta;
			}	
			else if(deltaX <= 0.0 && deltaY <= 0.0){
				theta = M_PI + theta;
			}	
			else if(deltaX >= 0.0 && deltaY <= 0.0){
				theta = 2*M_PI - theta;
			}
			
			//Not enough movement.
			if(fabs(deltaX) <= noMvmtThres && fabs(deltaY) <= noMvmtThres){
				tempDirXY = MIDDLE;
			}
			//Enough movement. Compute direction of movement.
			else if ((theta >= 0 && theta <= M_PI/8) || (theta >= 15*M_PI/8 && theta <= 2*M_PI)){
				tempDirXY = RIGHT;
			}
			else if (theta >= M_PI/8 && theta <= 3*M_PI/8){
				tempDirXY = UP_RIGHT;
			}
			else if (theta >= 3*M_PI/8 && theta <= 5*M_PI/8){
				tempDirXY = UP;
			}
			else if (theta >= 5*M_PI/8 && theta <= 7*M_PI/8){
				tempDirXY = UP_LEFT;
			}
			else if (theta >= 7*M_PI/8 && theta <= 9*M_PI/8){
				tempDirXY = LEFT;
			}
			else if (theta >= 9*M_PI/8 && theta <= 11*M_PI/8){
				tempDirXY = DOWN_LEFT;
			}
			else if (theta >= 11*M_PI/8 && theta <= 13*M_PI/8){
				tempDirXY = DOWN;
			}		
			else {
				tempDirXY = DOWN_RIGHT;
			}	
		}
		
		//Z plane.
		double zNoMvmtThres = 0.02;
		if(pastZ != 0.0)
		{
			double deltaZ = z - pastZ;	
			
			//Not enough movement.
			if(fabs(deltaZ) <= zNoMvmtThres)
			{
				tempDirZ = CENTER;
			}
			//Enough movement. Compute direction of movement.
			else if (deltaZ > 0.0)
			{
				tempDirZ = FORWARD;
			}		
			else
			{
				tempDirZ = BACKWARD;
			}
		}
		
		//Left hand is right hand to the Kinect.
		if(leftHand){
			directionXYLeft = tempDirXY;
			directionZLeft = tempDirZ;
			pastXMov[0] = x;
			pastYMov[0] = y;
			pastZMov[0] = z;	
		}
		
		else{
			directionXYRight = tempDirXY;
			directionZRight = tempDirZ;
			pastXMov[1] = x;
			pastYMov[1] = y;
			pastZMov[1] = z;		
		}		
	} 
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

//Play a sound depending on the state of the gesture FSM.	
void playSound(sound_play::Sound soundGood, sound_play::Sound soundBad, sound_play::Sound soundStart)
{
	switch(GestureState::sound)
	{
		case 1:
			soundStart.play();	
			GestureState::sound = 0; //Reset not to play the song infinitely.			
			break;
		case 2:
			soundGood.play();
			GestureState::sound = 0; //Reset not to play the song infinitely.	
			break;
		case 3:
			soundBad.play();	
			GestureState::sound = 0; //Reset not to play the song infinitely.	
			break;
		default:
			break;
	}					
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gesture_recognition");
    //Rostopic named "features".
    ros::NodeHandle nh;   

    string configFilename = ros::package::getPath("gesture_recognition") + "/openni_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return 1;
	}

    XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, 
		UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	//*********
	//ROS code.
	//*********
	
	ros::Rate r(30); //Kinect has 30fps. (Note: was 50)
    ros::NodeHandle pnh;
    ros::Publisher vectorPub = nh.advertise<gesture_recognition::Feat>("features", 100);
    gesture_recognition::Feat features;   
    string frame_id("openni_depth_frame");
    pnh.getParam("camera_frame_id", frame_id);    
   
    vector<uint8_t> rightFeaturesXY; //Feature vector representing right hand XY movements.
    vector<uint8_t> rightFeaturesZ; //Feature vector representing right hand Z movements.
    vector<uint8_t> leftFeaturesXY; //Feature vector representing left hand XY movements.
    vector<uint8_t> leftFeaturesZ; //Feature vector representing left hand Z movements.
        
    sound_play::SoundClient sc; //Sound client.
    //Sounds to play.
	sound_play::Sound soundGood = sc.waveSound("../groovy_workspace/gesture_recognition/sounds/good.wav");	
	sound_play::Sound soundBad = sc.waveSound("../groovy_workspace/gesture_recognition/sounds/error.wav");
	sound_play::Sound soundStart = sc.waveSound("../groovy_workspace/gesture_recognition/sounds/start.wav");
     
	while (ros::ok()) {
		
		playSound(soundGood, soundBad, soundStart);		
		
		g_Context.WaitAndUpdateAll();				
	
		getdirection(XN_SKEL_RIGHT_HAND, true); //Get direction of latest left hand movement. REVERSED.	
		getdirection(XN_SKEL_LEFT_HAND, false); //Get direction of latest right hand movement. REVERSED.	
		GestureState::update(directionXYLeft); //Update FSM state.		
		
		//User was lost.
		if(!g_UserGenerator.GetSkeletonCap().IsTracking(user))
		{
			GestureState::reset(); //Reset the FSM.
			//Reset the feature vectors.
			rightFeaturesXY.clear();
			rightFeaturesZ.clear();
			leftFeaturesXY.clear();
			leftFeaturesZ.clear();
			continue;
		}	
		
		if(GestureState::canGatherData) //Ok to collect data: capture gesture.
		{
			if(directionXYLeft != VOID) //Ignore VOIDs.
			{
				GestureState::dataAmount++; //Increment vector length counter.
				//Add directions to feature vectors.
				rightFeaturesXY.push_back((uint8_t)directionXYRight);
				rightFeaturesZ.push_back((uint8_t)directionZRight);				
				leftFeaturesXY.push_back((uint8_t)directionXYLeft);				
				leftFeaturesZ.push_back((uint8_t)directionZLeft);
				//cout << testResultsZ[directionZLeft] << "\n"; /**TESTING**/ 
				//cout << testResults[directionXYRight] << "\n"; /**TESTING**/ 
			}					
		}
		
		//Publish the vector of direction to ROS topic.
		if(GestureState::canPublish){			
		    
            //Padding with 0s.
            int size = rightFeaturesXY.size();
			for (int i = size ; i < 180 ; i++)
			{				
			   rightFeaturesXY.push_back((uint8_t)0);
               rightFeaturesZ.push_back((uint8_t)0);
               leftFeaturesXY.push_back((uint8_t)0);
               leftFeaturesZ.push_back((uint8_t)0);

			}  			  
              normalize(&rightFeaturesXY, &rightFeaturesZ, &leftFeaturesXY, &leftFeaturesZ); //Normalize vector length.
			  filter(&rightFeaturesXY, &leftFeaturesXY); //Filter MIDDLEs out.
			  
            /**TESTING**/
            /*for (int i = 0 ; i < rightFeaturesXY.size() ; i++){ 
                cout << +rightFeaturesXY[i];   
                cout << +rightFeaturesZ[i];  
                cout << +leftFeaturesXY[i];  
                cout << +leftFeaturesZ[i];             
            }
            cout << "\n"; *//**TESTING**/
            
            //Prepare vector to publish.
			features.rightXY = rightFeaturesXY;
			features.rightZ = rightFeaturesZ;
			features.leftXY = leftFeaturesXY;
			features.leftZ = leftFeaturesZ;
			vectorPub.publish(features);            
            
			GestureState::canPublish = false; //Reset.
			rightFeaturesXY.clear(); //Reset the feature vector for the next round.
			rightFeaturesZ.clear(); //Reset the feature vector for the next round.
			leftFeaturesXY.clear(); //Reset the feature vector for the next round.
			leftFeaturesZ.clear(); //Reset the feature vector for the next round.
		}

		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
