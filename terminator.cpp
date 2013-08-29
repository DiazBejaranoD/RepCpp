/* Autonomous robot, Elimia
 * Author. Daniel D�az Bejarano
 */

#include <stdio.h>
#include <libpowerbutton.h>
#include <phidget21.h>
#include <iostream>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <sys/time.h>
#include <vector>
#include <signal.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <sstream>
#include <string>

using namespace cv;

using namespace std;

//GLOBAL VARIABLES
//alarm for obstacles
volatile sig_atomic_t keep_going = 0;
//alarm for frames: coordination between capture_frame AND calculate_matches
volatile sig_atomic_t frame_alert = 0;

bool obstacle =false;
bool moving=false;

bool look_for_box=true;  //Used to synchronize the box_recognition and the image_recognition
bool look_for_image=false; //We set the box_recognition to work first and the image_recognition to wait for a box to recognize.
bool look_for_base=false;
bool box_true_base_false = true;
//global value: 8
int source=8;
int sensor0_value=0;
int sensor1_value=0;
//touch sensor boolean
bool extreme = false;
CPhidgetMotorControlHandle motoControl = 0;
char direction= 'N';
Mat *templa = new Mat[11];
Mat descriptorsTempla;  //Descriptors for template
Mat descriptorsCamera;  //Descriptors for camera
//Mat outCamera;
Mat outTempla;  //For the template
Mat chosen_pic_box, chosen_pic_base;
CvMat* image = 0, *gray=0; //For the camera
//CAPTURE 
CvCapture* capture;
std::vector<KeyPoint> keypointsCamera;
//CENTRAL DISTANCE VARIABLES
float central_x = 0;
float central_y = 0;
bool template_found=false;

//SERVO CONTROLLER
CPhidgetAdvancedServoHandle servo;
vector <KeyPoint> final_items;

int chosen_box,chosen_base;	
  	

int AttachHandler(CPhidgetHandle MC, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (MC, &name);
	CPhidget_getSerialNumber(MC, &serialNo);
	printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

int DetachHandler(CPhidgetHandle MC, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (MC, &name);
	CPhidget_getSerialNumber(MC, &serialNo);
	printf("%s %10d detached!\n", name, serialNo);

	return 0;
}

int ErrorHandler(CPhidgetHandle MC, void *userptr, int ErrorCode, const char *Description)
{
	printf("Error handled. %d - %s\n", ErrorCode, Description);
	return 0;
}



int InputChangeHandlerMo(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State)
{
	//printf("Input %d > State: %d\n", Index, State);
	return 0;
}

int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
	//printf("Motor %d > Current Speed: %f\n", Index, Value);
	return 0;
}

int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
	//printf("Motor: %d > Current Draw: %f\n", Index, Value);
	return 0;
}




int display_MO_properties(CPhidgetMotorControlHandle phid)
{
	int serialNo, version, numInputs, numMotors;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);
	
	CPhidgetMotorControl_getInputCount(phid, &numInputs);
	CPhidgetMotorControl_getMotorCount(phid, &numMotors);

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("# Inputs: %d\n# Motors: %d\n", numInputs, numMotors);

	return 0;
}

//SERVO CONTROLLING
int PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value)
{
	printf("Motor: %d > Current Position: %f\n", Index, Value);
	return 0;
}

int display_SERVO_properties(CPhidgetAdvancedServoHandle phid)
{
	int serialNo, version, numMotors;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetAdvancedServo_getMotorCount (phid, &numMotors);

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n# Motors: %d\n", serialNo, version, numMotors);

	return 0;
}


/*************************InterfaceKit functions***************************///(sensor)

//callback that will run if an output changes.
//Index - Index of the output that generated the event, State - boolean (0 or 1) representing the output state (on or off)
int OutputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	printf("Digital Output: %d > State: %d\n", Index, State);
	return 0;
}


int go_backwards(){

	direction='B';
	CPhidgetMotorControl_setAcceleration (motoControl, 0, 100.00);
	CPhidgetMotorControl_setVelocity (motoControl, 0, 100.00);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, 100.00);
	CPhidgetMotorControl_setVelocity (motoControl, 1, 100.00);
}
int go_forward(){
	direction='F';
	CPhidgetMotorControl_setAcceleration (motoControl, 0, -80.00);
	CPhidgetMotorControl_setVelocity (motoControl, 0, -40.00);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, -80.00);
	CPhidgetMotorControl_setVelocity (motoControl, 1, -40.00);
	
}
int turn_right(){
	direction='R';
	//Stop the motor by decreasing speed to 0;
	CPhidgetMotorControl_setVelocity (motoControl, 0, 100.00);
	CPhidgetMotorControl_setAcceleration (motoControl, 0, 100.00);

	CPhidgetMotorControl_setVelocity (motoControl, 1, -100.00);
	CPhidgetMotorControl_setAcceleration (motoControl, 1, -100.00);
}

int turn_left(){
	direction='L';
	//Stop the motor by decreasing speed to 0;
	CPhidgetMotorControl_setVelocity (motoControl, 0, -100.00);
	CPhidgetMotorControl_setAcceleration (motoControl, 0, -100.00);

	CPhidgetMotorControl_setVelocity (motoControl, 1, 100.00);
	CPhidgetMotorControl_setAcceleration (motoControl, 1, 100.00);
}


void go_forward_slowly(){
			direction='F';
			CPhidgetMotorControl_setAcceleration (motoControl, 0, -40.00);
			CPhidgetMotorControl_setVelocity (motoControl, 0, -20.00);
			CPhidgetMotorControl_setAcceleration (motoControl, 1, -40.00);
			CPhidgetMotorControl_setVelocity (motoControl, 1, -20.00);
}


void turn_left_slowly(){
			direction='L';
			
			CPhidgetMotorControl_setAcceleration (motoControl, 0, -50.00);
			CPhidgetMotorControl_setVelocity (motoControl, 0, -25.00);			
			CPhidgetMotorControl_setAcceleration (motoControl, 1, 50.00);
			CPhidgetMotorControl_setVelocity (motoControl, 1, 25.00);


}

void turn_right_slowly(){
			direction='R';
			
			CPhidgetMotorControl_setAcceleration (motoControl, 1, -50.00);
			CPhidgetMotorControl_setVelocity (motoControl, 1, -25.00);
			
			CPhidgetMotorControl_setAcceleration (motoControl, 0, 50.00);
			CPhidgetMotorControl_setVelocity (motoControl, 0, 25.00);

}

void go_backwards_slowly(){

			direction='B';
			CPhidgetMotorControl_setAcceleration (motoControl, 0, 80.00);
			CPhidgetMotorControl_setVelocity (motoControl, 0, 30.00);
			CPhidgetMotorControl_setAcceleration (motoControl, 1, 80.00);
			CPhidgetMotorControl_setVelocity (motoControl, 1, 30.00);
}

void stop(){
			direction='S';
			//STOP
			CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
			CPhidgetMotorControl_setVelocity (motoControl, 1, 0.00);
			CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
			CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);

}



int wait(unsigned long milisec)
{
    struct timespec req={0};
    time_t sec=(int)(milisec/1000);
    milisec=milisec-(sec*1000);
    req.tv_sec=sec;
    req.tv_nsec=milisec*1000000L;
    while(nanosleep(&req,&req)==-1)
         continue;
    return 1;
}
/* The signal handler just clears the flag and re-enables itself. */
void catch_alarm (int sig){
       printf("Alarm run out!\n");
       keep_going = 0;
       signal (sig, catch_alarm);
}

void set_alarm(int time){
	
	//Set an alarm to go off in a little while.
	printf("Alarm set!\n");
	// Establish a handler for SIGALRM signals. 
	keep_going=1;
	signal(SIGALRM, catch_alarm);
	alarm(time);  
	  
	//Check the flag once in a while to see when to quit. 
	//while (keep_going)
		//printf("Timer Waiting\n");				
	
}

void close_grabber(){
	//Step 1: Position 40.00 - also engage servo
	printf("Move to position 100.00 and engage. Press any key to Continue\n");


	CPhidgetAdvancedServo_setPosition (servo, 0, 120.00);
	CPhidgetAdvancedServo_setEngaged(servo, 0, 1);

	//Step 4: Position 150.00
	printf("Move to position 150.00. Press any key to Continue\n");
}

//ALARM METHODS FOR OBJECT GRABBING
//frame_alert
void catch_alarm2 (int sig){

       printf("Frame alarm run out!\n");
       frame_alert = 0;
       signal (sig, catch_alarm2);
}

void set_alarm2 (int time){
	
	//Set an alarm to go off in a little while.
	printf("Frame alarm!\n");
	// Establish a handler for SIGALRM signals. 
	frame_alert=1;
	signal(SIGALRM, catch_alarm2);
	alarm(time);

}

void turn_around(){

		//check sensor values to decide which turn is better
		if(sensor0_value>sensor1_value)
			if (direction != 'L')			
				turn_left_slowly();
		if(sensor0_value<=sensor1_value)
			if (direction != 'R')			
				turn_right_slowly();
		wait(2000);
		if (direction != 'F')
			go_forward();
		extreme=false;
}


void obstacle_found(){
	
		moving=true;
		if (direction != 'B')	
			go_backwards();	
		set_alarm(1);
		//while alarm is on, check if EXTREME value is on(if touch is activated) 					
		while (keep_going && !extreme);
		if(extreme){
			printf("Sonar interrupted!!");
			obstacle=false;
			return;
		}
		//action after alarm
		//check sensor values to decide which turn is better
		if(sensor0_value>sensor1_value)
			if (direction != 'L')			
				turn_left();
		if(sensor0_value<=sensor1_value)
			if (direction != 'R')			
				turn_right();
		set_alarm(1);
		//while alarm is on, check if EXTREME value is on(if touch is activated) 					
		while (keep_going && !extreme);
		if(extreme){
			printf("Sonar interrupted!!");
			obstacle=false;
			return;
		}
		if (direction != 'F')
			go_forward();
		obstacle=false;
		printf("Sonar finished!");
		moving=false;
}

//callback that will run if the sensor value changes by more than the OnSensorChange trigger.
//Index - Index of the sensor that generated the event, Value - the sensor read value



//DIGITAL INPUTS
int InputChangeHandlerKi(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	if (!extreme){

		switch(Index)
		{	//Front sensor
			case 0:
			case 1:
				//we have touch
				if(State == 1)
				{
					//set extreme boolean to true
					extreme=true;
					printf("OBSTACLE FOUND WITH TOUCH SENSOR\n\n");
					//go back
					if (direction != 'B')	
						go_backwards();	
					set_alarm(2);
					//do nothing while alarm is on			
					while (keep_going);
					//turn around
					printf("im TURNING\n\n\n\n\n\n\n");
					turn_around();
				}			
				break;
		}
		//printf("Digital Input: %d > State: %d\n", Index, State);
	}
	return 0;
}


//ANALOG INPUTS
int SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
{
	moving=true;
	//WE FOUND TEMPLATE=> IGNORE SENSORS
	if (!template_found){

	      	if(!obstacle){      //
				//printf("\n\n\nSENSORS WORKING\n\n\n\n");
				//printf("Sensor: %d > Value: %d\n", Index, Value);
				//int sensorValue=0;
				//CPhidgetInterfaceKit_getSensorValue(ifKit,0,&sensorValue);
				switch(Index)
				{
				case 0:
					printf("\n");
					//printf("RIGHT IR WORKING\n\n");
					if (source == 0  || source == 8 ){
						if(Value < 300)
						{
							if (direction != 'F')
								go_forward();
							source=8; 						
		
						}	
						else 
						{
							if(Value >= 300 && Value < 400){
								//printf("Sensor: %d between 300 and 400\n", Index);
								source = 0;
								if (direction != 'L')			
									turn_left();			
							}
							else
							{
								//printf("Sensor: %d over 400\n", Index);			
								if (direction != 'B')	
									go_backwards();		
							}
						}
				}
					sensor0_value=Value;
					break;

				case 1:	
				printf("\n");
				//printf("LEFT IR WORKING\n\n");
				if (source == 1 || source == 8 ){
					if(Value < 300)
					{
						if (direction != 'F')
							go_forward();		
						source=8;
					}	
					else 
					{
						if(Value >= 300 && Value < 400){
							//printf("Sensor: %d between 300 and 400\n", Index);
							source = 1;
							if (direction != 'R')			
								turn_right();			
						}
						else
						{
							//printf("Sensor: %d over 400\n", Index);		
							if (direction != 'B')	
								go_backwards();		
						}
					}
				}
				sensor1_value=Value;
				break;	
				
				case 2:
					if (source == 8 ){
						if(Value <= 18){
							printf("\n");
						//	printf("SONAR WORKING\n\n");
							//printf("Sensor: %d > Value: %d\n", Index, Value);
							obstacle=true;
							obstacle_found();
						}	
					}	
					break;
			
				}
		}//obstacle
	moving=false;
	}//template_found
	return 0;
}
int display_IK_properties(CPhidgetInterfaceKitHandle phid)
{
	int serialNo, version, numInputs, numOutputs, numSensors, triggerVal, ratiometric, i;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetInterfaceKit_getInputCount(phid, &numInputs);
	CPhidgetInterfaceKit_getOutputCount(phid, &numOutputs);
	CPhidgetInterfaceKit_getSensorCount(phid, &numSensors);
	CPhidgetInterfaceKit_getRatiometric(phid, &ratiometric);

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("# Digital Inputs: %d\n# Digital Outputs: %d\n", numInputs, numOutputs);
	printf("# Sensors: %d\n", numSensors);
	printf("Ratiometric: %d\n", ratiometric);

	for(i = 0; i < numSensors; i++)
	{
		CPhidgetInterfaceKit_getSensorChangeTrigger (phid, i, &triggerVal);

		printf("Sensor#: %d > Sensitivity Trigger: %d\n", i, triggerVal);
	}

	return 0;
}

/*************************InterfaceKit functions end***************************/ //(Sensor)


int button_pressed_method(){
	
	printf("Button pressed %i times.\n",power_button_get_value());
    	while(power_button_get_value()<1)
    	{
       	 	sleep(1);
        	//printf("Button pressed %i times.\n",power_button_get_value());
   	 }
    	power_button_reset();
	return 0;
}

void open_grabber(){
	CPhidgetAdvancedServo_setPosition (servo, 0, 210.00);
	CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
	set_alarm(1);
	while (keep_going);
	//Step 7: Disengage
	CPhidgetAdvancedServo_setEngaged(servo, 0, 0);

}



//CHANGE PRIORITY OF TOUCH OVER IR!!!!!!!!
//CHANGE PRIORITY OF TOUCH OVER IR!!!!!!!!
//CHANGE PRIORITY OF TOUCH OVER IR!!!!!!!!

void *motorIK_method(void *threadid)
{	
	int result, i;
	const char *err;
	int numSensors;
	

	//Declare a motor control handle
	motoControl = 0;
	//create the motor control object
	CPhidgetMotorControl_create(&motoControl);
	
	//Declare an InterfaceKit handle
	CPhidgetInterfaceKitHandle ifKit = 0;
	//create the InterfaceKit object
	CPhidgetInterfaceKit_create(&ifKit);


	//****************************************SERVO CONTROL***********************************//
	
	//int result;
	double curr_pos;
	//const char *err;
	double minAccel, maxVel;

	//Declare an advanced servo handle
	servo = 0;

	//create the advanced servo object
	CPhidgetAdvancedServo_create(&servo);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)servo, ErrorHandler, NULL);

	//Registers a callback that will run when the motor position is changed.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo, PositionChangeHandler, NULL);

	//open the device for connections
	CPhidget_open((CPhidgetHandle)servo, -1);

	//get the program to wait for an advanced servo device to be attached
	printf("Waiting for Phidget to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)servo, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	//Display the properties of the attached device
	display_SERVO_properties(servo);

	//****************************************SERVO CONTROL***********************************//

	
	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.

	//MotorControl
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)motoControl, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)motoControl, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)motoControl, ErrorHandler, NULL);
	
	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)ifKit, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);


	//Registers a callback that will run if an input changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnInputChange_Handler (motoControl, InputChangeHandlerMo, NULL);

	//Registers a callback that will run if a motor changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnVelocityChange_Handler (motoControl, VelocityChangeHandler, NULL);

	//Registers a callback that will run if the current draw changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnCurrentChange_Handler (motoControl, CurrentChangeHandler, NULL);

	//Registers a callback that will run if an input changes.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetInterfaceKit_set_OnInputChange_Handler (ifKit, InputChangeHandlerKi, NULL);

	//Registers a callback that will run if the sensor value changes by more than the OnSensorChange trig-ger.
	//Requires the handle for the IntefaceKit, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetInterfaceKit_set_OnSensorChange_Handler (ifKit, SensorChangeHandler, NULL);

	//Registers a callback that will run if an output changes.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetInterfaceKit_set_OnOutputChange_Handler (ifKit, OutputChangeHandler, NULL);

	//open the motor control for device connections

	CPhidget_open((CPhidgetHandle)motoControl, -1);
	//get the program to wait for a motor control device to be attached

	printf("Waiting for MotorControl to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)motoControl, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	
	//open the interfacekit for device connections
	CPhidget_open((CPhidgetHandle)ifKit, -1);

	//get the program to wait for an interface kit device to be attached
	printf("Waiting for interface kit to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	//Display the properties of the attached motor control device
	display_MO_properties(motoControl);
	display_IK_properties(ifKit);

	//read motor control event data
	printf("Reading.....\n");

	//*******************************************************************//

	
    	power_button_reset();	
	printf("Button pressed %i times.\n",power_button_get_value());
    	while(power_button_get_value()<1)
    	{
       	 	sleep(1);
        	//printf("Button pressed %i times.\n",power_button_get_value());
   	 }
    	power_button_reset();	
	
	go_forward();
	
printf("Button pressed %i times.\n",power_button_get_value());
    	while(power_button_get_value()<1)
    	{
       	 	sleep(1);
        	//printf("Button pressed %i times.\n",power_button_get_value());
   	 }
    	power_button_reset();

	//Stop the motor by decreasing speed to 0;
	CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
	CPhidgetMotorControl_setVelocity (motoControl, 1, 0.00);
	CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
	CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);
	
	//printf("SPEED OF 0 SET to 0\n\n");	
	//Step 4: Close Session
	while (moving == true){
		CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
		CPhidgetMotorControl_setVelocity (motoControl, 1, 0.00);
		CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
		CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);	
	}
	open_grabber();
	
		
	CPhidget_close((CPhidgetHandle)motoControl);
	CPhidget_delete((CPhidgetHandle)motoControl);
	printf("Sensor closing");
	CPhidget_close((CPhidgetHandle)ifKit);
	CPhidget_delete((CPhidgetHandle)ifKit);
	printf("Sensor closed");
			
	//CLOSE SERVO 
	
	printf("Servo closing");
	CPhidget_close((CPhidgetHandle)servo);
	CPhidget_delete((CPhidgetHandle)servo);
	printf("Servo closed");
	
	//all done, exit
	pthread_exit(NULL);
	return 0;
}

int inform_templates(){

	//Alphabetic ordering in the array of templas
	templa[0] = imread("images/celebes.png", CV_LOAD_IMAGE_GRAYSCALE );
	if( !templa[0].data || !templa[0].data )
 	 	{ std::cout<< " --(!) Error reading image cellebes" << std::endl; return -1; }
	templa[1] = imread("images/ferrari.png", CV_LOAD_IMAGE_GRAYSCALE );
	if( !templa[1].data || !templa[1].data )
 	 	{ std::cout<< " --(!) Error reading image ferrari" << std::endl; return -1; }
	templa[2] = imread("images/fry.png", CV_LOAD_IMAGE_GRAYSCALE );
	if( !templa[2].data || !templa[2].data )
 	 	{ std::cout<< " --(!) Error reading image fry" << std::endl; return -1; }
	templa[3] = imread("images/iron.png", CV_LOAD_IMAGE_GRAYSCALE );
	if( !templa[3].data || !templa[3].data )
 	 	{ std::cout<< " --(!) Error reading image iron" << std::endl; return -1; }
	templa[4] = imread("images/mario.png", CV_LOAD_IMAGE_GRAYSCALE );
	if( !templa[4].data || !templa[4].data )
 	 	{ std::cout<< " --(!) Error reading image mario" << std::endl; return -1; }
	templa[5] = imread("images/starry.png", CV_LOAD_IMAGE_GRAYSCALE );
	if( !templa[5].data || !templa[5].data )
 	 	{ std::cout<< " --(!) Error reading image starry" << std::endl; return -1; }
	templa[6] = imread("images/terminator.png", CV_LOAD_IMAGE_GRAYSCALE );
	if( !templa[6].data || !templa[6].data )
 	 	{ std::cout<< " --(!) Error reading image terminator" << std::endl; return -1; }
	templa[7] = imread("images/thor.png", CV_LOAD_IMAGE_GRAYSCALE );
	if( !templa[7].data || !templa[7].data )
 	 	{ std::cout<< " --(!) Error reading image thor " << std::endl; return -1; }
	templa[8] = imread("images/walle.png", CV_LOAD_IMAGE_GRAYSCALE );
	if( !templa[8].data || !templa[8].data )
 	 	{ std::cout<< " --(!) Error reading image walle" << std::endl; return -1; }
	templa[9] = imread("images/base1small.png", CV_LOAD_IMAGE_GRAYSCALE );
	if( !templa[9].data || !templa[9].data )
 	 	{ std::cout<< " --(!) Error reading image strangebase" << std::endl; return -1; }
 	
 	templa[10] = imread("images/base2small.png", CV_LOAD_IMAGE_GRAYSCALE );
	if( !templa[10].data || !templa[10].data )
 	 	{ std::cout<< " --(!) Error reading image queen" << std::endl; return -1; }

	return 0;

}

void *PrintHello(void *threadid)
{
   long tid;
   tid = (long)threadid;
   cout << "Hello World! Thread ID, " << tid << endl;
   pthread_exit(NULL);
}

void inform_template_descriptors_box(){
	
	
	std::vector<KeyPoint> keypointsTempla; 
	//HESSIANS
	SurfFeatureDetector surf(450.);
	SurfDescriptorExtractor surfDesc;
	surf.detect(chosen_pic_box, keypointsTempla);
	//drawKeypoints(chosen_pic, keypointsTempla, outTempla, Scalar(255,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	//imshow("SURF detector Templa", outTempla);
	
	surfDesc.compute(chosen_pic_box, keypointsTempla, descriptorsTempla);	
	printf("Template: Descriptor cols: %i \n Descriptor rows: %i\n",descriptorsTempla.cols,descriptorsTempla.rows);	
	//imshow("SURF detector Templa", outTempla);
	
}

void inform_template_descriptors_base(){
	
	
	std::vector<KeyPoint> keypointsTempla; 
	//HESSIANS
	SurfFeatureDetector surf(450.);
	SurfDescriptorExtractor surfDesc;
	surf.detect(chosen_pic_base, keypointsTempla);
	//drawKeypoints(chosen_pic, keypointsTempla, outTempla, Scalar(255,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	//imshow("SURF detector Templa", outTempla);
	
	surfDesc.compute(chosen_pic_base, keypointsTempla, descriptorsTempla);	
	printf("Template: Descriptor cols: %i \n Descriptor rows: %i\n",descriptorsTempla.cols,descriptorsTempla.rows);	
	//imshow("SURF detector Templa", outTempla);
	
}


void grab_object(float x){
	//CHECK RANGE OF X
	if (x> 400){
		turn_right_slowly();
		wait(200);
	}
			
	else if (x<240){
		turn_left_slowly();
		wait(200);
	}
		
	go_forward();
	wait(5000);
	close_grabber();
	wait(1200);
	turn_around();

}

void release_object(float x){
	//CHECK RANGE OF X
	if (x> 400){
		turn_right_slowly();
		wait(200);
	}
			
	else if (x<240){
		turn_left_slowly();
		wait(200);
	}

	go_forward();
	wait(4800);
	CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
	CPhidgetMotorControl_setVelocity (motoControl, 1, 0.00);
	CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
	CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);
	open_grabber();
	wait(2000);
	go_backwards();
	wait(2000);
	turn_around();

}


void *calculate_matches(void *threadid){	//FERRARI CODE

	//COLUMNS: ATTRIBUTES
	//column 0:	(float)distance_threshold 
	//cloumn 1:	(int)neighbours_threshold
	//column 2: 	(int)match_threshold 
	

	
	int i,j,k;

	CvMat* image = 0, *gray=0; //For the camera
	Mat descriptorsCamera;  //Descriptors for camera
	Mat outCamera;

	int key = 0;
	int sum;
	static CvScalar red_color[] ={0,0,255};	
	int times_check=0;
	int times_verified=0;
	
	//set different hessian for the surfs
	//BIGGER VALUES FOR 
	SurfFeatureDetector surf_frame(750.);//750
	SurfDescriptorExtractor surfDesc_frame;	
	int best_match_threshold; 
	float first_distance;	
	//sleep(3);
	float box_x=0;
	float box_y=0;
	float base_x=0;
	float base_y=0;

	while( key!='q' )//key != 'q'
	{
		while(!look_for_image); //Wait for the box_detection thread
		//DISTINGUISH BETWEEN BOX TEMPLATE AND BASE TEMPLATE
		//box_true_base_false 
		if (box_true_base_false){
			inform_template_descriptors_box();
			
			//INFORM THRESHOLDS
			  //TEMPLATE CASING
		          switch (chosen_box){
				//fry
				case 2:
					first_distance=0.32;
					best_match_threshold = 15; 
					break;
				//iron
				case 3:
					first_distance=0.32;
					best_match_threshold = 22; 	
					break;
				//starry
				case 5:
					first_distance=0.34;
					best_match_threshold = 22; 	
					break;
				default:
					first_distance=0.25;	
					best_match_threshold = 22; 				
					break;
		        }	
			
		}
		else{
			//INFORM THRESHOLDS
			inform_template_descriptors_base();
			first_distance=0.275;
			best_match_threshold = 14; 
		}
		printf("Distance criterion: %f, Match threashold: %d\n\n",first_distance,best_match_threshold);
		
		printf("IM IN IMAGE DETECTION THREADDDD\n");
		//DIFFICULT FRAMES COULD RESULT IN FAIL	
		int firstFrame = gray == 0;
		//was declared inside
		IplImage* frame = cvQueryFrame(capture);
		
		if(!frame){
			break;
			printf("Frame error\n");
		}
		if(!gray)
		{
			//CHANGED SIZE OF IMAGE
			image = cvCreateMat(frame->height, frame->width, CV_8UC1);
		}
		
		//Convert the RGB image obtained from camera into Grayscale
		cvCvtColor(frame, image, CV_BGR2GRAY);
		
		//Define sequence for storing surf keypoints and descriptors
		std::vector<KeyPoint> keypointsCamera;
		Mat cameraDescriptors;
		
		//Extract SURF points by initializing parameters
		//SECOND PARAMETER IS params.extended
		// 0 means basic descriptors (64 elements each),
                // 1 means extended descriptors (128 elements each)
		surf_frame.detect(image, keypointsCamera);
		
		//drawKeypoints(image, keypointsCamera, outCamera, Scalar(255,255,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		//namedWindow("SURF detector camera");
		//imshow("SURF detector camera", outCamera);

		surfDesc_frame.compute(image,keypointsCamera,descriptorsCamera);
	
		
		//printf("CAMERA DESCRIPTORS");
		printf("CAMERA: Descriptor cols: %i \n Descriptor rows: %i\n\n",descriptorsCamera.cols,descriptorsCamera.rows);

		//ARRAY FOR MATCHES: length is number of descriptors in Template (one descriptor per row)
		
		int size_of_matches = descriptorsTempla.rows;
		int matches[size_of_matches];

		
		//column 1:  1st neighbor distance	
		//column 2:  2nd neighbor distance
		//column 3:  camera descriptor
		//column 4 : index of descriptor in template
		
		double distance_of_match[size_of_matches][4];

		for(i=0;i<size_of_matches;i++)
		{
			float distance = 10000; //Initial distance, large one
			float distance_of_second = 10000;
			int matching_descriptor=0;
			
			for(j=0;j<descriptorsCamera.rows;j++)
			{
				//COMPARE TEMPLA COLUMN i with all j COLUMNS of CAMERA
				Mat t = descriptorsTempla.row(i);
				Mat c = descriptorsCamera.row(j);					
				// COMPUTE EUCLIDIAN DISTANCE 
				float d = 0;
				for(int k=0; k < 64; k++){ //We calculate the distance of the 64 elements				
					//IF YOU EXTRACT POINTS AS FLOATS RATHER THAN DOUBLES THERE IS BIG DIFFERENCE	
					float t1 =(float)t.at<float>(k);
					float c1 = (float)c.at<float>(k);
					//printf("Elements(double) %lf %lf \n\n",t1,c1);
					d += pow(t1-c1,2);//2
					//printf("difference %i square %i \n\n",(t1-c1),pow(t1-c1,2));
					//d += sqrt(t.at<int>(0,k)-c.at<int>(0,k));
				}
				float calculation = (float)sqrt(d);//(double)sqrt(d)
				//printf("Calculated distance for row%i: %d \n\n",j,calculation);
				 
				if (calculation < distance){
					matching_descriptor = j;
					float temp= distance;					
					distance = calculation;	//Update the distance
					distance_of_second = temp;
				}					
			}
			//printf("Calculated best distance for descriptor %d: %f \n\n",i,distance);

			//SET BEST IN ARRAY
			matches[i] = matching_descriptor; 
			//Set distance, distance of second neighbor, the descriptor in the camera and descriptor in template
			distance_of_match[i][0]= distance;
			distance_of_match[i][1]= distance_of_second;
			distance_of_match[i][2]= matching_descriptor;	
			distance_of_match[i][3]= i;
			//printf("Before sorting!");
			//printf("Best distance for descriptor %d: %f  Matching with: %d\n",i,distance,matching_descriptor);	
		}//if_matches
		//printf("Before sorting!");
		
		//CHOOSE BEST DESCRIPTORS => THOSE WITH LESS DISTANCE e.g 0.7
		//DRAW KEYPOINTS WHOSE DESCRIPTORS MATCH
		vector<KeyPoint> selected_points;
		//column 1:  1st neighbor distance	
		//column 2:  2nd neighbor distance
		//column 3:  camera descriptor
		//column 4 : index of descriptor of template
		
		printf("Before Filtering!");
		
		for (int m=0;m< selected_points.size();m++){
				KeyPoint n_point = (KeyPoint)selected_points[m];							
				//(float)n_point.pt.x;
				//(float)n_point.pt.y;
				printf("Element at %d: x:%f y:%f \n",m,(float)n_point.pt.x,(float)n_point.pt.y); 
			
		}
			
		int counter=0;
		for (int m=0;m< size_of_matches;m++){	//size_of_matches
		
			bool discard = false;
			//printf("matching point index: %d ", m);
			//THIS COULD BE IT
			int matching = (int)distance_of_match[m][2]; //Column with the matching descriptor
			//SET THRESHOLD FOR DISTANCE
			float temp_dist=(float)distance_of_match[m][0]; //Distance first neighbor
			float temp_second=(float)distance_of_match[m][1]; //Distance second neighbor
			
			//printf("1ST NEIGHBOR: %f  2ND NEIGHBOR: %f\n", temp_dist, temp_second);
			float ratio = temp_dist/temp_second;
			
			//SELECT BEST THRESHOLD 
			
			if (temp_dist > first_distance)//0.37
				discard = true;
			
			//RATIO OF 1ST OVER 2ND CLOSEST NEIGHBOR
			if (ratio > 0.8)//0.8
				discard = true;
				
			KeyPoint temp = keypointsCamera[matching];
			
			if (!discard){
				counter++;
				//printf("Added distance %f\n\n", temp_dist);
				selected_points.push_back(temp);
				
			}
		}
		
		printf("\nTotal features selected: %d\n",counter);
		//PROCEED WHEN selected_points IS NOT NULL
		if(!selected_points.empty()){
			
			float x_difference=0;
			float y_difference=0;
			float sum=0;;
			float final=0;
			vector<float> distances_among_points;
			
			int size = selected_points.size();
			int counting=0;
			vector<float> distances_to_center;
			
			//CENTRAL X,Y
			float x_coord=0,y_coord=0;
			float central_x,central_y;
			for (int n=0;n<size;n++){
			
				KeyPoint n_point = (KeyPoint)selected_points[n];
				x_coord +=(float)n_point.pt.x;
				y_coord +=(float)n_point.pt.y;
			}
			central_x = x_coord / size;
			central_y = y_coord / size;

			//BASE CENTRAL COORDINATES
			base_x = central_x;
			base_y = central_y; 
	
			//printf("Central x: %f Central y: %f\n",central_x,central_y);		
			
			//CALCULATE DISTANCE TO CENTER
			//distances_to_center
			for (int n=0;n<size;n++){//size
 
						   KeyPoint n_point = (KeyPoint)selected_points[n];							
						   x_difference=(float)central_x-(float)n_point.pt.x;
						   y_difference=(float)central_y-(float)n_point.pt.y;
						   sum = pow(x_difference,2)+pow(y_difference,2);
						   final = (float)sqrt(sum);
						   distances_to_center.push_back(final);
         					   //printf("Distance added: %f\n",(float)final);
         		}
			
	
			//SORT TO GET MEAN
			int size_b= distances_to_center.size();
			//printf("Total Elements: %d\n",(int)size_b);
			float dist_m;
			float dist_l;
			for (int m=0;m< size_b;m++){
			
				for(int l=m+1;l<size_b;l++){
					
					dist_m =(float)distances_to_center[m];
					dist_l =(float)distances_to_center[l];
					
					if (dist_m > dist_l){
					
						//ERASE FIRST
						distances_to_center.erase (distances_to_center.begin()+m);
						//WE HAVE NEW M NOW
						//inserting new elements before the element at position m						
						distances_to_center.insert(distances_to_center.begin()+ m,dist_l);
						
						distances_to_center.erase (distances_to_center.begin()+l);
						distances_to_center.insert(distances_to_center.begin()+ l, dist_m);
						  
					}			
				}
			}
			
			size = selected_points.size();
			vector <KeyPoint> median_first;


			float median = 1.2*(float)distances_to_center[(int)size_b/2];
			//printf("Median(*1.2): %f\n",(float)median);
			//printf("Central x: %f Central y: %f\n",central_x,central_y);
			
			
			//ELIMINATE POINTS THAT HAVE BIGGER DISTANCE FROM CENTER THAN THE MEDIAN OF ABOVE VECTOR
			
			for (int n=0;n<size;n++){//size
 
						   KeyPoint n_point = (KeyPoint)selected_points[n];							
						   x_difference= central_x-n_point.pt.x;
						   y_difference= central_y-n_point.pt.y;
						   sum = pow(x_difference,2)+pow(y_difference,2);
						   final = (float)sqrt(sum);
						   if (final< median){
						   	median_first.push_back(selected_points[n]);
						   	//printf("Point added!\n\n");
						   }	

         		}
			
			//CALCULATE NEW CENTERcapture
			
			int vector_size = median_first.size();
			x_coord =0;
			y_coord =0;
			for (int n=0;n<vector_size;n++){
			
				KeyPoint n_point = (KeyPoint)median_first[n];
				x_coord +=(float)n_point.pt.x;
				y_coord +=(float)n_point.pt.y;
			}
			central_x = x_coord / vector_size;
			central_y = y_coord / vector_size; 	

			//CENTRAL FOR BOX
			box_x = central_x;
			box_y = central_y;

			printf("Median first size: %d\n",median_first.size());
			printf("New Central x: %f\n Central y: %f\n",central_x,central_y);
			
		
			//LATEST SET IS median_first
			//central_x,central_y
			
			//SECOND ROUND OF MEDIAN
			//CALCULATE DISTANCE TO CENTER			
			distances_to_center.clear();
			//printf("Distances to center size: %d\n",distances_to_center.size());			
			vector_size = median_first.size();
			
			//distances_to_center
			for (int n=0;n<vector_size;n++){//size
 
						   KeyPoint n_point = (KeyPoint)median_first[n];							
						   x_difference =(float)central_x-(float)n_point.pt.x;
						   y_difference = (float)central_y-(float)n_point.pt.y;
						   sum = pow(x_difference,2)+pow(y_difference,2);
						   final = (float)sqrt(sum);
						   //printf("Point checking (x): %f\n(y): %f\n",(float)n_point.pt.x,(float)n_point.pt.y);
						   //printf("distance to center: %f\n",final);
						   distances_to_center.push_back(final);
         					   //printf("Distance added: %f\n",(float)final);
         		}
         		
         		
         		//SORT VECTOR FIRST
         		//SORT TO GET SECOND MEAN
			size_b= distances_to_center.size();
			//printf("Total Elements: %d\n",(int)size_b);
			dist_m=0;
			dist_l=0;

			for (int m=0;m< size_b;m++){
			
				for(int l=m+1;l<size_b;l++){
					
					dist_m =(float)distances_to_center[m];
					dist_l =(float)distances_to_center[l];
					
					if (dist_m > dist_l){
					
						//ERASE FIRST
						distances_to_center.erase (distances_to_center.begin()+m);
						//WE HAVE NEW M NOW
						//inserting new elements before the element at position m						
						distances_to_center.insert(distances_to_center.begin()+ m,dist_l);
						
						distances_to_center.erase (distances_to_center.begin()+l);
						distances_to_center.insert(distances_to_center.begin()+ l, dist_m);
						  
					}			
				}
			}
			/*
			//PRINT VECTOR
			for (int m=0;m< size_b;m++){
				printf("After Sort : Element at %d: %f\n",m,(float)distances_to_center[m]); 
			
			}*/
         		
         		float second_median = 1*(float)distances_to_center[(int)size_b/2];
			printf("Second Median: %f\n",(float)second_median);
					
		
		//DRAW SELECTED POINTS
		Mat center_mat; 
		//DRAW CENTER OF POINTS
		vector <KeyPoint> central;
		KeyPoint center = (KeyPoint)selected_points[0];
		center.pt.x = central_x;
		center.pt.y = central_y;
		central.push_back(center);		
		
		//drawKeypoints(image,selected_points,center_mat,Scalar(0,255,0),0);//central
		
		Mat best_matches;
		
		//drawKeypoints(image,median_first,best_matches,Scalar(100,220,245),0);//DrawMatchesFlags::DRAW_RICH_KEYPOINTS,//median_first
		//Mat total;
		//cv::hconcat(best_matches, center_mat, total);
		
		//namedWindow("Matches");
		//imshow("Matches", best_matches);
		
		//TAKE A DECISION BASED ON NUMBER OF BEST_MATCHES => SET A THRESHOLD

		//set threshold based on base or box
		int threshold;
		float distance_criterion;
		
		if (box_true_base_false){//for box is the median vector size	
			printf("Look for box!\n");
			threshold=median_first.size();
			distance_criterion = 45;
		}
		else {//for base is the first set of points
			printf("Look for base!\n");
			threshold = counter;
			distance_criterion = 200;
			
		}
		printf("Threshold: %d, %f\n",threshold,distance_criterion);
		
		if (threshold > best_match_threshold && second_median <= distance_criterion){//35
			printf("\n\nYES WE HAVE TEMPLATE!!\n\n");
			//OBJECT HAS TO BE VERYFIED FOR SOME TIMES(2 OUT OF 4)
			times_check++;
			times_verified++;
			
		}
		else {
			printf("\n\nNO TEMPLATE\n\n");
			times_check++;
					
		}
	}//if selected_points is not null
	else{
		times_check++;
	}
	
		//CHECK IF OBJECT IS VERIFIED	
		if(times_check == 5){
				//OBJECT FOUND ENOUGH TIMES
				if(times_verified>=2){
					printf("\n\nOBJECT VERIFIED!!!\n\n");
					if(box_true_base_false){
						//freeze recongnition threads
						look_for_box=false;
						look_for_base=false;

						//keep an eye on this
						//if last frame is not ok

						grab_object(box_x);
						//unfreeze recongnition threads
						look_for_base =true;
						box_true_base_false = false;
					}
					else{//close grabber
						//freeze recongnition threads
						look_for_box=false;
						look_for_base=false;
						release_object(base_x);
						//unfreeze recongnition threads
						look_for_box=true;
						box_true_base_false = true;

					}
					look_for_image = false;	
									
				}
				//OBJECT NOT VERIFIED
				else{
					printf("\n\nNOT VERIFIED\n\n");
					turn_around();
					wait(1000);
					look_for_image = false;
					look_for_box =true;	
					look_for_base =false;				
					
				}
				template_found=false;
				times_check=0;
				times_verified=0;
		}
			
		//cvWaitKey(30);
		//release memory
		//cvReleaseImage(&frame);
		//cvReleaseMat(&image);

	}	//while
	//cvDestroyAllWindows();
	
}//method

IplImage* GetThresholdedImage(IplImage* img)
{
	// We first convert the image into an HSV space image
	IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
    	cvCvtColor(img, imgHSV, CV_BGR2HSV);
	IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
	/*Lower and upper bounds of whiteness in the frame*/
	// For white, S very low V white very high
	cvInRangeS(imgHSV, cvScalar(0, 0, 225), cvScalar(255, 30, 255), imgThreshed); //FOR YELLOW
	//cvInRangeS(imgHSV, cvScalar(245, 245, 10), cvScalar(255, 255, 50), imgThreshed);	
	cvReleaseImage(&imgHSV);
	return imgThreshed;

}
IplImage* DetectBases(IplImage* img)
{	

	//We store the contours we detect
    CvSeq* contours;
    CvSeq* contours2;
    CvSeq* contours3;		
		//Create a storage area, to store the contours
    CvMemStorage *storage = cvCreateMemStorage(0); 
	//For temporally storing the points of a contour as we go thorugh this contour 
    CvSeq* result;

	IplImage* ret = cvCreateImage(cvGetSize(img), 8, 3);
	IplImage* temp = cvCreateImage(cvGetSize(img), 8, 1);
	//cvCvtColor(img, temp, CV_BGR2BGR);
	cvCopy(img,temp,NULL);
	//Detect all the contours in the image
	cvFindContours(temp, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	// Loop over all the contours looking for which of them are quads.
	double largestArea = 0;
	CvSeq* largest_contour;
	CvSeq* result_largest=NULL;
	//------------------------------------------------CONTOURS1-------------------------//
	//find biggest area
	if(contours)
		printf("Contours1 size: %d\n",contours->total);	
	else{
		printf("Empty frame\n");
		return ret;
	}
	while(contours){

		result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0);

		if(result->total!=4 && fabs(cvContourArea(result, CV_WHOLE_SEQ))> 30) {	
				if(result!= NULL){
					
					double area = fabs(cvContourArea(result,CV_WHOLE_SEQ));
						if(area > largestArea){
							printf("Compare this area: %f to this one: %f\n\n",area,largestArea);
				       			largestArea = area;
							largest_contour = contours;
							result_largest = result;
				   		 }
						
				}
		}
		contours = contours->h_next;
	}
	int central_x=0,central_y=0;
	
	//points of biggest contours
	if(result_largest!= NULL){
		CvPoint *pt[result_largest->total];
		for(int i=0;i<result_largest->total;i++){
			pt[i] = (CvPoint*)cvGetSeqElem(result_largest, i);
			central_x+=pt[i]->x;
			central_y+=pt[i]->y;
		}
		central_x = central_x/result_largest->total;
		central_y = central_y/result_largest->total;
		printf("Biggest Contours Central x,y: %d,%d\n",central_x,central_y);
	}
	else{
		printf("Largest is empty!\n");
		return ret;
	}
	int contours_x=0,contours_y=0;
	vector <double> distances_to_center;
	int x_difference;
	int y_difference;
	double sum;
	double final;

	free(contours);
	//take all points of contours
	//reset back to beggining	
	cvFindContours(temp, storage, &contours2, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	

	//--------------------------------------------CONTOURS 2---------------------------//
		
	int pass_threshold=0;
	if(contours2){
		//printf("Contours2 size: %d\n",contours2->total);
	}	
	else{
		//printf("Empty frame\n");
		return ret;
        }	
	while(contours2){

			result = cvApproxPoly(contours2, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours2)*0.02, 0);

			if(result->total!=4 && fabs(cvContourArea(result, CV_WHOLE_SEQ))> 30) {	
				
				if(result!= NULL){
					pass_threshold++;
					CvPoint *pt[result->total];
					//take all points of each contours
		 		  	for(int i=0;i<result->total;i++){
						pt[i] = (CvPoint*)cvGetSeqElem(result, i);
						contours_x+=pt[i]->x;
						contours_y+=pt[i]->y;
					}
					contours_x = contours_x/(result_largest->total);
					contours_y = contours_y/(result_largest->total);
					//printf("Contours x,y: %d,%d\n",contours_x,contours_y);

					x_difference=central_x-contours_x;
					y_difference=central_y-contours_y;
				        sum = pow(x_difference,2)+pow(y_difference,2);
					final = (double)sqrt(sum);
					//printf("Distance: %f\n",final);
					distances_to_center.push_back(final);

		   		}
				contours_x=0;
				contours_y=0;
			}
			contours2 = contours2->h_next;
	}
	
	//sort vector to get mean
	//printf("pass_threshold total: %d\n",pass_threshold);
	int size_b= distances_to_center.size();
	//printf("Total Elements: %d\n",(int)size_b);
	double dist_m;
	double dist_l;
	for (int m=0;m< size_b;m++){
			
			for(int l=m+1;l<size_b;l++){
					
					dist_m =(double)distances_to_center[m];
					dist_l =(double)distances_to_center[l];
					
					if (dist_m > dist_l){
					
						//ERASE FIRST
						distances_to_center.erase (distances_to_center.begin()+m);
						//WE HAVE NEW M NOW
						//inserting new elements before the element at position m						
						distances_to_center.insert(distances_to_center.begin()+ m,dist_l);
						
						distances_to_center.erase (distances_to_center.begin()+l);
						distances_to_center.insert(distances_to_center.begin()+ l, dist_m);
						  
					}			
			}
	}
	//print vector to see if correct
	//PRINT VECTOR
			for (int m=0;m< size_b;m++){
				//printf("After Sort : Element at %d: %f\n",m,(double)distances_to_center[m]); 
			
			}	

	//-------------------------------------------------CONTOURS3--------------------------------------//
	//take the median of distances as threshold
	double median = 150;//distances_to_center[(int)(size_b/2)];

	free(contours2);
	//reset back to beggining	
	cvFindContours(temp, storage, &contours3, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	pass_threshold =0;
	if(contours3){
		//printf("Contours3 size: %d\n",contours3->total);	
	}
	else{
		//printf("Empty frame\n");
		return ret;
	}
	//printf("Contours3 size: %d\n",contours3->total);
	//draw contours
	

	while(contours3){
			result = cvApproxPoly(contours3, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours3)*0.02, 0);
			/*we can set a minimum limit on a contour to be considered a quad… say 20 pixels… so if a contour has more than 20 pixels 
			inside it, it would be considered a quad… otherwise it would be taken as noise and would be ignored (no drawn).*/
			
			//if box not detected to have 4 corners, could be avoided by the area criterion:
			//far: 600 near: 100
			//box total is more or less 4/ area is for the outliers
			//base is not detected as square
			contours_x=0;
			contours_y=0;

				
			CvPoint *pt[result->total];		
			
			//take all points of each contours
		 	for(int i=0;i<result->total;i++){
					pt[i] = (CvPoint*)cvGetSeqElem(result, i);
					contours_x+=pt[i]->x;
					contours_y+=pt[i]->y;
			}
			//printf("Contours_x ,,,,%d,,,,,, Contours_y,,,,,%d,,,,,result_largest ,,,%d,,,,,//\n",contours_x,contours_y,result_largest);	

			contours_x = contours_x/result_largest->total;
			contours_y = contours_y/result_largest->total;
			//printf("Contours x,y: %d,%d\n",contours_x,contours_y);
			//printf("BEFORE USING RESULT2\n");	
			x_difference=central_x-contours_x;
			y_difference=central_y-contours_y;
			sum = pow(x_difference,2)+pow(y_difference,2);
			final = (double)sqrt(sum);
			//printf("BEFORE USING RESULT3\n");	

			if(result->total!=4 && fabs(cvContourArea(result, CV_WHOLE_SEQ))> 30  && final <= median) {	
				pass_threshold++;
				//printf("BEFORE USING RESULT4\n");	
				if(result!= NULL){
					printf("BEFORE USING RESULT5\n");	
					CvPoint *pt[result->total];
		 		  	for(int i=0;i<result->total;i++)
		   			     pt[i] = (CvPoint*)cvGetSeqElem(result, i);
				 	for(int j=0;j<result->total;j++){				     
					    if(j!=result->total-1)				
						cvLine(ret, *pt[j], *pt[j+1], cvScalar(255));
					    else
						cvLine(ret, *pt[j], *pt[0], cvScalar(255));
					}
		   		}
				//printf("BEFORE USING RESULT6\n");	
	
			}
			contours3 = contours3->h_next;
	}
	
	//printf("3 WHILE: pass_threshold total: %d\n",pass_threshold);


	free(contours3);
	//Out of the loop, we have a pointer to the highest contour
        //cvDrawContours(ret, result_largest, CV_RGB(255,0,0), CV_RGB(0,255,0),CV_FILLED ==-1, 3, 8 );
	cvDrawContours(ret, result_largest, CV_RGB(255,0,0), CV_RGB(0,255,0),-1, CV_FILLED, 8 );
	//We are going to get the largest of the contours, the box that is closer to the camera, and set it as the target.
    	cvReleaseImage(&temp);
    	cvReleaseMemStorage(&storage);
	
    return ret;
}
//Detects shapes and draw them in an image
IplImage* DetectBoxes(IplImage* img)
{	
	double largestArea = 0;
	CvSeq* largest_contour;
	CvSeq* result_largest=NULL;
	//We store the contours we detect
    CvSeq* contours;
	//For temporally storing the points of a contour as we go thorugh this contour 
    CvSeq* result;
	//Create a storage area, to store the contours
    CvMemStorage *storage = cvCreateMemStorage(0);
    //CvMemStorage *storage2 = cvCreateMemStorage(0);

	IplImage* ret = cvCreateImage(cvGetSize(img), 8, 3);
	IplImage* temp = cvCreateImage(cvGetSize(img), 8, 1);
	//cvCvtColor(img, temp, CV_BGR2BGR);
	cvCopy(img,temp,NULL);
	//Detect all the contours in the image
	cvFindContours(temp, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	// Loop over all the contours looking for which of them are quads.
	while(contours){
        result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0);
	/*we can set a minimum limit on a contour to be considered a quad… say 20 pixels… so if a contour has more than 20 pixels 
	inside it, it would be considered a quad… otherwise it would be taken as noise and would be ignored (no drawn).*/
	if(result->total==4 && fabs(cvContourArea(result, CV_WHOLE_SEQ))> 20) {
	       // if(result->total==4)
		double area = fabs(cvContourArea(result,CV_WHOLE_SEQ));
		if(area > largestArea){
       			 largestArea = area;
        		largest_contour = contours;
			result_largest = result;
   		 }	
	}
	contours = contours->h_next;
	}
	//Out of the loop, we have a pointer to the highest contour
		if(result_largest!= NULL){
		CvPoint *pt[4];
         		   for(int i=0;i<4;i++)
           			     pt[i] = (CvPoint*)cvGetSeqElem(result_largest, i);
           		 cvLine(ret, *pt[0], *pt[1], cvScalar(255));
           		 cvLine(ret, *pt[1], *pt[2], cvScalar(255));
          		 cvLine(ret, *pt[2], *pt[3], cvScalar(255));
           		 cvLine(ret, *pt[3], *pt[0], cvScalar(255));}
	   //cvDrawContours(ret, result_largest, CV_RGB(255,0,0), CV_RGB(0,255,0),CV_FILLED ==-1, 3, 8 );
		cvDrawContours(ret, result_largest, CV_RGB(255,0,0), CV_RGB(0,255,0),-1, CV_FILLED, 8 );
//We are going to get the largest of the contours, the box that is closer to the camera, and set it as the target.

    cvReleaseImage(&temp);
    cvReleaseMemStorage(&storage);
	
    return ret;
}


//THREAD FOR FINDING BOXES

void *box_detection(void *threadid){
		sleep(3);
		//bool capt=true;
		int current_id = 0;
		int key = 0;
		/*CvCapture* capture = cvCreateCameraCapture(0);
		cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH,  640);//640//480
		cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT,  480);//480//360
		cvSetCaptureProperty( capture, CV_CAP_PROP_CONTRAST,  0.12); */
		//namedWindow("Capture");
		//namedWindow("WhitePatches");
		//namedWindow("Contours");
		IplImage* imgScribble = NULL;
		CvSeq* res;
		capture = cvCreateCameraCapture(0); 
		//MAKE WINDOW SMALLER
		cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH,  640);//640
		cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT,  480);//480
		cvSetCaptureProperty( capture, CV_CAP_PROP_CONTRAST,  0.12); 
		int misses=0;
		int perfect_Y_min;
		int perfect_Y_max;
		int near_Y;
 
		while( key!='q' )//key != 'q'
		{
			//DIFFICULT FRAMES COULD RESULT IN FAIL	
			//int firstFrame = gray == 0;
			//The image "frame" is originally stored 
			while(!look_for_box && !look_for_base);
			if(look_for_box){
				perfect_Y_min=240;
				perfect_Y_max=300;

			}
			else if(look_for_base){
				perfect_Y_min=235;
				perfect_Y_max=295;
			}
			//if(capt == true)
				//capt=false;	
				printf("\n\n\n 000000000000000000000000000000000000000000000 \n\n\n");
				cvGrabFrame(capture);
				cvGrabFrame(capture);				
				cvGrabFrame(capture);
				cvGrabFrame(capture);
				/*cvGrabFrame(capture);
				cvGrabFrame(capture);
				cvGrabFrame(capture);*/
				//IplImage* temp =cvRetrieveFrame(capture);
				
				IplImage* frame = cvRetrieveFrame(capture);
				if(!frame){
					break;printf("Frame error\n");}
				printf("\n\n\n 11111111111111111111111111111111111111111 \n\n\n");
				if(imgScribble == NULL){
			 		   imgScribble = cvCreateImage(cvGetSize(frame), 8, 3);
				}
				IplImage* WhitePatchImage = cvCreateImage(cvGetSize(frame), 8, 3);
			
				WhitePatchImage= GetThresholdedImage(frame);
				/*Following code works assuming that thereis just a white patch in the image.
				 If we assume that we can have more than one, we have to change it.*/
				 // Calculate the moments to estimate the position of the ball

				//DISTINCTION BETWEEN BASE AND BOX
				IplImage* contourDrawn;
			        if(look_for_box)
					contourDrawn = DetectBoxes(WhitePatchImage);
				else if (look_for_base)
					contourDrawn = DetectBases(WhitePatchImage);					

				//We convert the image with the larger patch into gray scale in order to calculate the moments.
				IplImage* gray=  cvCreateImage(cvGetSize(frame), 8, 1);
		      		CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));	
		      		cvCvtColor(contourDrawn, gray, CV_RGB2GRAY); 	
				cvMoments(gray, moments, 1);		  
				// we use calculations to figure out the patch position
		       		// The actual moment values
				/*You first allocate memory to the moments structure, and then you calculate the various moments. 
				And then using the moments structure, you calculate the two first order moments (moment10 and moment01) 
				and the zeroth order moment (area).Dividing moment10 by area gives the X coordinate of the white patch
				, and similarly, dividing moment01 by area gives the Y coordinate.*/
				double moment10 = cvGetSpatialMoment(moments, 1, 0);
		       	 	double moment01 = cvGetSpatialMoment(moments, 0, 1);
		      		double area = cvGetCentralMoment(moments, 0, 0);
				 // Holding the last and current white patch positions
		     		   static int posX = 0;
		     		   static int posY = 0;
		     		  int lastX = posX;
		      		  int lastY = posY;
		     		   posX = moment10/area;
		     		   posY = moment01/area;
		     		  printf("position (%d,%d)\n", posX, posY);
		
		     		  printf("area (%f)\n",area);
				 // We want to draw a line only if its a valid position

		       		 if(lastX!=0 && lastY!=0 && posX!=0 && posY!=0)
		      		 {
			  	     // Draw a yellow line from the previous point to the current point
		       		     cvLine(imgScribble, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,255,255), 5);
		     		 }
				//We simply create a line from the previous point to the current point, of yellow colour and a width of 5 pixels.
		 		cvAdd(frame, imgScribble, frame);
		      	         //cvShowImage("WhitePatches", WhitePatchImage);
				 //cvShowImage("Contours", contourDrawn);	
		      		 //cvShowImage("Capture", frame);
		//if(!gray)

			if(posX>0){
				misses=0;
				template_found= false; //deactivate the sensors.
				//FAR	
				if(posY<200){
					//FAR RIGHT
					if(posX>=450){//395
						printf("FAR RIGHT\n");
						printf("TURNING RIGHT");
						turn_right_slowly();
						wait(250);
						//sleep(1);
						//go_forward();
						CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
						CPhidgetMotorControl_setVelocity (motoControl, 1, 0.00);
						CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
						CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);
						wait(1000);
						}
					//FAR LEFT
					else if (posX<190){//235
						printf("FAR LEFT\n");
						printf("TURNING LEFT");
						turn_left_slowly();
						wait(250);
						CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
						CPhidgetMotorControl_setVelocity (motoControl, 1, 0.00);
						CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
						CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);
						wait(1000);
						//sleep(1);
					//	go_forward();
					}
					//FAR MIDDLE 
					else{
						printf("FAR MIDDLE\n");
						go_forward_slowly();
					}
				}
				//NEAR
				else if(posY>=200)   //Close to the box
				{
					template_found = true;  //Deactivate sensors
					//NEAR RIGHT
					if(posX>420){
						printf("NEAR RIGHT\n");
						printf("TURNING RIGHT CLOSE");
						turn_right_slowly();
						wait(300);//170
						CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
						CPhidgetMotorControl_setVelocity (motoControl, 1, 0.00);
						CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
						CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);
						wait(1500);
					}
					//NEAR LEFT
					else if (posX<220){//120
						printf("NEAR LEFT\n");
						
						printf("TURNING LEFT CLOSE");
						turn_left_slowly();
						wait(300);//170
						CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
						CPhidgetMotorControl_setVelocity (motoControl, 1, 0.00);
						CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
						CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);
						wait(1500);

					}
					//NEAR MIDDLE
					else
					{
						printf("NEAR MIDDLE\n");
						//VERY NEAR MIDDLE
						//IDENTIFIABLE DISTANCE
						if(posY>perfect_Y_min && posY<= perfect_Y_max){//250,315
							printf("PERFECT DISTANCE\n");
							//STOP, put this thread to wait, and allow the thread of the image recognition to work.
							CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
							CPhidgetMotorControl_setVelocity (motoControl, 1, 0.00);
							CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
							CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);
							printf("STOP\n");
							
							//BOOLEAN VALUES FOR THREADS
							if(look_for_box){
								
								look_for_box = false;
						
							}
							if(look_for_base){
								
								look_for_base = false;
								
							}
							look_for_image = true;
				
						}
						//posY is greater than 200 but less than 250
						//=> go forward a little
						//ROBOT A LITTLE FAR
						else if (posY<perfect_Y_min){
							printf("A LITTLE FAR\n");
							go_forward_slowly();
							wait(500);
							CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
							CPhidgetMotorControl_setVelocity (motoControl, 1, 0.00);
							CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
							CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);
						
						}
						//ROBOT TOO CLOSE TO OBJECT
						else if(posY> perfect_Y_max){
							printf("TOO CLOSE\n");
							go_backwards_slowly();
							wait(500);
							CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
							CPhidgetMotorControl_setVelocity (motoControl, 1, 0.00);
							CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
							CPhidgetMotorControl_setVelocity (motoControl, 0, 0.00);
						}
					}

				}

			}//posX >0
			else if(posX<=0|| posY<=0) {//if posX is negative the contours has dissappeared
				if (misses==5){
					printf("Misses times: %d\n\n",misses);
					misses=0;
					template_found= false;
					go_forward();
				}
				else
					misses++;
			}
			//capt = true;
		 	// Add the scribbling image and the frame...
		     		  
		 	cvReleaseImage(&WhitePatchImage);
			//cvReleaseImage(&temp);
			cvReleaseImage(&gray);
		//IF(CAPT==TRUE)
		//cvWaitKey(10);
	}//while( key!='q' )
	//cvDestroyAllWindows();
	cvReleaseCapture(&capture);
	pthread_exit(NULL);
}

/* MAIN 
 HERE all the threads are spawn using pthread_t*/

int main(int argc, char* argv[])
{
	
      //inform templates
	if(argc!=3){
	printf("Wrong number of arguments. Recall> ./terminator number_template(0,...,8) number_base(0,1)\n"); 
	return -1;
	}
	
      inform_templates();
      //MAKE YOUR CHOICE OF PIC HERE	
      chosen_box = atoi(argv[1]);      
      chosen_base = atoi(argv[2]);

      chosen_pic_box = templa[chosen_box];
      chosen_pic_base = templa[chosen_base];
	
      cout << "Reached Template descriptors\n";	
      inform_template_descriptors_box();
      
      //DO NOT SHOW-SAVE SOURCES
      //imshow("SURF detect126.793541or Templa", outTempla);
      
      //creation of threads
	power_button_reset();	
	printf("Button pressed %i times.\n",power_button_get_value());
    	while(power_button_get_value()<1)
    	{
       	 	sleep(1);
        	//printf("Button pressed %i times.\n",power_button_get_value());
   	 }

      pthread_t matches_thread,motor_thread,camera_thread,show_thread, box_thread;
      int name=1;
      int mt;
     
	  mt = pthread_create(&box_thread, NULL, 
                          box_detection, (void *)name);
      if (mt){
         cout << "Error:unable to create thread," << mt << endl;
         exit(-1);
      }

  //DESCRIPTORS THREAD
      mt = pthread_create(&matches_thread, NULL, 
                          calculate_matches, (void *)name);
      if (mt){
         cout << "Error:unable to create thread," << mt << endl;
         exit(-1);
      }
       
	 
     mt = pthread_create(&motor_thread, NULL, motorIK_method, (void *)name);
      if (mt){
         cout << "Error:unable to create motor thread," << mt << endl;
         exit(-1);
      }
       	char c;
	while (c != 't') 
		c = getchar();
	
	return 0;
}


