#pragma config(Sensor, S1,     ,               sensorI2CCustom)
#pragma config(Sensor, S2,     ,               sensorI2CCustom9V)
//#pragma config(Sensor, S3,     ,               sensorSONAR)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "mindsensors-ev3smux.h"
#include "EV3Servo-lib-UW.c"
#include "EV3Multiplex.c"


// class for position (x,y)
typedef struct
{
	float x;
	float y;
} Position;


// function prototypes
void mainProgram(Position *robotPos);
int getMuxSensorValue(int i);
void configureSensors();
void drive(int pwr);
void driveBoth(int pwrL, int pwrR);
int rotateRobot(int angle);
int rotateAbsolute(int angle);
void triangulate(Position *robotPos);
bool driveUltrasonic(int distance, Position *robotPos);
void correctiveDrive(int distance, Position *robotPos);
void liftPID(int distance);
void driveToPos(Position targetPos, Position *robotPos, int finalRotation=0);
int pickUpObject();
void emergShutdown();
void goToBin(int color, Position *robotPos);


// constants
const int MOTOR_LEFT = motorD;
const int MOTOR_RIGHT = motorA;
const int MOTOR_LIFT = motorC;
const int MAX_POWER = 70;
const float TICK_TO_CM = 180/(PI*1.6);
//const float CM_TO_TICK = (PI*1.6)/180;
const float DEG_TO_RAD = PI/180;
const float RAD_TO_DEG = 180/PI;
const float ULTRA_DEG = 12;
const float SENSOR_OFFSET = 5;
const float CLAW_OFFSET = 5;
const int TRI_OFFSET = 30;
const int TRI_LENGTH_B = 20;
const int SQUARE_LENGTH = 100;
const int MAX_LIFT = 22;

// sensors
const int RIGHT_ULTRA_PORT = 0;
const int SIDE_ULTRA_PORT = 1;
const int LEFT_ULTRA_PORT = (int) S3;
const int COLOR_PORT = 2;
const int GYRO_PORT = (int) S4;
const int GRIPPER_PORT = (int) S2;


tMSEV3 muxedSensor[3];
tEV3SensorTypeMode typeMode[3] = {sonarCM, sonarCM, colorMeasureColor};


task main()
{
	clearDebugStream();

	configureSensors();
	if (!initSensor(&muxedSensor[0], msensor_S1_1, typeMode[0]))
  	writeDebugStreamLine("initSensor() failed! for msensor_S1_1");

  if (!initSensor(&muxedSensor[1], msensor_S1_2, typeMode[1]))
  	writeDebugStreamLine("initSensor() failed! for msensor_S1_2");

  if (!initSensor(&muxedSensor[2], msensor_S1_3, typeMode[2]))
  	writeDebugStreamLine("initSensor() failed! for msensor_S1_3");

	// tracks position of robot
	Position robotPos;
	robotPos.x = 0;
	robotPos.y = 0;


  liftPID(-MAX_LIFT);
}


// Main Logic of program
void mainProgram(Position *robotPos)
{
	bool failed = false;
	liftPID(MAX_LIFT);
	int count = 0;
	while (!getButtonPress(buttonEnter) && (count < 2 || failed == false))
	{
		count = 0;
	  bool detected = false;
		while(!getButtonPress(buttonEnter) && (count < 2 || detected == true))
		{
			detected = driveUltrasonic( SQUARE_LENGTH, &*robotPos);
			detected = driveUltrasonic(-SQUARE_LENGTH, &*robotPos);
			count++;
		}
		sleep(1000);
		triangulate(robotPos);
		//correctiveDrive(TRI_OFFSET + CLAW_OFFSET, &*robotPos);
		int objectColor = pickUpObject();
		if (objectColor == -1)
			failed = true;

		//goToBin(objectColor, &*robotPos);
	 	Position origin;
		origin.x = 0;
		origin.y = 0;
		driveToPos(origin, &*robotPos);
	}

	emergShutdown();// in case button enter is pressed or program ends
}


// Reading from the Multiplexer (int i) is the port number of the sensor
int getMuxSensorValue(int i)
{
	sleep(100);//wait for i2c port
	if (!readSensor(&muxedSensor[i]))
		writeDebugStreamLine("readSensor() failed! for %d", i);
 	if(muxedSensor[i].typeMode == sonarCM)
		return muxedSensor[i].distance;
	else if (muxedSensor[i].typeMode == colorMeasureColor)
		return muxedSensor[i].color;
	return -1;
}


// Configures all the sensors connected directly to the EV3
void configureSensors()
{
	SensorType[LEFT_ULTRA_PORT] = sensorSONAR;
	wait1Msec(50);
	SensorType[GYRO_PORT] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[GYRO_PORT] = modeEV3Gyro_Calibration;
	wait1Msec(50);
	SensorMode[GYRO_PORT] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
}


// Drive both motors at the same power
void drive(int pwr)
{
	motor[MOTOR_LEFT] = motor[MOTOR_RIGHT] = pwr;
}


// Drive each side individually
void driveBoth(int pwrL, int pwrR)
{
	motor[MOTOR_LEFT] = pwrL;
	motor[MOTOR_RIGHT] = pwrR;
}


// Rotates the robot an angle relative to its current angle, positive angles are clockwise when viewed from above
int rotateRobot(int angle)
{
	int startAngle = getGyroDegrees(GYRO_PORT);
	const float KP = 0.5;//0.26
	const float KI = 0.001;//0.0008
	const float KD = 0.01;//0.23
	float error = angle - (getGyroDegrees(GYRO_PORT)-startAngle);
	float mPower = 0;
	float prevError = 0;
	time1[T1] = 0;

	bool positive = angle>0? true: false;
	while ( !getButtonPress(buttonEnter)
				  && abs((getGyroDegrees(GYRO_PORT)-startAngle) - angle) > 0
				  && (  positive*(getGyroDegrees(GYRO_PORT)-startAngle<angle)   ||    (!positive)*(getGyroDegrees(GYRO_PORT)-startAngle>angle)  )
				 )
	{
		error = abs(angle - (getGyroDegrees(GYRO_PORT)-startAngle));
		mPower = KP*error + KI*((error+prevError)*(time1[T1] + 1)/2) + KD*abs(((error-prevError)/(time1[T1] + 1)));
		if (angle>0)
		{
			driveBoth(-mPower, mPower);
		}
		else
		{
			driveBoth(mPower,-mPower);
		}
		prevError = error;
	}
	drive(0);
	return abs(getGyroDegrees(GYRO_PORT));
}


// Rotates the robot to an angle relative to the initial gyro zeroing when program starts, positive angles are clockwise when viewed from above
int rotateAbsolute(int angle)
{
	const float KP = 0.5;//0.26
	const float KI = 0.001;//0.0008
	const float KD = 0.01;//0.23
	float error = angle - (getGyroDegrees(GYRO_PORT));//
	float mPower = 0;
	float prevError = 0;
	time1[T1] = 0;// reset timer for PID loop

	bool positive = (getGyroDegrees(GYRO_PORT) - angle)>0? true : false;
	while (!getButtonPress(buttonEnter)
				 && abs(getGyroDegrees(GYRO_PORT) - angle) > 0
				 && (  positive*((getGyroDegrees(GYRO_PORT)-angle)>0)   ||    (!positive)*((getGyroDegrees(GYRO_PORT)-angle)<0)  )
				)
	{
		error = angle - getGyroDegrees(GYRO_PORT);// error for turn PID
		mPower = KP*error + KI*((error+prevError)*(time1[T1] + 1)/2) + KD*abs(((error-prevError)/(time1[T1] + 1)));// turn PID calculation
		driveBoth(-mPower, mPower);// turn motors based on motor power from PID with one being negative
		prevError = error;// previous error for PID
	}
	drive(0);
	return abs(getGyroDegrees(GYRO_PORT));
}


// -
void triangulate(Position *robotPos)
{
	int triLengthA = SensorValue[LEFT_ULTRA_PORT]/2;
  int triLengthC = (getMuxSensorValue(RIGHT_ULTRA_PORT))/20;
  float avgTriLength = 0;
  int count = 0;
  if(triLengthA == 0 || triLengthC == 0)
  	writeDebugStreamLine("ERROR");

 	while (!getButtonPress(buttonEnter) && triLengthA != avgTriLength && count != 10)
 	{
		triLengthA = SensorValue[LEFT_ULTRA_PORT]/2;
		triLengthC = (getMuxSensorValue(RIGHT_ULTRA_PORT))/20;
		if(triLengthA == 0 || triLengthC == 0)
			writeDebugStreamLine("ERROR");
		if (triLengthA < 50 && triLengthC < 50 && triLengthA != 0 && triLengthC != 0 && TRI_LENGTH_B + triLengthC > triLengthA)
		{
			float gammaInit = (acos((pow(triLengthA, 2) + pow(TRI_LENGTH_B, 2) - pow(triLengthC, 2))/(2*triLengthA*TRI_LENGTH_B)))/DEG_TO_RAD;
			writeDebugStreamLine("Gamma init: %d",gammaInit);
			avgTriLength = (triLengthA + triLengthC)/2;
			float gammaFinal = acos(TRI_LENGTH_B/(2*avgTriLength))/DEG_TO_RAD;
			writeDebugStreamLine("Gamma final: %d",gammaInit);
			float deltaGamma = (gammaInit - gammaFinal);

			if(abs(deltaGamma) < 90)
			{
				rotateRobot(deltaGamma);
			}
			writeDebugStreamLine("rotated: %d", deltaGamma);
		}
		count++;
		writeDebugStreamLine("inner loop");
		writeDebugStreamLine("Left: %i", SensorValue[LEFT_ULTRA_PORT]/2);
		writeDebugStreamLine("Right: %i", getMuxSensorValue(RIGHT_ULTRA_PORT)/20);
	}
	correctiveDrive(abs(sqrt(pow(TRI_LENGTH_B/2, 2) + pow(avgTriLength, 2))), robotPos);
}


// -
bool driveUltrasonic(int distance, Position *robotPos)
{
	float initDistance	= nMotorEncoder[motorA];
	int inverted = 1;
	if (distance < 0)
		inverted = -1;
	float sensorDistance = 0;
	float error = 0;
	const float RANGE = 105;
	const float TOLERANCE = 0.5;
	int initEncoder = nMotorEncoder[motorA];
	drive(-30*inverted);
	while (!getButtonPress(buttonEnter) && abs((nMotorEncoder[motorA] - initEncoder)/TICK_TO_CM) < abs(distance) && !(getMuxSensorValue(SIDE_ULTRA_PORT)/10 < RANGE))
	{

		sensorDistance = getMuxSensorValue(SIDE_ULTRA_PORT)/10;
	}

	drive(0);
	wait1Msec(100);
	// when an object is spotted
	if(sensorDistance < RANGE)
	{

		float distTraveled = 0;
		if (-1*inverted < 0) // going forward (decreasing encoder)
			distTraveled = (initEncoder - nMotorEncoder[motorA])/TICK_TO_CM;
		else // going backward (increasing encoder)
			distTraveled = (initEncoder - nMotorEncoder[motorA])/TICK_TO_CM;
		(*robotPos).x += distTraveled;

		writeDebugStreamLine("x pos: %f", (*robotPos).x);
		writeDebugStreamLine("y pos: %f", (*robotPos).y);


		writeDebugStreamLine("sensor dist: %f", sensorDistance);
		float offset = SENSOR_OFFSET + sin(ULTRA_DEG*DEG_TO_RAD)*sensorDistance;
		writeDebugStreamLine("Distance to travel back: %f", offset);
		correctiveDrive(offset, &*robotPos);
		rotateRobot(90);
		float objDist = sqrt (pow (offset, 2) + pow(sensorDistance, 2));
		float objDistTravel = objDist-TRI_OFFSET;
		if(TRI_OFFSET > objDist)
		{
			objDistTravel = 0;
		}
		correctiveDrive(objDistTravel, &*robotPos);
		return(true);
	}
	//writeDebugStreamLine("not funny")
	drive(0);
	return (false);
}


// -
void correctiveDrive(int distance, Position *robotPos)
{

	//Only works when distance is negative
	distance *= -1;
	//Driving PID Constants
	const float KP = 1;
	const float KI = 0.001;
	const float KD = 0.02;
	//Turn PID Constants
	const float TURN_KP = 0.5;//0.26
	const float TURN_KI = 0.0005;//0.0008
	const float TURN_KD = 0.01;//0.23
	const float ANGLE = getGyroDegrees(GYRO_PORT);//get angle before driving starts
	const float TOLERANCE = 0.5;
	float turnError = 0;
	float mTurnPower = 0;
	float turnPrevError = 0;
	nMotorEncoder[MOTOR_RIGHT] = 0;
	nMotorEncoder[MOTOR_LEFT] = 0;
	float mPower = 0;
	float prevError = 0;
	int inverted = 1;
	time1[T1] = 0;// start timer for PID loop
	if (distance < 0)// allows for negative direction
	{
		inverted = -1;
	}
	float error = distance*inverted;
	float distanceTravelled = 0;
	while (!getButtonPress(buttonEnter) && abs(error) > TOLERANCE)
	{
		if(nMotorEncoder[MOTOR_RIGHT] > nMotorEncoder[MOTOR_LEFT])// takes the lowest encoder value
		{
			distanceTravelled = nMotorEncoder[MOTOR_LEFT]/TICK_TO_CM;
		}
		else
		{
			distanceTravelled = nMotorEncoder[MOTOR_RIGHT]/TICK_TO_CM;
		}
		turnError = ANGLE - getGyroDegrees(GYRO_PORT);// Turn PID error
		mTurnPower = TURN_KP*turnError + TURN_KI*((turnError+turnPrevError)*(time1[T1] + 1)/2) + TURN_KD*(turnError-turnPrevError);// turn pid calculation
		error = (distance - distanceTravelled)*inverted;// Drive PID error

		mPower = KP*error + KI*((error+prevError)*(time1[T1] + 1)/2) + KD*abs(((error-prevError)/(time1[T1] + 1)));// drive PID calculation
		if(mPower > MAX_POWER)
		{
			mPower = MAX_POWER;
		}
		driveBoth((mPower*inverted -mTurnPower), (mPower*inverted +mTurnPower));//add turn power to drive power to adjust for
		prevError = error;// for PID
		turnPrevError = turnError;// for Turn PID
	}
	rotateAbsolute(ANGLE);
	drive(0);// stop motors

	(*robotPos).x += cos((ANGLE)*DEG_TO_RAD)*-distance;
	(*robotPos).y += sin((ANGLE)*DEG_TO_RAD)*-distance;

	writeDebugStreamLine("x change: %f", cos((ANGLE)*DEG_TO_RAD)*-distance);
	writeDebugStreamLine("y change: %f", sin((ANGLE)*DEG_TO_RAD)*-distance);
	writeDebugStreamLine("x pos: %f", (*robotPos).x);
	writeDebugStreamLine("y pos: %f", (*robotPos).y);
}


// -
void liftPID(int distance)
{
	distance *= -1;
	const float KP = 0.85;
	const float KI = 0.005;
	const float KD = 0.05;
	const float TOLERANCE = 0.5;
	float error = distance - nMotorEncoder[MOTOR_LIFT]/TICK_TO_CM;
	float mPower = 0;
	float prevError = 0;
	time1[T1] = 0;
	while (!getButtonPress(buttonEnter) && abs(error) > TOLERANCE)
	{
		error = distance - nMotorEncoder[MOTOR_LIFT]/TICK_TO_CM;
		mPower = KP*error + KI*((error+prevError)*(time1[T1] + 1)/2) + KD*abs(((error-prevError)/(time1[T1] + 1)));
		motor[MOTOR_LIFT] = mPower;
		prevError = error;
	}
	motor[MOTOR_LIFT] = 0;
}


// -
void driveToPos(Position targetPos, Position *robotPos, int finalRotation)
{

	rotateAbsolute(180);

	float angle = 0;

	Position posRelative;
	posRelative.x = (*robotPos).x - targetPos.x;
	posRelative.y = (*robotPos).y - targetPos.y;

	angle = posRelative.x==0? 90 : atan2(posRelative.y, posRelative.x)*RAD_TO_DEG;

	rotateRobot(angle);
	correctiveDrive( -sqrt(pow(posRelative.x, 2) + pow(posRelative.y, 2)) , &*robotPos);

	rotateAbsolute(finalRotation);
}


// -
int pickUpObject()
{
	setGripperPosition(GRIPPER_PORT, 5, 70);
	liftPID(0);
	setGripperPosition(GRIPPER_PORT, 5, 1);
	liftPID(MAX_LIFT);
	return getMuxSensorValue(COLOR_PORT);
}


// -
void emergShutdown() // called when stuff goes really wrong
{
	driveBoth(0,0);
	setGripperPosition(GRIPPER_PORT, 5, 65);
	liftPID(0);
}


// -
void goToBin(int color, Position *robotPos)
{
	Position targetPos;

	if(color == (int)colorRed)
	{
		targetPos.x = 10;
		targetPos.y = 0;
		driveToPos(targetPos, &*robotPos, 270);
		//drive forward slightly to container
	}
	if(color == (int)colorBlue)
	{
		targetPos.x = 0;
		targetPos.y = 10;
    driveToPos(targetPos, &*robotPos, 180);
    //drive forward slightly to container
	}
	if(color == (int)colorGreen)
	{
		targetPos.x = 0;
		targetPos.y = 0;
    driveToPos(targetPos, &*robotPos, 225);
    //drive forward slightly to container
	}
	setGripperPosition(GRIPPER_PORT, 5, 65);
	targetPos.x = 0;
	targetPos.y = 0;

	driveToPos(targetPos, &*robotPos, 0); // return to origin
}
