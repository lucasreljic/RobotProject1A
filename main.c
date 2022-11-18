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
int getMuxSensorValue(int i);
void configureSensors();
void drive(int pwr);
void driveBoth(int pwrL, int pwrR);
int rotateRobot(int angle);
int rotateAbsolute(int angle);
void triangulate();
void driveUltrasonic(int distance, Position *robotPos );
void correctiveDrive(int distance, Position *robotPos);
void liftPID(int distance);
void driveToPos(Position targetPos, Position *robotPos, int finalRotation=0);


// constants
const int MOTOR_LEFT = motorD;
const int MOTOR_RIGHT = motorA;
const int MOTOR_LIFT = motorC;
const int MAX_POWER = 70;
const float TICK_TO_CM = 180/(PI*1.6);
const float CM_TO_TICK = (PI*1.6)/180;
const float DEG_TO_RAD = PI/180;
const float RAD_TO_DEG = 180/PI;
const float ULTRA_DEG = 12;
const float SENSOR_OFFSET = 5;
const float TRI_LENGTH_B = 20;


// sensors
const int RIGHT_ULTRA_PORT = 0;
const int SIDE_ULTRA_PORT = 1;
const int LEFT_ULTRA_PORT = (int) S3;
const int COLOR_PORT = 2;
const int GYRO_PORT = (int) S4;


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


	while (!getButtonPress(buttonEnter))
	{
		while(!getButtonPress(buttonAny))
		{}

		if (getButtonPress(buttonUp))
		{
			driveUltrasonic(-75, &robotPos);
		}
	}
}





// -
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


// -
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



// -
void drive(int pwr)
{
	motor[MOTOR_LEFT] = motor[MOTOR_RIGHT] = pwr;
}


// -
void driveBoth(int pwrL, int pwrR)
{
	motor[MOTOR_LEFT] = pwrL;
	motor[MOTOR_RIGHT] = pwrR;
}


// -
int rotateRobot(int angle) //rotates robot in place to given angle then stops. Positive angles are clockwise when viewed from above
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


// -
int rotateAbsolute(int angle) //rotates robot in place to given angle then stops. Positive angles are clockwise when viewed from above
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
void triangulate()
{
  int triLengthA = SensorValue[LEFT_ULTRA_PORT];
  int triLengthC = (getMuxSensorValue(RIGHT_ULTRA_PORT))/10;
  if(triLengthA == 0 || triLengthC == 0)
  	writeDebugStreamLine("ERROR");
  if (triLengthA < 30 && triLengthC < 30 && triLengthA != 0 && triLengthC != 0)
  {
  	float gammaInit = (acos((pow(triLengthA, 2) + pow(TRI_LENGTH_B, 2) - pow(triLengthC, 2))/(2*triLengthA*TRI_LENGTH_B)))/DEG_TO_RAD;
  	float avgTriLength = (triLengthA + triLengthC)/2;
  	float gammaFinal = acos(TRI_LENGTH_B/(2*avgTriLength))/DEG_TO_RAD;
  	int deltaGamma = ceil(gammaInit - gammaFinal);
  	if(abs(deltaGamma) < 90)
		{
		rotateRobot(deltaGamma);
		}
	}
}


// -
void driveUltrasonic(int distance, Position *robotPos)
{

	nMotorEncoder[motorA] = 0;
	const float TOLERANCE = 0.5;
	float error = distance;
	int inverted = 1;
	float sensorDistance = 0;
	float threshold = getMuxSensorValue(SIDE_ULTRA_PORT) - 5;
	while (!getButtonPress(buttonEnter) && !(getMuxSensorValue(SIDE_ULTRA_PORT) < threshold))
	{
		sensorDistance = getMuxSensorValue(SIDE_ULTRA_PORT);
		if(abs(error) < TOLERANCE)
		{
			inverted *= -1;
		}

		error = distance - (nMotorEncoder[motorA]/TICK_TO_CM)*inverted;
		drive(error*inverted);
	if(distance < (nMotorEncoder[motorA]/TICK_TO_CM)*inverted)
	{
		inverted *=-1;
	}
	}
	if(getMuxSensorValue(SIDE_ULTRA_PORT) < threshold)
	{
		sensorDistance = getMuxSensorValue(SIDE_ULTRA_PORT);
		drive(0);
		correctiveDrive(SENSOR_OFFSET + sin(ULTRA_DEG*DEG_TO_RAD)*sensorDistance, &*robotPos);
		rotateRobot(-90);
		correctiveDrive(sensorDistance, &*robotPos);
	}
	drive(0);
}


// -
void correctiveDrive(int distance, Position *robotPos)
{

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
	const float KP = 0.85;
	const float KI = 0.005;
	const float KD = 0.05;
	const float TOLERANCE = 0.5;
	nMotorEncoder[MOTOR_LIFT] = 0;
	float error = distance - nMotorEncoder[MOTOR_LIFT]*TICK_TO_CM;
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
