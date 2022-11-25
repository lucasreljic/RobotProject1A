#pragma config(Sensor, S1,     ,               sensorI2CCustom)
#pragma config(Sensor, S2,     ,               sensorI2CCustom9V)
//#pragma config(Sensor, S3,     ,               sensorSONAR)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "mindsensors-ev3smux.h"
#include "EV3Servo-lib-UW.c"


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
float triangulate(Position *robotPos);
bool driveUltrasonic(float distance, Position *robotPos);
void correctiveDrive(float distance, Position *robotPos);
void liftPID(int distance);
void driveToPos(Position targetPos, Position *robotPos, int finalRotation=0);
int pickUpObject();
void emergShutdown();
void goToBin(int color, Position *robotPos);


// motors
const int MOTOR_LEFT = motorD;
const int MOTOR_RIGHT = motorA;
const int MOTOR_LIFT = motorC;

// conversions
const float TICK_TO_CM = 180/(PI*1.6);
//const float CM_TO_TICK = (PI*1.6)/180;
const float DEG_TO_RAD = PI/180.0;
const float RAD_TO_DEG = 180/PI;

//
const float ULTRA_FOV = 8; // FOV
const float SENSOR_OFFSET = 5;
// const float CLAW_OFFSET = 5;
const int GRIP_LENGTH = 10;
const int TRI_OFFSET = 30;
const int TRI_LENGTH_B = 20;
const float SQUARE_LENGTH = 100.0;
const int MAX_LIFT = 25;
const int ULTRA_INTERCEPT = 39; //  distance from sensors where they intercept
const int ANGLE_CORRECTION = 3;
const int FIX_ANGLE = 2;
const int MAX_PASSES = 4;
const int MAX_POWER = 70;
const int GRIPPER_OPEN = 70;
const int GRIPPER_CLOSED = 0;

// sensors
const int RIGHT_ULTRA_PORT = 0;
const int SIDE_ULTRA_PORT = 1;
const int LEFT_ULTRA_PORT = (int) S3;
const int COLOR_PORT = 2;
const int GYRO_PORT = (int) S4;
const int TETRIX_PORT = (int) S2;
const int GRIPPER_PORT = 5;


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

	// tracks position of robot=
	Position robotPos;
	robotPos.x = 0;
	robotPos.y = 0;

	setGripperPosition(TETRIX_PORT, GRIPPER_PORT, GRIPPER_OPEN);
	//liftPID(MAX_LIFT);
	//while (true){
//		displayString(5, "color: %d", getMuxSensorValue(COLOR_PORT));}
	if (!getButtonPress(buttonEnter))
	{
		mainProgram(&robotPos);
	}
	emergShutdown();
}


// Main Logic of program
void mainProgram(Position *robotPos)
{
	bool failed = false;
	liftPID(MAX_LIFT);
	int inverted = 1;
	int passes = 0;

	while (!getButtonPress(buttonEnter) && passes < MAX_PASSES)
	{
		bool detected = driveUltrasonic(SQUARE_LENGTH*inverted, &*robotPos);

		if (detected)
		{
			float distToBlock = triangulate(&*robotPos);

			if (distToBlock != -1)
			{
				liftPID(0);

				correctiveDrive(distToBlock, &*robotPos);

				int objectColor = pickUpObject();
				if (objectColor == -1)
					failed = true;
				goToBin(objectColor, &*robotPos);
			}
			passes = 0;
			inverted = 1;
		}
		else
		{
			inverted *= -1;
			passes++;
		}
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
	const float KP = 0.26;//0.26
	const float KI = 0.001;//0.0008
	const float KD = 0.1;//0.23
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
		mPower = KP*error + KI*((error+prevError)*(time1[T1] + 1)/2) + KD*((error-prevError)/(time1[T1] + 1));
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
	const float KP = 0.26;//0.26
	const float KI = 0.001;//0.0008
	const float KD = 0.1;//0.23
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
		mPower = KP*error + KI*((error+prevError)*(time1[T1] + 1)/2) + KD*((error-prevError)/(time1[T1] + 1));// turn PID calculation
		driveBoth(-mPower, mPower);// turn motors based on motor power from PID with one being negative
		prevError = error;// previous error for PID
	}
	drive(0);
	return abs(getGyroDegrees(GYRO_PORT));
}


// -
float triangulate(Position *robotPos)
{

	float triLengthA = SensorValue[LEFT_ULTRA_PORT];
  float triLengthC = (getMuxSensorValue(RIGHT_ULTRA_PORT))/10;
  int count = 0;
  float rotatedOffset = 0;
  float avgTriLength = 0;
  float maxGammaTriLength = 0;

  const int REPS = 30;

 	while (!getButtonPress(buttonEnter) && triLengthA != avgTriLength && triLengthC != avgTriLength && count <= REPS)//!getButtonPress(buttonEnter) && triLengthA != avgTriLength && triLengthC != avgTriLength && count <= 20
 	{

  	avgTriLength = 0;
		triLengthA = SensorValue[LEFT_ULTRA_PORT];
		triLengthC = (getMuxSensorValue(RIGHT_ULTRA_PORT))/10;

		const int OKAY_DIFF = 2;

		if(triLengthA == 0 || triLengthC == 0)
		{
			displayString(1, "ERROR");
		}
		else if (triLengthA < ULTRA_INTERCEPT && triLengthC < ULTRA_INTERCEPT)
		{

			if ( abs(triLengthA - triLengthC) <= OKAY_DIFF)
			{
				count = REPS;
			}
			else if ((triLengthA < TRI_OFFSET-5 || triLengthC < TRI_OFFSET-5) && triLengthA != 0 && triLengthC != 0 && TRI_LENGTH_B + triLengthC > triLengthA)
			{
				float gammaInit = (acos((pow(triLengthA, 2) + pow(TRI_LENGTH_B, 2) - pow(triLengthC, 2))/(2*triLengthA*TRI_LENGTH_B)))/DEG_TO_RAD;

				avgTriLength = (triLengthA + triLengthC)/2;
				if(avgTriLength > maxGammaTriLength)
					maxGammaTriLength = avgTriLength;
				float gammaFinal = (acos(TRI_LENGTH_B/(2*avgTriLength)))/DEG_TO_RAD;
				float deltaGamma = (gammaInit - gammaFinal);
				if(abs(deltaGamma) < 90)
				{
					rotateRobot(deltaGamma/ANGLE_CORRECTION);
				}
			}
		}
		else if ((triLengthA > ULTRA_INTERCEPT && triLengthC < ULTRA_INTERCEPT))
			rotateRobot(-FIX_ANGLE);
		else if ((triLengthA < ULTRA_INTERCEPT && triLengthC > ULTRA_INTERCEPT))
			rotateRobot(FIX_ANGLE);
		else if (triLengthA > ULTRA_INTERCEPT && triLengthC > ULTRA_INTERCEPT)
			displayString(7, "both too long");
		count++;
	}
	rotatedOffset = sqrt(pow(maxGammaTriLength, 2)-pow(TRI_LENGTH_B/2, 2));
	//if (rotatedOffset < GRIP_LENGTH)
		//return(0);
	return(TRI_OFFSET+GRIP_LENGTH);
}


// -
bool driveUltrasonic(float distance, Position *robotPos)
{

	int inverted = 1;
	if (distance < 0)
		inverted = -1;
	float sensorDistance = 0;
	const float RANGE = 105;
	int initEncoder = nMotorEncoder[motorA];
	drive(-30*inverted);
	float turnError = 0;
	while (!getButtonPress(buttonEnter) && fabs((nMotorEncoder[motorA] - initEncoder)/TICK_TO_CM) < fabs(distance) && !(getMuxSensorValue(SIDE_ULTRA_PORT)/10 < RANGE))
	{
		turnError = getGyroDegrees(GYRO_PORT);
		sensorDistance = getMuxSensorValue(SIDE_ULTRA_PORT)/10;
		driveBoth(-30*inverted + turnError, -30*inverted - turnError);
	}
	if(getButtonPress(buttonEnter))
		emergShutdown();

	sensorDistance = getMuxSensorValue(SIDE_ULTRA_PORT)/10;
	drive(0);
	wait1Msec(1000);

	float distTraveled = 0;


	// going forward: -20 -> -80   -20-(-80)= 60
	// going backward -80 -> -20   -80-(-20)= -60

	//if (-1*inverted < 0) // going forward (decreasing encoder)

	//	distTraveled = (initEncoder - nMotorEncoder[motorA])/TICK_TO_CM;
	//else // going backward (increasing encoder)
	//{
	distTraveled = ((initEncoder - nMotorEncoder[motorA])/TICK_TO_CM);

	(*robotPos).x += distTraveled;

	displayString(1, "ULTRASONIC");
	displayString(2, "new x: %f", (*robotPos).x);
	displayString(3, "new y: %f", (*robotPos).y);
	displayString(4, "x change: %f", distTraveled);
	displayString(5, "y change: ---");

	writeDebugStreamLine("ULTRASONIC");
	writeDebugStreamLine("new x: %f", (*robotPos).x);
	writeDebugStreamLine("new y: %f", (*robotPos).y);
	writeDebugStreamLine("x change: %f", distTraveled);
	writeDebugStreamLine("y change: ---");

	// when an object is spotted
	if(sensorDistance < RANGE)
	{

		float offset = SENSOR_OFFSET + sin(ULTRA_FOV*DEG_TO_RAD)*sensorDistance;
		correctiveDrive(offset*inverted, &*robotPos);
		rotateRobot(90);
		//float objDist = sqrt (pow(sensorDistance, 2) - pow(offset, 2));
		if (sensorDistance > TRI_OFFSET)
			correctiveDrive(sensorDistance-TRI_OFFSET, &*robotPos);
		return(true);
	}

	drive(0);
	return (false);
}


// distance (positive is forward, negative is reverse)
void correctiveDrive(float distance, Position *robotPos)
{

	//Only works when distance is negative
	//distance *= -1;
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
	time1[T1] = 0;// start timer for PID loop
	float error = distance;
	float distanceTravelled = 0;
	while (!getButtonPress(buttonEnter) && abs(error) > TOLERANCE)
	{
		if(nMotorEncoder[MOTOR_RIGHT] > nMotorEncoder[MOTOR_LEFT])// takes the lowest encoder value
		{
			distanceTravelled = -nMotorEncoder[MOTOR_LEFT]/TICK_TO_CM;
		}
		else
		{
			distanceTravelled = -nMotorEncoder[MOTOR_RIGHT]/TICK_TO_CM;
		}
		turnError = ANGLE - getGyroDegrees(GYRO_PORT);// Turn PID error
		mTurnPower = TURN_KP*turnError + TURN_KI*((turnError+turnPrevError)*(time1[T1] + 1)/2) + TURN_KD*(turnError-turnPrevError);// turn pid calculation
		error = (distance - distanceTravelled);// Drive PID error

		mPower = KP*error + KI*((error+prevError)*(time1[T1] + 1)/2) + KD*abs(((error-prevError)/(time1[T1] + 1)));// drive PID calculation
		if(mPower > MAX_POWER)
		{
			mPower = MAX_POWER;
		}
		driveBoth((-mPower -mTurnPower), (-mPower +mTurnPower));//add turn power to drive power to adjust for
		prevError = error;// for PID
		turnPrevError = turnError;// for Turn PID
	}
	rotateAbsolute(ANGLE);
	drive(0);// stop motors

	(*robotPos).x += cos((ANGLE)*DEG_TO_RAD)*distance;
	(*robotPos).y += sin((ANGLE)*DEG_TO_RAD)*distance;

	displayString(1, "CORRECTIVE");
	displayString(2, "new x: %f", (*robotPos).x);
	displayString(3, "new y: %f", (*robotPos).y);
	displayString(4, "x change: %f", cos((ANGLE)*DEG_TO_RAD)*distance);
	displayString(5, "y change: %f", sin((ANGLE)*DEG_TO_RAD)*distance);

	writeDebugStreamLine("CORRECTIVE");
	writeDebugStreamLine("new x: %f", (*robotPos).x);
	writeDebugStreamLine("new y: %f", (*robotPos).y);
	writeDebugStreamLine("x change: %f", cos((ANGLE)*DEG_TO_RAD)*distance);
	writeDebugStreamLine("y change: %f", sin((ANGLE)*DEG_TO_RAD)*distance);
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

	Position posRelative;
	posRelative.x = (*robotPos).x - targetPos.x;
	posRelative.y = (*robotPos).y - targetPos.y;

	const float TOL = 1.0/10;
	if ( fabs(posRelative.x - targetPos.x) <= TOL && fabs(posRelative.x - targetPos.x) <= TOL)
	{
		rotateAbsolute(finalRotation);
	}
	else
	{
		float angle = posRelative.x==0? 90 : atan2(posRelative.y, posRelative.x)*RAD_TO_DEG;

		rotateAbsolute(180 + angle);
		correctiveDrive( sqrt(pow(posRelative.x, 2) + pow(posRelative.y, 2)) , &*robotPos);

		rotateAbsolute(finalRotation);
	}
}


// -
int pickUpObject()
{
	setGripperPosition(TETRIX_PORT, GRIPPER_PORT, GRIPPER_OPEN);
	liftPID(0);
	wait1Msec(1000);
	setGripperPosition(TETRIX_PORT, GRIPPER_PORT, GRIPPER_CLOSED);
	liftPID(MAX_LIFT);
	return getMuxSensorValue(COLOR_PORT);
}


// -
void emergShutdown() // called when stuff goes really wrong
{
	driveBoth(0,0);
	setGripperPosition(TETRIX_PORT, GRIPPER_PORT, GRIPPER_OPEN);
	liftPID(0);
	wait1Msec(5000);
	stopAllTasks();
}


// -

const int BIN_DISTANCE = 30;
void goToBin(int color, Position *robotPos)
{
	Position targetPos;

	if(color == (int)colorRed)
	{
		targetPos.x = -BIN_DISTANCE;
		targetPos.y = 0;
		driveToPos(targetPos, &*robotPos, -90);
		}
	if(color == (int)colorBlue)
	{
		targetPos.x = 0;
		targetPos.y = -BIN_DISTANCE;
    driveToPos(targetPos, &*robotPos, 180);
    }
	if(color == (int)colorGreen)
	{
		targetPos.x = -BIN_DISTANCE;
		targetPos.y = -BIN_DISTANCE;
    driveToPos(targetPos, &*robotPos, 225);
  }

	wait1Msec(100);
	setGripperPosition(TETRIX_PORT, GRIPPER_PORT, GRIPPER_OPEN);
	correctiveDrive(-20, &*robotPos);

	Position origin;
	origin.x = 0;
	origin.y = 0;
	driveToPos(origin, &*robotPos);
}
