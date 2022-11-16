//start of code

const int MOTOR_LEFT = motorD;
const int MOTOR_RIGHT = motorA;
const int MOTOR_LIFT = motorC;
const int MAX_POWER = 70;
const float TICK_TO_CM = 180/(PI*1.6);//Encoder ticks to CM calculation
//const float CM_TO_TICK = (PI*1.6)/180;//Encoder ticks to CM calculation
const float DEG_TO_RAD = PI/180;
const float RAD_TO_DEG = 180/PI;
const float ULTRA_DEG = 12;
const float SENSOR_OFFSET = 5;


// sensor constants
const int TOUCH_PORT = (int) S1;
const int ULTRASIDE_PORT = (int) S2;
const int COLOR_PORT = (int) S3;
const int GYRO_PORT = (int) S4;


void configureSensors(); // 1
void drive(int pwr);
void driveBoth(int pwrL, int pwrR);
int rotateRobot(int angle); // 2
int rotateAbsolute(int angle); // 3
void driveUltrasonic(int distance); // 4
void correctiveDrive(int distance); // 5
void liftPID(int distance); // 6
void returnToOrigin();


// tracks position (x and y coordinates) in cm
typedef struct
{
	float x;
	float y;
} Position;


// tracks position of robot
Position robotPos;


task main()
{
	clearDebugStream();
	configureSensors();

	robotPos.x = 0;
	robotPos.y = 0;

	while (true)
	{
		while(!getButtonPress(buttonAny))
		{}

		if (getButtonPress(buttonLeft))
			rotateRobot(10);
		else if (getButtonPress(buttonRight))
			rotateRobot(-10);
		else if (getButtonPress(buttonUp))
			correctiveDrive(-30);
		else if (getButtonPress(buttonDown))
			returnToOrigin();
		else if (getButtonPress(buttonEnter))
			goto exit;
	}
	exit:
}



// -
void configureSensors()
{
	SensorType[TOUCH_PORT]=sensorEV3_Touch;
	wait1Msec(50);
	SensorType[ULTRASIDE_PORT]=sensorEV3_Ultrasonic;
	wait1Msec(50);
	SensorType[COLOR_PORT]=sensorEV3_Color;
	wait1Msec(50);
	SensorType[GYRO_PORT] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[GYRO_PORT] = modeEV3Gyro_Calibration;
	wait1Msec(50);
	SensorMode[GYRO_PORT] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
	return;
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
	int lastGyro = getGyroDegrees(GYRO_PORT);
	const float KP = 0.5;//0.26
	const float KI = 0.001;//0.0008
	const float KD = 0.01;//0.23
	const float TOLERANCE = 0.25;
	float error = angle - (getGyroDegrees(GYRO_PORT)-lastGyro);
	float mPower = 0;
	float prevError = 0;
	time1[T1] = 0;
	while (!getButtonPress(buttonEnter) && abs((getGyroDegrees(GYRO_PORT)-lastGyro) - angle) > TOLERANCE)
	{
		error = abs(angle - (getGyroDegrees(GYRO_PORT)-lastGyro));
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


int rotateAbsolute(int angle) //rotates robot in place to given angle then stops. Positive angles are clockwise when viewed from above
{
	const float KP = 0.5;//0.26
	const float KI = 0.001;//0.0008
	const float KD = 0.01;//0.23
	const float TOLERANCE = 0.25;
	float error = angle - (getGyroDegrees(GYRO_PORT));//
	float mPower = 0;
	float prevError = 0;
	time1[T1] = 0;// reset timer for PID loop
	while (!getButtonPress(buttonEnter) && abs((getGyroDegrees(GYRO_PORT)) - angle) > TOLERANCE)
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
void driveUltrasonic(int distance)
{

	nMotorEncoder[motorA] = 0;
	const float TOLERANCE = 0.5;
	float error = distance - nMotorEncoder[motorA]*TICK_TO_CM;
	int inverted = 1;
	float sensorDistance = 0;
	float threshold = SensorValue[ULTRASIDE_PORT] - 5;
	while (!getButtonPress(buttonEnter) && !(SensorValue[ULTRASIDE_PORT] < threshold))
	{
		sensorDistance = SensorValue[ULTRASIDE_PORT];
		if(abs(error) < TOLERANCE)
		{
			inverted *= -1;
		}

		error = distance - (nMotorEncoder[motorA]/TICK_TO_CM)*inverted;
		drive(20*inverted);
	}
	if(SensorValue[ULTRASIDE_PORT] < threshold)
	{
		sensorDistance = SensorValue[ULTRASIDE_PORT];
		drive(0);
		correctiveDrive(SENSOR_OFFSET + sin(ULTRA_DEG*DEG_TO_RAD)*sensorDistance);
		rotateRobot(-90);
		correctiveDrive(sensorDistance);
	}
	drive(0);
}


// -
void correctiveDrive(int distance)
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
	while (!getButtonPress(buttonEnter) && abs(error) > TOLERANCE && SensorValue[TOUCH_PORT] == 0)
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

	robotPos.x += cos((ANGLE)*DEG_TO_RAD)*-distance;
	robotPos.y += sin((ANGLE)*DEG_TO_RAD)*-distance;


	displayString(1, "x: %f", robotPos.x);
	displayString(2, "y: %f", robotPos.y);
	displayString(3, "xcur: %f", cos(ANGLE*DEG_TO_RAD)*distance);
	displayString(4, "ycur: %f", sin(ANGLE*DEG_TO_RAD)*distance);
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
	while (!getButtonPress(buttonEnter) && abs(error) > TOLERANCE && SensorValue[TOUCH_PORT] == 0)
	{
		error = distance - nMotorEncoder[MOTOR_LIFT]/TICK_TO_CM;
		mPower = KP*error + KI*((error+prevError)*(time1[T1] + 1)/2) + KD*abs(((error-prevError)/(time1[T1] + 1)));
		motor[MOTOR_LIFT] = mPower;
		prevError = error;
	}
	motor[MOTOR_LIFT] = 0;
}


// -
void returnToOrigin()
{
	rotateAbsolute(180);

	float angle = 0;
	angle = robotPos.x==0? 90 : atan(robotPos.y/robotPos.x)*RAD_TO_DEG;
	displayString(7, "angle: %f", angle);

	rotateRobot(angle);
	correctiveDrive( -sqrt(pow(robotPos.x, 2) + pow(robotPos.y, 2)) );

	rotateAbsolute(0);
}
