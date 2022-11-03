

const int MOTOR_LEFT = motorD;
const int MOTOR_RIGHT = motorA;
const int MAX_POWER = 70;

void configureSensors();
int rotateRobot(int angle);
void drivePID(int distance);
void correctiveDrive(int distance);

void driveBoth(int pwrL, int pwrR);
void drive(int pwr);
int rotateAbsolute(int angle);


task main()
{
	clearDebugStream();
	configureSensors();

	while (true)
	{
		while(!getButtonPress(buttonAny))
		{}

		if (getButtonPress(buttonLeft))
			rotateAbsolute(-90);
		else if (getButtonPress(buttonRight))
			rotateAbsolute(90);
		else if (getButtonPress(buttonUp))
			correctiveDrive(-150);
		else if (getButtonPress(buttonDown))
			correctiveDrive(150);
		else if (getButtonPress(buttonEnter))
			goto exit;
	}
	exit:
}




void configureSensors()
{
	SensorType[S1]=sensorEV3_Touch;
	wait1Msec(50);
	SensorType[S2]=sensorEV3_Ultrasonic;
	wait1Msec(50);
	SensorType[S3]=sensorEV3_Color;
	wait1Msec(50);
	SensorType[S4] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[S4] = modeEV3Gyro_Calibration;
	wait1Msec(50);
	SensorMode[S4] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
	return;
}

int rotateRobot(int angle) //rotates robot in place to given angle then stops. Positive angles are clockwise when viewed from above
{
	int lastGyro = getGyroDegrees(S4);
	const float kP = 0.5;//0.26
	const float kI = 0.001;//0.0008
	const float kD = 0.01;//0.23
	const float tolerance = 0.25;
	float error = angle - (getGyroDegrees(S4)-lastGyro);
	float mPower = 0;
	float prevError = 0;
	time1[T1] = 0;
	while (!getButtonPress(buttonEnter) && abs((getGyroDegrees(S4)-lastGyro) - angle) > tolerance)
	{
		error = abs(angle - (getGyroDegrees(S4)-lastGyro));
		mPower = kP*error + kI*((error+prevError)*(time1[T1] + 1)/2) + kD*abs(((error-prevError)/(time1[T1] + 1)));
		if (angle>0)
		{
			driveBoth(-mPower, mPower);
		}
		else
		{
			driveBoth(mPower,-mPower);
		}
		prevError = error;
		displayString(5, "%f",getGyroDegrees(S4));
	}
	drive(0);
	return abs(getGyroDegrees(S4));
}
int rotateAbsolute(int angle) //rotates robot in place to given angle then stops. Positive angles are clockwise when viewed from above
{
	const float kP = 0.5;//0.26
	const float kI = 0.001;//0.0008
	const float kD = 0.01;//0.23
	const float tolerance = 0.25;
	float error = angle - (getGyroDegrees(S4));//
	float mPower = 0;
	float prevError = 0;
	time1[T1] = 0;// reset timer for PID loop
	while (!getButtonPress(buttonEnter) && abs((getGyroDegrees(S4)) - angle) > tolerance)
	{
		error = angle - getGyroDegrees(S4);// error for turn PID
		mPower = kP*error + kI*((error+prevError)*(time1[T1] + 1)/2) + kD*abs(((error-prevError)/(time1[T1] + 1)));// turn PID calculation
		driveBoth(-mPower, mPower);// turn motors based on motor power from PID with one being negative
		prevError = error;// previous error for PID
		displayString(5, "%f",getGyroDegrees(S4));
	}
	drive(0);
	return abs(getGyroDegrees(S4));
}

void drivePID(int distance)
{
	const float kP = 0.85;
	const float kI = 0.005;
	const float kD = 0.05;
	const float TickToCM = 180/(PI*1.6);
	const float tolerance = 0.5;
	nMotorEncoder[motorA] = 0;
	float error = distance - nMotorEncoder[motorA]*TickToCM;
	displayString(5, "%f",error);
	float mPower = 0;
	float prevError = 0;
	time1[T1] = 0;
	while (!getButtonPress(buttonEnter) && abs(error) > tolerance && SensorValue[S1] == 0)
	{
		error = distance - nMotorEncoder[motorA]/TickToCM;
		mPower = kP*error + kI*((error+prevError)*(time1[T1] + 1)/2) + kD*abs(((error-prevError)/(time1[T1] + 1)));
		displayString(7, "%f",mPower);
		displayString(5, "%f",error);
		drive(mPower);
		prevError = error;
	}
	drive(0);
}

void correctiveDrive(int distance)
{
	//Driving PID Constants
	const float kP = 1;
	const float kI = 0.001;
	const float kD = 0.02;
	//Turn PID Constants
	const float turnkP = 0.5;//0.26
	const float turnkI = 0.0005;//0.0008
	const float turnkD = 0.01;//0.23
	float angle = getGyroDegrees(S4);//get angle before driving starts
	float turnError = 0;
	float mTurnPower = 0;
	float turnPrevError = 0;
	const float TickToCM = 180/(PI*1.6);//Encoder ticks to CM calculation
	const float tolerance = 0.5;
	nMotorEncoder[MOTOR_RIGHT] = 0;
	nMotorEncoder[MOTOR_LEFT] = 0;
	float error = distance;
	displayString(5, "%f",error);
	float mPower = 0;
	float prevError = 0;
	time1[T1] = 0;// start timer for PID loop
	while (!getButtonPress(buttonEnter) && abs(error) > tolerance && SensorValue[S1] == 0)
	{
		turnError = angle - getGyroDegrees(S4);// Turn PID error
		mTurnPower = turnkP*turnError + turnkI*((turnError+turnPrevError)*(time1[T1] + 1)/2) + turnkD*(turnError-turnPrevError);// turn pid calculation
		error = distance - ((nMotorEncoder[MOTOR_RIGHT] + nMotorEncoder[MOTOR_LEFT])/2)/TickToCM;// Drive PID error

		mPower = kP*error + kI*((error+prevError)*(time1[T1] + 1)/2) + kD*abs(((error-prevError)/(time1[T1] + 1)));// drive PID calculation
		displayString(7, "%f",mPower);
		if(mPower > MAX_POWER)
		{
			mPower = MAX_POWER;
		}
		driveBoth(mPower -mTurnPower, mPower +mTurnPower);//add turn power to drive power to adjust for
		displayString(5, "%f",getGyroDegrees(S4));// printing Gyro Degrees
		prevError = error;// for PID
		turnPrevError = turnError;// for Turn PID
	}
	rotateAbsolute(angle);
	drive(0);// stop motors
}
void driveBoth(int pwrL, int pwrR)
{
	motor[MOTOR_LEFT] = pwrL;
	motor[MOTOR_RIGHT] = pwrR;
}


void drive(int pwr)
{
	motor[MOTOR_LEFT] = motor[MOTOR_RIGHT] = pwr;
}
