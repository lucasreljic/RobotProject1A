

const int motorLeft = motorD;
const int motorRight = motorA;


void configureSensors();
int rotateRobot(int angle);
void drivePID(int distance);
void driveBoth(int pwrL, int pwrR);
void drive(int pwr);


task main()
{
	clearDebugStream();
	configureSensors();

	while (true)
	{
		while(!getButtonPress(buttonAny))
		{}

		if (getButtonPress(buttonLeft))
			rotateRobot(-90);
		else if (getButtonPress(buttonRight))
			rotateRobot(90);
		else if (getButtonPress(buttonUp))
			drivePID(50);
		else if (getButtonPress(buttonDown))
			drivePID(-50);
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


void drivePID(int distance)
{
	const float kP = 0.85;
	const float kI = 0.005;
	const float kD = 0.05;
	const float TickToCM = 180/(PI*2.75);
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


void driveBoth(int pwrL, int pwrR)
{
	motor[motorLeft] = pwrL;
	motor[motorRight] = pwrR;
}


void drive(int pwr)
{
	motor[motorLeft] = motor[motorRight] = pwr;
 }
