void configureAllSensors(); //configures all sensors to standard configuration
void drive(int motor_power); //powers both drive motors with the same power
void driveBoth (int motor_power_A, int motor_power_D); //powers both motors independently
void waitButton(TEV3Buttons button_name);  // wait for push and release of specified button
void drivePID(int distance);
void autoScan();

float angleAtScan = 0;
float ScanDistance = 0;

int rotateRobot(int angle, int power) //rotates robot in place to given angle then stops. Positive angles are clockwise when viewed from above
{
	resetGyro(S4);
	int lastGyro = getGyroDegrees(S4);
	const float kP = 0.25;
	const float kI = 0.0008;
	const float kD = 0.15;
	const float tolerance = 0.9;
	nMotorEncoder[motorA] = 0;
	float error = angle - getGyroDegrees(S4);
	displayString(5, "%f",error);
	float mPower = 0;
	float prevError = 0;
	time1[T1] = 0;
	while (!getButtonPress(buttonEnter) && abs(getGyroDegrees(S4) - angle) > tolerance)
	{
		error = angle - (getGyroDegrees(S4));
		mPower = kP*error + kI*((error+prevError)*(time1[T1] + 1)/2) + kD*abs(((error-prevError)/(time1[T1] + 1)));
		displayString(7, "%f",mPower);
		displayString(5, "%f",error);
		if (angle>0)
	{
		driveBoth(-mPower*power, mPower*power);
	}
	else
	{
		driveBoth(mPower*power,-mPower*power);
	}
		//wait1Msec(100);
		prevError = error;
	}

	drive(0);
	if(SensorValue[S1] == 1)
	{
		return abs(getGyroDegrees(S4));
	}
	else{
		return 90;
	}
}
void drivePID(int distance)
{
	const float kP = 0.85;
	const float kI = 0.005;
	const float kD = 0.05;
	const float TickToCM = 180/(PI*2.75);
	const float tolerance = 0.9;
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
task main()
{

	autoScan();


}

void configureAllSensors()
{
	SensorType[S1] = sensorEV3_Touch;
	SensorType[S2] = sensorEV3_Ultrasonic;
	SensorType[S3] = sensorEV3_Color;
	wait1Msec(50);
	SensorMode[S3] = modeEV3Color_Color;
	wait1Msec(50);

	SensorType[S4] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[S4] = modeEV3Gyro_Calibration;
	wait1Msec(100);
	SensorMode[S4] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
}

void drive(int motor_power)//powers both drive motors with the same power
{
	motor[motorA] = motor[motorD] = motor_power;
}

void driveBoth(int motor_power_A, int motor_power_D)//powers both motors independently
{
	motor[motorA] = motor_power_A;
	motor[motorD] = motor_power_D;
}

void waitButton(TEV3Buttons button_name)
{
	while(!getButtonPress(button_name))
	{}
	while(getButtonPress(button_name))
	{}
}
void autoScan()
{
	float lowestDist = 0;
	int angleAt = 0;
	const int power = 1;
	string output = "";
	for(int angle = 0; angle < 360; angle++)
	{
	rotateRobot(angle, power);
	float dist = SensorType[S2];
	if (dist < lowestDist)
	{
		lowestDist = dist;
		angleAt = SensorType[S4];
	}
	}
	angleAtScan = angleAt;
	ScanDistance = lowestDist;
	}
