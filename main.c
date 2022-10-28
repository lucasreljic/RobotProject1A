int rotateRobot(int angle) //rotates robot in place to given angle then stops. Positive angles are clockwise when viewed from above
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
		driveBoth(-mPower, mPower);
	}
	else
	{
		driveBoth(mPower,-mPower);
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



}
