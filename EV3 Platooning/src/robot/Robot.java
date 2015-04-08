package robot;

import settings.Settings;
import settings.Settings_1;
import settings.Settings_2;
import settings.Settings_3;
import lejos.hardware.Button;
import lejos.hardware.Key;
import lejos.hardware.KeyListener;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

/** class representing the robot and all its capabilities */
public class Robot {

	/**
	 * Enum used to indicate which line to follow REGULAR normal right lane EXIT
	 * following the exit ramp DONT_EXIT not following the next exit ramp
	 * OVERTAKE left lane for overtaking
	 */
	private enum LineFollowingMode {
		REGULAR, EXIT, DONT_EXIT, OVERTAKE;
	}

	private enum LowDistanceReaction {
		KEEP_DISTANCE, OVERTAKE;
	}

	/** the two motors of the EV3 */
	//private EV3LargeRegulatedMotor motorLeft = new EV3LargeRegulatedMotor(MotorPort.B);
	//private EV3LargeRegulatedMotor motorRight = new EV3LargeRegulatedMotor(MotorPort.C);
	private UnregulatedMotor motorRight = new UnregulatedMotor(MotorPort.C);
	private UnregulatedMotor motorLeft = new UnregulatedMotor(MotorPort.B);

	/** color sensors */
	private EV3ColorSensor colorSensorRight = new EV3ColorSensor(SensorPort.S3);
	private EV3ColorSensor colorSensorLeft;

	/** ultrasonic sensor for distance measurement */
	private EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(
			SensorPort.S4);

	/** indicates whether the robot is lead vehicle of a platoon */
	private boolean isLead;

	/** the current LineFollowingMode */
	private LineFollowingMode lineFollowingMode = LineFollowingMode.REGULAR;

	/** indicates whether the robot shall access the next exit ramp */
	private boolean shallExit;

	/** indicates the desired gap size between platoon members */
	private float gapSize;

	/** indicates whether the robot shall terminate as soon as possible */
	private boolean shallStop = false;

	/**
	 * indicates whether the robot is currently overtaking (driving on the left
	 * lane)
	 */
	private boolean isOvertaking = false;

	/** indicates whether the robot shall change the current line */
	private boolean shallChangeLine = false;

	/** colors which are used for line following */
	private float defaultRightColor;
	private float defaultLeftColor;
	private float exitColor;

	/** desired color to follow the line in an optimal way */
	private float lateralMidpoint;

	/**
	 * parameters used by the PID Controller for line following. need to be
	 * adjusted for every robot
	 */
	private float ki;
	private float kp;
	private float kd;


	private boolean hasTwoColorSensors;
	private float velocity;
	
	private String server_ip;
	private int server_port;
	
	private RobotCommunication communication;
	private Settings settings;
	private boolean shallLeavePlatoon;
	private int markerCount = 0;
	private int distanceLeft;
	private int distanceRight;

	/**
	public void testSpeed(int speed){
		while(!closeToWall()){
			motorLeftTest.setPower(speed);
			motorRightTest.setPower(speed);
			motorLeftTest.forward();
			motorRightTest.forward();	
		}
		motorLeftTest.stop();
		motorRightTest.stop();
	}
	*/

	/**
	 * constructor
	 * 
	 * @param settings
	 *            the settings for the robot
	 */
	public Robot(Settings settings) {
		this.settings = settings;
		adjustSettings(settings);
		if (hasTwoColorSensors) {
			colorSensorLeft = new EV3ColorSensor(SensorPort.S2);
		}
		Button.ESCAPE.addKeyListener(new StopButtonListener());
		Button.UP.addKeyListener(new OvertakeListener());
		Button.DOWN.addKeyListener(new TestListener());
		
	}
	
	public void setShallChangeLine(boolean shallChangeLine){
		this.shallChangeLine = shallChangeLine;
	}

	public void adjustSettings(Settings settings) {
		this.isLead = settings.isLead();
		shallExit = settings.shallExit();
		gapSize = settings.getGapSize();
		hasTwoColorSensors = settings.hasTwoColorSensors();
		kp = settings.getKp();
		ki = settings.getKi();
		kd = settings.getKd();
		velocity = settings.getVelocity();

	}
	
	public void enableCommunication(Settings settings){
		server_ip = settings.getServerIP();
		server_port = settings.getServerPort();
		
		communication = new RobotCommunication(this, server_ip, server_port);
		communication.start();
		
	}

	/**
	 * calibrates the light sensor to follow the lines. is needed to cope with
	 * different light conditions etc
	 */
	public void calibrateLightSensor() {
		LCD.drawString("Press the ENTER", 0, 0);
		LCD.drawString("button to", 0, 1);
		LCD.drawString("calibrate right", 0, 2);
		LCD.drawString("road side color!", 0, 3);
		Button.ENTER.waitForPressAndRelease();
		SensorMode colorTestMode = colorSensorRight.getRedMode();
		float[] colorSample = new float[colorTestMode.sampleSize()];
		colorTestMode.fetchSample(colorSample, 0);
		defaultRightColor = colorSample[0];
		LCD.clear();

		LCD.drawString("Now press the ENTER", 0, 0);
		LCD.drawString("button to", 0, 1);
		LCD.drawString("calibrate left", 0, 2);
		LCD.drawString("road side color!", 0, 3);
		Button.ENTER.waitForPressAndRelease();
		colorTestMode.fetchSample(colorSample, 0);
		defaultLeftColor = colorSample[0];
		LCD.clear();

		LCD.drawString("Press the ENTER ", 0, 0);
		LCD.drawString("button to", 0, 1);
		LCD.drawString("calibrate right", 0, 2);
		LCD.drawString("exit ramp color!", 0, 3);
		Button.ENTER.waitForPressAndRelease();
		colorTestMode.fetchSample(colorSample, 0);
		exitColor = colorSample[0];
		LCD.clear();

	}
	
	public void drawString(String message){
		LCD.drawString(message, 0, 0);
	}

	/**
	 * lets the robot follow the line like a highway according to parameters
	 * like isLead, shallExit etc.
	 */
	public void driveOnHighway() {
		SensorMode colorMode = null;
		float[] colorSample = null;
		if (hasTwoColorSensors) {
			colorMode = colorSensorLeft.getColorIDMode();
			colorSample = new float[colorMode.sampleSize()];
		}
		int currentColor = Color.NONE;

		SensorMode redMode = colorSensorRight.getRedMode();
		float[] redSample = new float[redMode.sampleSize()];

		SampleProvider distanceMode = usSensor.getDistanceMode();
		float[] distanceSample = new float[distanceMode.sampleSize()];

		while (!shallStop) {
			if (hasTwoColorSensors) {
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
			}
			if (shallChangeLine) {
				changeLine();
				LCD.clear();
				LCD.drawString(Integer.toString(motorLeft.getTachoCount()), 0, 0);
			}
			if(shallLeavePlatoon){
				leavePlatoon();
			}

//			if (isLead) {
//				distanceMode.fetchSample(distanceSample, 0);
//				if (distanceSample[0] < 0.25) {
//					changeLine();
//				}
//			}

			switch (currentColor) {
			case Color.YELLOW:
				if(distanceLeft > 100 && distanceRight > 100){
					motorLeft.resetTachoCount();
					motorRight.resetTachoCount();
					markerCount++;
				}
				
			case Color.GREEN:		
				distanceLeft = motorLeft.getTachoCount();
				distanceRight = motorRight.getTachoCount();
			case Color.NONE:
			case Color.BLUE:
			case Color.BLACK:
				lineFollowingMode = LineFollowingMode.REGULAR;
				followLine(redMode, redSample);
				break;
			case Color.RED:
				if (shallExit) {
					lineFollowingMode = LineFollowingMode.EXIT;
					isLead = true;
				} else {
					lineFollowingMode = LineFollowingMode.DONT_EXIT;

				}
				while (currentColor != Color.BLUE) {
					if (hasTwoColorSensors) {
						colorMode.fetchSample(colorSample, 0);
						currentColor = (int) colorSample[0];
					}
					followLine(redMode, redSample);

				}
				shallExit = false;
				break;
			}

		}
		motorLeft.stop();
		motorRight.stop();
	}

	/** the robot leaves the platoon by reducing its speed until a gap size of 50 cm */
	private void leavePlatoon() {
		velocity = velocity - 20;
		isLead = false;
		
		SampleProvider distanceMode = usSensor.getDistanceMode();
		float[] distanceSample = new float[distanceMode.sampleSize()];
		
		SensorMode redMode = colorSensorRight.getRedMode();
		float[] redSample = new float[redMode.sampleSize()];
		
		float distance = 0;
		while(distance < 1){
			followLine(redMode, redSample);
			distanceMode.fetchSample(distanceSample, 0);
			distance = distanceSample[0];
		}
		velocity = velocity + 20;
		shallLeavePlatoon = false;
		isLead = true;
		
	}

	/** the robot changes the current highway lane */
	private void changeLine() {
		SensorMode colorMode = colorSensorRight.getRedMode();
		float[] colorSample = new float[colorMode.sampleSize()];
		colorSample[0] = 100;
		if (!isOvertaking) {
			motorLeft.setPower(20);
			motorRight.setPower(70);
			motorLeft.forward();
			motorRight.forward();
			Delay.msDelay(300);
			motorLeft.setPower(70);
			while (colorSample[0] > defaultRightColor + 0.1
					|| colorSample[0] < defaultRightColor - 0.1) {
				colorMode.fetchSample(colorSample, 0);
			}
			isOvertaking = true;
			velocity = settings.getVelocity() + 20;
		} else {
			motorRight.setPower(20);
			motorLeft.setPower(70);
			motorLeft.forward();
			motorRight.forward();
			Delay.msDelay(300);
			motorRight.setPower(70);
			while (colorSample[0] > defaultRightColor + 0.1
					|| colorSample[0] < defaultRightColor - 0.1) {
				colorMode.fetchSample(colorSample, 0);
			}
			isOvertaking = false;
			velocity = settings.getVelocity();
		}
		shallChangeLine = false;

	}

	/** basic method to follow a line */
	private void followLine(SensorMode redMode, float[] redSample) {
		SampleProvider distanceMode = usSensor.getDistanceMode();
		float[] distanceSample = new float[distanceMode.sampleSize()];

		int errorMultiplicator = 1;
		if (lineFollowingMode == LineFollowingMode.REGULAR) {
			lateralMidpoint = (defaultRightColor - defaultLeftColor) / 2
					+ defaultLeftColor;
		} else if (lineFollowingMode == LineFollowingMode.EXIT) {
			lateralMidpoint = (defaultRightColor - exitColor) / 2 + exitColor;
			errorMultiplicator = -1;
		} else if (lineFollowingMode == LineFollowingMode.DONT_EXIT) {
			lateralMidpoint = (defaultLeftColor - exitColor) / 2 + exitColor;
		}
		float lateralCorrection = 0;
		float longitudinalCorrection = 0;

		redMode.fetchSample(redSample, 0);
		distanceMode.fetchSample(distanceSample, 0);
		lateralCorrection = calculateLateralCorrection(redSample[0],
				lateralMidpoint);
		if (!isLead) {
			longitudinalCorrection = calculateLongitudinalCorrection(
					distanceSample[0], gapSize);
		}
		if (isOvertaking) {
			lateralCorrection = -lateralCorrection;
		}
		int powerLeft = (int) (velocity + errorMultiplicator * velocity * lateralCorrection - longitudinalCorrection);
		motorLeft.setPower(powerLeft);
		int powerRight = (int) (velocity - errorMultiplicator * velocity
				* lateralCorrection - longitudinalCorrection);
		motorRight.setPower(powerRight);
		motorLeft.forward();
		motorRight.forward();
	}

	/**
	 * calculates the lateral correction needed to follow the line in an optimal
	 * way
	 */
	public float calculateLateralCorrection(float value, float midpoint) {
		float currentKp = kp;
		float currentKi = ki;
		float currentKd = kd;
		if(velocity > 50){
			currentKp = 2F;
			currentKi = 0;
			currentKd = 0;
		}
		float lastError = 0;
		float error;
		float integral = 0;
		float derivative;
		float correction;
		error = midpoint - value;
		integral = error + integral;
		derivative = error - lastError;
		correction = currentKp * error + currentKi * integral + currentKd * derivative;
		return correction;

	}

	/** calculates speed adjustments for optimal gap keeping */
	public float calculateLongitudinalCorrection(float value, float midpoint) {
		float k = 400;
		float error;
		float correction;
		error = midpoint - value;
		correction = k * error;
		if (correction > 50) {
			return 50;
		} else if (correction < -20) {
			return -20;
		} else {
			return correction;
		}

	}

	/**
	 * indicates whether the robot is close to a wall (less than 10 cm distance)
	 */
	public boolean closeToWall() {
		SampleProvider distanceMode = usSensor.getDistanceMode();
		float[] sample = new float[distanceMode.sampleSize()];
		distanceMode.fetchSample(sample, 0);
		return sample[0] < 0.1;
	}

	/** listener used to stop the robot if ESCAPE button is pressed */
	class StopButtonListener implements KeyListener {
		@Override
		public void keyPressed(Key k) {
			shallStop = true;
		}

		@Override
		public void keyReleased(Key k) {
			// Do nothing

		}
	}

	/** listener used to change lines if UP button is pressed */
	class OvertakeListener implements KeyListener {
		@Override
		public void keyPressed(Key k) {
			shallChangeLine = true;
		}

		@Override
		public void keyReleased(Key k) {
			// Do nothing

		}
	}
	
	/** listener used to change lines if UP button is pressed */
	class TestListener implements KeyListener {
		@Override
		public void keyPressed(Key k) {
			LCD.clear();
			LCD.drawString(Integer.toString(distanceLeft), 0, 0);
			LCD.drawString(Integer.toString(distanceRight), 0, 1);
			LCD.drawString(Integer.toString(markerCount), 0, 2);
		}

		@Override
		public void keyReleased(Key k) {
			// Do nothing

		}
	}

	public void setShallExit(boolean shallExit) {
		this.shallExit = shallExit;
		
	}

	public void clearDisplay() {
		LCD.clear();
		
	}
	
	public String toString(){
		return settings.getName();
	}

	public void setShallStop(boolean shallStop) {
		this.shallStop = true;
		
	}

	public void setIsLead(boolean isLead) {
		this.isLead = isLead;
		
	}

	public void setShallLeavePlatoon(boolean shallLeavePlatoon) {
		this.shallLeavePlatoon = shallLeavePlatoon;
		
	}

}
