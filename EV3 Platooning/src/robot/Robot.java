package robot;

import java.util.Date;

import settings.Lane;
import settings.Position;
import settings.Settings;
import settings.Settings_1;
import settings.Settings_2;
import settings.Settings_3;
import lejos.hardware.Button;
import lejos.hardware.Key;
import lejos.hardware.KeyListener;
import lejos.hardware.lcd.LCD;
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
public class Robot implements PlatooningVehicle {

	/**
	 * Enum used to indicate which line to follow REGULAR normal right lane EXIT
	 * following the exit ramp DONT_EXIT not following the next exit ramp
	 * OVERTAKE left lane for overtaking
	 */
	private enum LineFollowingMode {
		REGULAR, EXIT, DONT_EXIT, OVERTAKE;
	}

	/** the two motors of the EV3 */
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
	
	/** Indicates whether the robot is part of a platoon */
	private boolean isInPlatoon = false;

	/** the current LineFollowingMode */
	private LineFollowingMode lineFollowingMode = LineFollowingMode.REGULAR;

	/** indicates whether the robot shall access the next exit ramp */
	private boolean shallExit;

	/** indicates the desired gap size between platoon members */
	private float gapSize;

	/** indicates whether the robot shall terminate as soon as possible */
	private boolean shallStop = true;

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

	/** indicates whether the robot has two color sensors or just one */
	private boolean hasTwoColorSensors;
	
	/**the robots velocity */
	private float velocity;
	
	/** IP and port of the infrastructure unit */
	private String server_ip;
	private int server_port;
	
	/** communication modules for V2I and V2V communication */
	private V2ICommunicationModule v2icommunication;
	private V2VCommunicationModule v2vcommunication;
	
	/** the settings instance which is used for the robot */
	private Settings settings;
	
	/**indicates whether the robot shall leave the current platoon */
	private boolean shallLeavePlatoon;
	
	/**the number of green markers on the floor that the robot passed */
	private int markerCount = 0;
	
	/** the current position of the robot */
	private Position currentPosition = new Position();
	
	/** the current highway lane. default is right */
	private Lane currentLane = Lane.RIGHT;
	
	/** indicates whether the robot shall terminate as fast as possible */
	private boolean shallTerminate = false;
	
	private boolean emergencyNotificationSent = false;

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
		Button.ESCAPE.addKeyListener(new TerminateButtonListener());
		calibrateLightSensor();

		
		enableCommunication();
		
		while(!shallTerminate){
			driveOnHighway();
		}
		
	}
	
	public synchronized Position getPosition(){
		return this.currentPosition;
	}
	
	public synchronized void setPosition(int markerNumber, double additionalDistance){
		currentPosition.setMarkerNumber(markerNumber);
		currentPosition.setAdditionalDistance(additionalDistance);
	}
	
	public String getName(){
		return this.settings.getName();
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
	
	private void enableCommunication(){
		server_ip = settings.getServerIP();
		server_port = settings.getServerPort();
		System.out.println("Enable V2I communication...");
		enableV2ICommunication(server_ip, server_port);
		System.out.println("V2I communication enabled.");
		System.out.println("Enable V2V communication...");
		enableV2VCommunication();
		System.out.println("V2V communication enabled.");
	}
	
	private void enableV2VCommunication(){
		v2vcommunication = new V2VCommunicationModule(this);
	}
	
	private void enableV2ICommunication(String ip, int port){
		v2icommunication = new V2ICommunicationModule(this, server_ip, server_port);
		v2icommunication.start();
	}
	
	

	/**
	 * calibrates the light sensor to follow the lines. is needed to cope with
	 * different light conditions etc
	 */
	public void calibrateLightSensor() {
		LCD.drawString("1. Press the ENTER", 0, 0);
		LCD.drawString("button to", 0, 1);
		LCD.drawString("calibrate right", 0, 2);
		LCD.drawString("road side color!", 0, 3);
		Button.ENTER.waitForPressAndRelease();
		SensorMode colorTestMode = colorSensorRight.getRedMode();
		float[] colorSample = new float[colorTestMode.sampleSize()];
		colorTestMode.fetchSample(colorSample, 0);
		defaultRightColor = colorSample[0];
		LCD.clear();

		LCD.drawString("2. Press the ENTER", 0, 0);
		LCD.drawString("button to", 0, 1);
		LCD.drawString("calibrate left", 0, 2);
		LCD.drawString("road side color!", 0, 3);
		Button.ENTER.waitForPressAndRelease();
		colorTestMode.fetchSample(colorSample, 0);
		defaultLeftColor = colorSample[0];
		LCD.clear();

		LCD.drawString("3. Press the ENTER ", 0, 0);
		LCD.drawString("button to", 0, 1);
		LCD.drawString("calibrate right", 0, 2);
		LCD.drawString("exit ramp color!", 0, 3);
		Button.ENTER.waitForPressAndRelease();
		colorTestMode.fetchSample(colorSample, 0);
		exitColor = colorSample[0];
		LCD.clear();
		System.out.println("Sensors calibrated");

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
			}
			if(shallLeavePlatoon){
				leavePlatoon();
			}

			//brake if an obstacle is detected
			if (isLead) {
				distanceMode.fetchSample(distanceSample, 0);
				if (distanceSample[0] < 0.15) {
					velocity = 0;
					if(isInPlatoon){
						//send emergency notification
						if(!emergencyNotificationSent){
							v2vcommunication.sendMessage(getName() + ": EMERGENCY BRAKE", true);
							emergencyNotificationSent = true;
						}
					}
				}
				else{
					velocity = settings.getVelocity();
					emergencyNotificationSent = false;
				}
			}
			setPosition(markerCount, ((double)motorLeft.getTachoCount()) / 360D * Math.PI * 5.6D);
			switch (currentColor) {
			case Color.YELLOW:			
			case Color.GREEN:			
			case Color.NONE:
			case Color.BLUE:
				if(currentPosition.getAdditionalDistance() >  7){
					setPosition(++markerCount, 0);
					motorLeft.resetTachoCount();
				}
			case Color.BLACK:
				lineFollowingMode = LineFollowingMode.REGULAR;
				followLine(redMode, redSample);
				break;
			case Color.RED:
				if (shallExit) {
					if(isInPlatoon){
						v2vcommunication.sendMessage(getName() + ": I exit the highway now", false);
					}
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
			//refresh current position
			setPosition(markerCount, ((double)motorLeft.getTachoCount()) / 360D * Math.PI * 5.6D);

		}
		motorLeft.stop();
		motorRight.stop();

	}

//	/** the robot leaves the platoon by reducing its speed until a gap size of 50 cm */
//	private void leavePlatoon() {
//		velocity = velocity - 20;
//		isLead = false;
//		
//		SampleProvider distanceMode = usSensor.getDistanceMode();
//		float[] distanceSample = new float[distanceMode.sampleSize()];
//		
//		SensorMode redMode = colorSensorRight.getRedMode();
//		float[] redSample = new float[redMode.sampleSize()];
//		
//		float distance = 0;
//		while(distance < 1){
//			followLine(redMode, redSample);
//			distanceMode.fetchSample(distanceSample, 0);
//			distance = distanceSample[0];
//		}
//		velocity = velocity + 20;
//		shallLeavePlatoon = false;
//		isLead = true;
//		
//	}

	/** the robot changes the current highway lane */
	private void changeLine() {
		if(isInPlatoon){
			v2vcommunication.sendMessage(getName() + ": I change line", false);
		}
		SensorMode redMode = colorSensorRight.getRedMode();
		float[] redSample = new float[redMode.sampleSize()];
		SensorMode colorMode = colorSensorLeft.getColorIDMode();
		float[] colorSample = new float[colorMode.sampleSize()];
		redSample[0] = 100;
		int currentColor;
		if (currentLane == Lane.RIGHT) {
			motorLeft.setPower(20);
			motorRight.setPower(70);
			motorLeft.forward();
			motorRight.forward();
			long startManeuver = new Date().getTime();
			int i = 0;
			while(new Date().getTime() < startManeuver + 300){
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
				currentPosition.setAdditionalDistance((double)motorLeft.getTachoCount() / 360 * Math.PI * 5.6);
				if(currentColor == Color.BLUE){
					if(currentPosition.getAdditionalDistance() >  7){
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();
					}
				}
				LCD.clear();
				LCD.drawString(Integer.toString(++i), 0, 0);
			}
			motorLeft.setPower((int) settings.getVelocity());
			motorRight.setPower((int) settings.getVelocity());
			while (redSample[0] > defaultRightColor + 0.1
					|| redSample[0] < defaultRightColor - 0.1) {
				redMode.fetchSample(redSample, 0);
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
				currentPosition.setAdditionalDistance((double)motorLeft.getTachoCount() / 360 * Math.PI * 5.6);
				if(currentColor == Color.BLUE){
					if(currentPosition.getAdditionalDistance() >  7){
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();
					}
				}
			}
			currentLane = Lane.OVERTAKING;
			velocity = settings.getVelocity() + 20;
		} else {
			motorRight.setPower(20);
			motorLeft.setPower(70);
			motorLeft.forward();
			motorRight.forward();
			long startManeuver = new Date().getTime();
			while(new Date().getTime() < startManeuver + 300){
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
				currentPosition.setAdditionalDistance(motorLeft.getTachoCount() / 360 * Math.PI * 5.6);
				if(currentColor == Color.BLUE){
					if(currentPosition.getAdditionalDistance() >  7){
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();
					}
				}
			}
			motorLeft.setPower((int) settings.getVelocity());
			motorRight.setPower((int) settings.getVelocity());
			while (redSample[0] > defaultRightColor + 0.1
					|| redSample[0] < defaultRightColor - 0.1) {
				redMode.fetchSample(redSample, 0);
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
				currentPosition.setAdditionalDistance((double)motorLeft.getTachoCount() / 360 * Math.PI * 5.6);
				if(currentColor == Color.BLUE){
					if(currentPosition.getAdditionalDistance() >  7){
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();
					}
				}
			}
			currentLane = Lane.RIGHT;
			velocity = settings.getVelocity();
		}
		shallChangeLine = false;
		if(isInPlatoon){
			v2vcommunication.sendMessage(getName() + ": I finished changing lines", false);
		}

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
		if (currentLane == Lane.OVERTAKING) {
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
	class TerminateButtonListener implements KeyListener {
		@Override
		public void keyPressed(Key k) {
			shallTerminate = true;
		}

		@Override
		public void keyReleased(Key k) {
			// Do nothing

		}
	}


	public void clearDisplay() {
		LCD.clear();
		
	}
	
	public String toString(){
		return settings.getName();
	}

	@Override
	public void setVelocity(int velocity) {
		this.velocity = velocity;
		System.out.println("Velocity set to: " + velocity);
		
	}

	@Override
	public void changeLine(Lane lane) {
		if(lane != currentLane){
			this.shallChangeLine = true;
		}
		
	}

	@Override
	public void joinPlatoon(String platoonName) {
		v2vcommunication.joinGroup(platoonName);
		System.out.println("Joined V2V communication of platoon " + platoonName);
		v2vcommunication.sendMessage(getName() + ": Hello, I'm new!", false);
		if(v2vcommunication.getPlatoonSize() > 1){
			isLead = false;
		}
		else{
			isLead = true;
		}	
		isInPlatoon = true;
	}

	@Override
	public void leavePlatoon() {
		// TODO What is the desired scenario?
		if(isInPlatoon){
			v2vcommunication.sendMessage(getName() + ": I leave the platoon", false);
			isInPlatoon = false;
			velocity = 50;
			System.out.println("Left the platoon.");
		}
	}

	@Override
	public void startDriving() {
		shallStop = false;
		System.out.println("Started driving");
		
	}

	@Override
	public void stopDriving() {
		if(isInPlatoon){
			v2vcommunication.sendMessage(getName() + ": I have to stop", false);
		}
		motorLeft.setPower(0);
		motorRight.setPower(0);
		shallStop = true;
		System.out.println("Stopped driving");
		
	}

	@Override
	public void exitNextRamp() {
		if(isInPlatoon){
			v2vcommunication.sendMessage(getName() + ": I will exit next ramp", false);
		}
		shallExit = true;
		
	}

	@Override
	public void setGapSize(float gapSize) {
		this.gapSize = gapSize;
		System.out.println("Gap size set to " + gapSize);
		
	}
	
	@Override
	public boolean sendMessageToPlatoon(String message, boolean isOOB){
		if(v2vcommunication == null ){
			return false;
		}
		else{
			v2vcommunication.sendMessage(message, isOOB);
			return true;
		}	
	}

	public void closeV2VCommunication() {
		v2vcommunication.close();
		
	}
	
	public static void main(String[] args){
		Robot robot = new Robot(new Settings_1());
	}

}
