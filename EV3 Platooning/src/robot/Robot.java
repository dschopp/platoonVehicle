package robot;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.util.Date;
import java.util.Properties;

import tools.Lane;
import tools.Position;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

/**
 * The module which contains all capabilities of a Lego Mindstorms EV3
 * Platooning Vehicle
 * 
 * @author Martin
 *
 */
public class Robot implements PlatooningVehicle {

	/**
	 * The robot's name
	 */
	private String name;

	/**
	 * Enum used to indicate which line to follow REGULAR normal right lane EXIT
	 * following the exit ramp DONT_EXIT not following the next exit ramp
	 * OVERTAKE left lane for overtaking
	 */
	private enum LineFollowingMode {
		REGULAR, EXIT, DONT_EXIT, OVERTAKE;
	}

	/** The right motor */
	private UnregulatedMotor motorRight = new UnregulatedMotor(MotorPort.C);

	/** The left motor */
	private UnregulatedMotor motorLeft = new UnregulatedMotor(MotorPort.B);

	/** Right color sensor */
	private EV3ColorSensor colorSensorRight = new EV3ColorSensor(SensorPort.S3);

	/** Left color sensor (only instantiated if physically existent) */
	private EV3ColorSensor colorSensorLeft;

	/** Ultrasonic sensor for distance measurement */
	private EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(
			SensorPort.S4);

	/** Indicates whether the robot is lead vehicle of a platoon */
	private boolean isLead = true;

	/** Indicates whether the robot is part of a platoon */
	private boolean isInPlatoon = false;

	/** The current LineFollowingMode */
	private LineFollowingMode lineFollowingMode = LineFollowingMode.REGULAR;

	/** Indicates whether the robot shall access the next exit ramp */
	private boolean shallExit;

	/** Indicates the desired gap size between platoon members */
	private float gapSize;

	/** Indicates whether the robot shall terminate as soon as possible */
	private boolean shallStop = true;

	/** Indicates whether the robot shall change the current line */
	private boolean shallChangeLine = false;

	// Colors which are used for line following

	/** The default right color of the road (after sensor calibration) */
	private float defaultRightColor;

	/** The default left color of the road (after sensor calibration) */
	private float defaultLeftColor;

	/** The default right color of the exit ramps (after sensor calibration) */
	private float exitColor;

	/** Desired color to follow the line in an optimal way */
	private float lateralMidpoint;

	// Parameters used by the PID Controller for line following. Need to be
	// adjusted for every robot
	private float ki;
	private float kp;
	private float kd;

	/** Indicates whether the robot has two color sensors or just one */
	private boolean hasTwoColorSensors;

	/** The robots current (desired) velocity */
	private float currentVelocity;

	/** The robots standard velocity (defined by config File) */
	private static final float STANDARD_VELOCITY = 50;

	/** IP and port of the infrastructure unit */
	private String server_ip;
	private int server_port;

	/** Communication module for V2I communication */
	private V2ICommunicationModule v2icommunication;

	/** Communication module for V2V communication */
	private V2VCommunicationModule v2vcommunication;

	/** Indicates whether the robot shall leave the current platoon */
	private boolean shallLeavePlatoon;

	/** The number of green markers on the floor that the robot passed */
	private int markerCount = 0;

	/** The current position of the robot */
	private Position currentPosition = new Position();

	/** The current highway lane. default is right */
	private Lane currentLane = Lane.RIGHT;

	/** Indicates whether the robot shall terminate as fast as possible */
	private boolean shallTerminate = false;

	/**
	 * Indicates whether an emergency notification was sent (to avoid duplicate
	 * messages)
	 */
	private boolean emergencyNotificationSent = false;

	/**
	 * Standard constructor of a Lego EV3 platooning robot
	 * 
	 * @param settings
	 */
	public Robot(String configFileName) {

		// read configuration
		handleProperties(configFileName);

		// initiate second color sensor if present
		if (hasTwoColorSensors) {
			colorSensorLeft = new EV3ColorSensor(SensorPort.S2);
		}

		// calibrate light sensors
		calibrateLightSensor();

		// enable communication
		enableCommunication();

		// as long as no termination signals were received, the robot continues
		// following the highway
		while (!shallTerminate) {
			currentVelocity = STANDARD_VELOCITY;
			driveOnHighway();
		}

	}

	/**
	 * Initial method to read properties (a configuration) from a configuration
	 * file
	 * 
	 * @param configFileName
	 *            The file name of the configuration file
	 */
	private void handleProperties(String configFileName) {
		try {
			// load properties from config file
			Properties properties = new Properties();
			BufferedInputStream stream = new BufferedInputStream(
					new FileInputStream(configFileName));
			properties.load(stream);
			stream.close();

			// adjust parameters based on properties
			name = properties.getProperty("name");
			kp = Float.parseFloat(properties.getProperty("kp"));
			ki = Float.parseFloat(properties.getProperty("kp"));
			kd = Float.parseFloat(properties.getProperty("kp"));
			gapSize = Float.parseFloat(properties.getProperty("gapSize"));
			hasTwoColorSensors = Boolean.parseBoolean(properties
					.getProperty("hasTwoColorSensors"));
			server_ip = properties.getProperty("server_ip");
			server_port = Integer.parseInt(properties
					.getProperty("server_port"));

		} catch (Exception e) {
			System.out.println("Error: Not able to read properties file at "
					+ configFileName);
		}
	}

	/**
	 * Retrieves the robot's current position
	 * 
	 * @return The robot's current position
	 */
	public synchronized Position getPosition() {
		return this.currentPosition;
	}

	/**
	 * Sets the robots current position according to the parameters
	 * 
	 * @param markerNumber
	 *            The number of position markers which were passed
	 * @param additionalDistance
	 *            The additional distance which was traveled after passing the
	 *            last marker
	 */
	public synchronized void setPosition(int markerNumber,
			double additionalDistance) {
		currentPosition.setMarkerNumber(markerNumber);
		currentPosition.setAdditionalDistance(additionalDistance);
	}

	/**
	 * Retrieves the robot's name
	 * 
	 * @return The robot's name
	 */
	public String getName() {
		return name;
	}

	/**
	 * Sets the shallChangeLine attribute according to the parameter
	 * 
	 * @param shallChangeLine
	 *            The desired value for the shallChangeLine attribute
	 */
	public void setShallChangeLine(boolean shallChangeLine) {
		this.shallChangeLine = shallChangeLine;
	}

	/**
	 * Initial method to enable V2I and V2V communication
	 */
	private void enableCommunication() {
		System.out.println("Enable V2I communication...");
		enableV2ICommunication(server_ip, server_port);
		System.out.println("V2I communication enabled.");
		System.out.println("Enable V2V communication...");
		enableV2VCommunication();
		System.out.println("V2V communication enabled.");
	}

	/**
	 * Enable V2V communication
	 */
	private void enableV2VCommunication() {
		v2vcommunication = new V2VCommunicationModule(this);
	}

	/**
	 * Enable V2I communication
	 * 
	 * @param ip
	 *            The IP to connect to
	 * @param port
	 *            The port to connect to
	 */
	private void enableV2ICommunication(String ip, int port) {
		v2icommunication = new V2ICommunicationModule(this, server_ip,
				server_port);
		v2icommunication.start();
	}

	/**
	 * Terminate V2V communication (e.g., if the robot leaves a platoon)
	 */
	public void closeV2VCommunication() {
		v2vcommunication.close();

	}

	/**
	 * Calibrates the light sensor to follow the lines. Is needed to cope with
	 * different light conditions etc.
	 */
	private void calibrateLightSensor() {

		// calibrate right road side color
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

		// calibrate left road side color
		LCD.drawString("2. Press the ENTER", 0, 0);
		LCD.drawString("button to", 0, 1);
		LCD.drawString("calibrate left", 0, 2);
		LCD.drawString("road side color!", 0, 3);
		Button.ENTER.waitForPressAndRelease();
		colorTestMode.fetchSample(colorSample, 0);
		defaultLeftColor = colorSample[0];
		LCD.clear();

		// calibrate right exit ramp color
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

	/**
	 * Central method of the Lego EV3 platooning system. The robot follows the
	 * highway according to multiple parameters. Events like new detected color
	 * markers etc. are basically handled here.
	 */
	private void driveOnHighway() {

		// initiate color sensors
		SensorMode colorMode = null;
		float[] colorSample = null;
		if (hasTwoColorSensors) {
			colorMode = colorSensorLeft.getColorIDMode();
			colorSample = new float[colorMode.sampleSize()];
		}
		int currentColor = Color.NONE;
		SensorMode redMode = colorSensorRight.getRedMode();
		float[] redSample = new float[redMode.sampleSize()];

		// initiate distance sensor and samples
		SampleProvider distanceMode = usSensor.getDistanceMode();
		float[] distanceSample = new float[distanceMode.sampleSize()];

		// as long as the robot shall not stop
		while (!shallStop) {

			// if it has a second color sensor: read color markers
			if (hasTwoColorSensors) {
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
			}

			// if the line shall be changed: change line
			if (shallChangeLine) {
				changeLine();
			}

			// if the platoon shall be left: leave platoon
			if (shallLeavePlatoon) {
				leavePlatoon();
			}

			// brake if an obstacle is detected
			if (isLead) {
				distanceMode.fetchSample(distanceSample, 0);
				if (distanceSample[0] < 0.15) {
					currentVelocity = 0;
					if (isInPlatoon) {
						// send emergency notification
						if (!emergencyNotificationSent) {
							v2vcommunication.sendMessage(getName()
									+ ": EMERGENCY BRAKE", true);
							emergencyNotificationSent = true;
						}
					}
				} else {
					currentVelocity = STANDARD_VELOCITY;
					emergencyNotificationSent = false;
				}
			}

			// set the current position
			setPosition(markerCount, ((double) motorLeft.getTachoCount())
					/ 360D * Math.PI * 5.6D);

			// different actions based on the color markings which were observed
			switch (currentColor) {

			// if a position marking is detected: refresh position
			case Color.YELLOW:
			case Color.GREEN:
			case Color.NONE:
			case Color.BLUE:
				if (currentPosition.getAdditionalDistance() > 7) {
					setPosition(++markerCount, 0);
					motorLeft.resetTachoCount();
				}

				// if no marking was detected: follow the highway as regular
			case Color.BLACK:
				lineFollowingMode = LineFollowingMode.REGULAR;
				followLine(redMode, redSample);
				break;

			// if a red marking was detected and if the robot shall exit the
			// next ramp: start exiting highway
			case Color.RED:
				if (shallExit) {

					// send a leaving message to platoon
					if (isInPlatoon) {
						v2vcommunication.sendMessage(getName()
								+ ": I exit the highway now", false);
					}

					// adjust line following mode (to the color of the exit
					// ramp)
					lineFollowingMode = LineFollowingMode.EXIT;
					isLead = true;
				} else {
					lineFollowingMode = LineFollowingMode.DONT_EXIT;

				}

				// as long as no blue marking is detected: follow the exit ramp
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
			// refresh current position
			setPosition(markerCount, ((double) motorLeft.getTachoCount())
					/ 360D * Math.PI * 5.6D);

		}

		// if the robot shall stop: stop all motors
		motorLeft.stop();
		motorRight.stop();

	}

	/**
	 * The robot changes the current highway lane
	 */
	private void changeLine() {

		// send a message via V2V communication if in platoon
		if (isInPlatoon) {
			v2vcommunication.sendMessage(getName() + ": I change line", false);
		}
		SensorMode redMode = colorSensorRight.getRedMode();
		float[] redSample = new float[redMode.sampleSize()];
		SensorMode colorMode = colorSensorLeft.getColorIDMode();
		float[] colorSample = new float[colorMode.sampleSize()];
		redSample[0] = 100;
		int currentColor;

		// if the robot is currently on the right highway lane
		if (currentLane == Lane.RIGHT) {

			// turn around slightly (for 300 ms)
			motorLeft.setPower(20);
			motorRight.setPower(70);
			motorLeft.forward();
			motorRight.forward();
			long startManeuver = new Date().getTime();
			int i = 0;
			while (new Date().getTime() < startManeuver + 300) {

				// observe position markings
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
				currentPosition.setAdditionalDistance((double) motorLeft
						.getTachoCount() / 360 * Math.PI * 5.6);
				if (currentColor == Color.BLUE) {
					if (currentPosition.getAdditionalDistance() > 7) {
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();
					}
				}
				LCD.clear();
				LCD.drawString(Integer.toString(++i), 0, 0);
			}

			// driving straightforward until the left highway lane is reached
			motorLeft.setPower((int) currentVelocity);
			motorRight.setPower((int) currentVelocity);
			while (redSample[0] > defaultRightColor + 0.1
					|| redSample[0] < defaultRightColor - 0.1) {

				// observe position markings
				redMode.fetchSample(redSample, 0);
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
				currentPosition.setAdditionalDistance((double) motorLeft
						.getTachoCount() / 360 * Math.PI * 5.6);
				if (currentColor == Color.BLUE) {
					if (currentPosition.getAdditionalDistance() > 7) {
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();
					}
				}
			}

			// lane change successful, normal travelling
			currentLane = Lane.OVERTAKING;
			currentVelocity = STANDARD_VELOCITY + 20;

			// if the robot is currently on the left highway lane
		} else {

			// turn around slightly (for 300 ms)
			motorRight.setPower(20);
			motorLeft.setPower(70);
			motorLeft.forward();
			motorRight.forward();
			long startManeuver = new Date().getTime();
			while (new Date().getTime() < startManeuver + 300) {

				// observe position markings
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
				currentPosition.setAdditionalDistance(motorLeft.getTachoCount()
						/ 360 * Math.PI * 5.6);
				if (currentColor == Color.BLUE) {
					if (currentPosition.getAdditionalDistance() > 7) {
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();
					}
				}
			}

			// driving straightforward until the right highway lane is reached
			motorLeft.setPower((int) currentVelocity);
			motorRight.setPower((int) currentVelocity);
			while (redSample[0] > defaultRightColor + 0.1
					|| redSample[0] < defaultRightColor - 0.1) {
				redMode.fetchSample(redSample, 0);
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
				currentPosition.setAdditionalDistance((double) motorLeft
						.getTachoCount() / 360 * Math.PI * 5.6);
				if (currentColor == Color.BLUE) {
					if (currentPosition.getAdditionalDistance() > 7) {
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();
					}
				}
			}
			// lane change successful, normal travelling
			currentLane = Lane.RIGHT;
			currentVelocity = STANDARD_VELOCITY;
		}

		// reset attribute and send message to platoon that line changing was
		// finished
		shallChangeLine = false;
		if (isInPlatoon) {
			v2vcommunication.sendMessage(getName()
					+ ": I finished changing lines", false);
		}

	}

	/**
	 * Basic method to follow a line based on the parameters
	 * 
	 * @param redMode
	 *            The SensorMode object used for line following (sensor mode of
	 *            the right color sensor)
	 * @param redSample
	 *            The sample object of the SensorMode
	 */
	private void followLine(SensorMode redMode, float[] redSample) {

		// the distance measure
		SampleProvider distanceMode = usSensor.getDistanceMode();
		float[] distanceSample = new float[distanceMode.sampleSize()];

		int errorMultiplicator = 1;

		// calculate the desired lateral midpoint based on the current situation
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

		// gather new distance and color data
		redMode.fetchSample(redSample, 0);
		distanceMode.fetchSample(distanceSample, 0);

		// calculate lateral correction based on the midpoint
		lateralCorrection = calculateLateralCorrection(redSample[0],
				lateralMidpoint);

		// if the vehicle is not leading a platoon (or driving solo), it should
		// maintain the gap size
		if (!isLead) {
			longitudinalCorrection = calculateLongitudinalCorrection(
					distanceSample[0], gapSize);
		}

		// adjustment if the robot is currently on the left highway lane
		if (currentLane == Lane.OVERTAKING) {
			lateralCorrection = -lateralCorrection;
		}

		// finally calculate the needed power of the two motors
		int powerLeft = (int) (currentVelocity + errorMultiplicator
				* currentVelocity * lateralCorrection - longitudinalCorrection);
		motorLeft.setPower(powerLeft);
		int powerRight = (int) (currentVelocity - errorMultiplicator
				* currentVelocity * lateralCorrection - longitudinalCorrection);
		motorRight.setPower(powerRight);
		motorLeft.forward();
		motorRight.forward();
	}

	/**
	 * Calculates the lateral correction needed to follow a line in the optimal
	 * way
	 * 
	 * @param value
	 *            The current value of the right color sensor
	 * @param midpoint
	 *            The desired value of the right color sensor (lateral midpoint)
	 * @return The lateral correction needed to follow a line in the optimal way
	 */
	public float calculateLateralCorrection(float value, float midpoint) {

		// PID Controller based on the parameters Kp, Ki, and Kd
		float currentKp = kp;
		float currentKi = ki;
		float currentKd = kd;
		if (currentVelocity > 50) {
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
		correction = currentKp * error + currentKi * integral + currentKd
				* derivative;
		return correction;

	}

	/**
	 * Calculates speed adjustments (for both motors) needed to maintain the gap
	 * size to robot in front
	 * 
	 * @param value
	 *            The current distance to the robot in front
	 * @param midpoint
	 *            The desired distance to the robot in front (longitudinal
	 *            midpoint)
	 * @return The speed adjustment needed to maintain the desired gap size
	 */
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

	// methods which are required to implement the interface PlatooningVehicle
	// refer to the interface definition for details
	@Override
	public String toString() {
		return name;
	}

	@Override
	public void setVelocity(int velocity) {
		this.currentVelocity = velocity;
		System.out.println("Velocity set to: " + velocity);

	}

	@Override
	public void changeLine(Lane lane) {
		if (lane != currentLane) {
			this.shallChangeLine = true;
		}

	}

	@Override
	public void joinPlatoon(String platoonName) {
		v2vcommunication.joinGroup(platoonName);
		System.out
				.println("Joined V2V communication of platoon " + platoonName);
		v2vcommunication.sendMessage(getName() + ": Hello, I'm new!", false);
		if (v2vcommunication.getPlatoonSize() > 1) {
			isLead = false;
		} else {
			isLead = true;
		}
		isInPlatoon = true;
	}

	@Override
	public void leavePlatoon() {
		// TODO What is the desired scenario?
		if (isInPlatoon) {
			v2vcommunication.sendMessage(getName() + ": I leave the platoon",
					false);
			isInPlatoon = false;
			currentVelocity = STANDARD_VELOCITY;
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
		if (isInPlatoon) {
			v2vcommunication.sendMessage(getName() + ": I have to stop", false);
		}
		motorLeft.setPower(0);
		motorRight.setPower(0);
		shallStop = true;
		System.out.println("Stopped driving");

	}

	@Override
	public void exitNextRamp() {
		if (isInPlatoon) {
			v2vcommunication.sendMessage(getName() + ": I will exit next ramp",
					false);
		}
		shallExit = true;

	}

	@Override
	public void setGapSize(float gapSize) {
		this.gapSize = gapSize;
		System.out.println("Gap size set to " + gapSize);

	}

	@Override
	public boolean sendMessageToPlatoon(String message, boolean isOOB) {
		if (v2vcommunication == null) {
			return false;
		} else {
			v2vcommunication.sendMessage(message, isOOB);
			return true;
		}
	}

	/**
	 * Main method
	 * 
	 * @param args
	 *            args
	 */
	public static void main(String[] args) {

		// Standard config file is "config.txt"
		@SuppressWarnings("unused")
		Robot robot = new Robot("config.txt");
	}

}
