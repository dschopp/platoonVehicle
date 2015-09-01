package de.wifo2.platooning.robot;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.util.Date;
import java.util.Properties;

import de.wifo2.platooning.communication.V2ICommunicationModule;
import de.wifo2.platooning.communication.V2VCommunicationModule;
import de.wifo2.platooning.protocol.Protocol;
import de.wifo2.platooning.utils.ExitRamp;
import de.wifo2.platooning.utils.Lane;
import de.wifo2.platooning.utils.Map;
import de.wifo2.platooning.utils.Position;
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
	private int number;

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
	private boolean shallStop = false;

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

	private float velocityAfterLineChange = 0;

	/** The robots standard velocity (defined by config File) */
	private static float START_VELOCITY = 50;

	/** IP and port of the infrastructure unit */
	private String server_ip;
	private int server_port;

	/** Communication module for V2I communication */
	private V2ICommunicationModule v2icommunication;

	/** Communication module for V2V communication */
	private V2VCommunicationModule v2vcommunication;

	/** Indicates whether the robot shall leave the current platoon */
	private boolean shallLeavePlatoon;

	/** The number of position markers on the floor that the robot passed */
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
	 * Represents the robot's IP address
	 */
	private String robot_ip;

	/**
	 * The robot's platoon number. 0, if the robot does not belong to a platoon
	 */
	private int platoonNumber = 0;

	/**
	 * The command which shall be sent to the infrastructure in the next message
	 */
	private int command;

	/**
	 * The data form which shall be used in the next message sent to the
	 * infrastructure
	 */
	private int dataForm;
	
	private int desiredExit = 0;
	
	private int destination = 0;
	
	private Map map = new Map();
	
	private boolean v2vCommunicationEnabled = true;

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
		if(v2vCommunicationEnabled){
			enableV2VCommunication();
		}

		// as long as no termination signals were received, the robot continues
		// following the highway
		while (!shallTerminate) {
				if (v2icommunication == null || !v2icommunication.getHasReceivedVelocity()) {
					currentVelocity = START_VELOCITY;
				}
			
			driveOnHighway();
		}
		v2icommunication.closeCommunication();

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
			number = Integer.parseInt(properties.getProperty("name"));
			kp = Float.parseFloat(properties.getProperty("kp"));
			ki = Float.parseFloat(properties.getProperty("ki"));
			kd = Float.parseFloat(properties.getProperty("kd"));
			START_VELOCITY = Float.parseFloat(properties
					.getProperty("velocity"));
			gapSize = Float.parseFloat(properties.getProperty("gapSize"));
			hasTwoColorSensors = Boolean.parseBoolean(properties
					.getProperty("hasTwoColorSensors"));
			server_ip = properties.getProperty("server_ip");
			server_port = Integer.parseInt(properties
					.getProperty("server_port"));
			robot_ip = properties.getProperty("robot_ip");
			destination = Integer.parseInt(properties.getProperty("destination"));
			v2vCommunicationEnabled = Boolean.parseBoolean(properties.getProperty("v2vCommunicationEnabled"));

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
	public int getVehicleNumber() {
		return number;
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
	 * Initial method to enable V2V communication
	 */
	private void enableV2VCommunication() {
		System.out.println("Enable V2V communication...");
		v2vcommunication = new V2VCommunicationModule(this);
		System.out.println("V2V communication enabled.");

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
		v2icommunication.startCommunication();
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
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
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
					System.out.println("Obstacle detected --> brake");
					currentVelocity = 0;

					if (isInPlatoon && v2vCommunicationEnabled) {
						System.out.println("inPlatoon --> send EMERGENCY");
						// send emergency notification
						if (!emergencyNotificationSent) {
							v2vcommunication.sendMessage(getVehicleNumber()
									+ ": EMERGENCY BRAKE", true);
							emergencyNotificationSent = true;
						}
					}
				} else {
					// currentVelocity = STANDARD_VELOCITY;
					emergencyNotificationSent = false;
				}
			}

			// set the current position
			setPosition(markerCount, ((double) motorLeft.getTachoCount())
					/ 360D * Math.PI * 5.6D);

			// different actions based on the color markings which were observed
			switch (currentColor) {

			// if a position marking is detected: refresh position
			case Color.BLUE:

				if (currentPosition.getAdditionalDistance() > 15) {
					setPosition(++markerCount, 0);
					motorLeft.resetTachoCount();

					if (markerCount == 1) {
						
						System.out.println("Enable V2I communication...");
						enableV2ICommunication(server_ip, server_port);
						System.out.println("V2I communication enabled.");
						
						dataForm = Protocol.FORM_DATA;
						command = Protocol.COMMAND_CHECKIN;
						v2icommunication.sendMessage();
						System.out.println("Sends checkin via V2I");

						// request platoon join/create
						dataForm = Protocol.FORM_PLATOON;
						command = Protocol.COMMAND_JOIN;
						v2icommunication.sendMessage();
						System.out.println("Sends join via V2I");

					}
					if (Position.marshallPosition(currentPosition) >= 500) {
						dataForm = Protocol.FORM_DATA;
						command = Protocol.COMMAND_CHECKOUT;
						v2icommunication.sendMessage();
					}
					
					for(ExitRamp exit: map.getExitRamps()){
					if (Position.marshallPosition(currentPosition) >= exit.getStart() && Position.marshallPosition(currentPosition) < exit.getEnd()) {
						if (shallExit) {
							// send a leaving message to platoon
							if (isInPlatoon && v2vCommunicationEnabled) {
								v2vcommunication.sendMessage(getVehicleNumber()
										+ ": I exit the highway now", false);
							}

							// adjust line following mode (to the color of the
							// exit
							// ramp)
							lineFollowingMode = LineFollowingMode.EXIT;
							isLead = true;
							isInPlatoon = false;
							
							// as long as no blue marking is detected: follow the exit ramp
							while (currentColor != Color.RED) {
								if (hasTwoColorSensors) {
									colorMode.fetchSample(colorSample, 0);
									currentColor = (int) colorSample[0];
								}
								followLine(redMode, redSample);

							}
							shallExit = false;
						} else {
							
							//if the robot is currently using the right highway lane
							if(currentLane == Lane.RIGHT){
								
								//change line following mode because of the exit ramp
								lineFollowingMode = LineFollowingMode.DONT_EXIT;
							}

						}
					}
				}
					
					dataForm = Protocol.FORM_DATA;
					command = Protocol.COMMAND_DATA;
					v2icommunication.sendMessage();
				}
				
				

				// if no marking was detected: follow the highway as regular
			case Color.NONE:
			case Color.BLACK:
				boolean isInExitRamp = false;
				for(ExitRamp exit: map.getExitRamps()){
					
				if (Position.marshallPosition(currentPosition) >= exit.getStart() && Position.marshallPosition(currentPosition) < exit.getEnd()) {
					isInExitRamp = true;
					break;
				}
				}
				
				// change the line following mode to regular mode if the robot
				// is using the left highway lane or if he is not currently
				// passing an exit ramp
				if ((!isInExitRamp) || (currentLane == Lane.OVERTAKING)) {
					lineFollowingMode = LineFollowingMode.REGULAR;
				}
				followLine(redMode, redSample);
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
		if (isInPlatoon && v2vCommunicationEnabled) {
			v2vcommunication.sendMessage(
					getVehicleNumber() + ": I change line", false);
		}
		SensorMode redMode = colorSensorRight.getRedMode();
		float[] redSample = new float[redMode.sampleSize()];
		SensorMode colorMode = colorSensorLeft.getColorIDMode();
		float[] colorSample = new float[colorMode.sampleSize()];
		redSample[0] = 100;
		int currentColor;

		// if the robot is currently on the right highway lane
		if (currentLane == Lane.RIGHT) {
			currentLane = Lane.OVERTAKING;
			currentVelocity = velocityAfterLineChange;
			float velocityBefore = currentVelocity;
			// turn around slightly (for 300 ms)
			motorLeft.setPower(20);
			motorRight.setPower(70);
			motorLeft.forward();
			motorRight.forward();
			long startManeuver = new Date().getTime();
			int i = 0;
			while (new Date().getTime() < startManeuver + 150) {

				// observe position markings
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
				currentPosition.setAdditionalDistance((double) motorLeft
						.getTachoCount() / 360 * Math.PI * 5.6);
				if (currentColor == Color.BLUE) {
					if (currentPosition.getAdditionalDistance() > 15) {
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();

						// send message to infrastructure
						dataForm = Protocol.FORM_DATA;
						command = Protocol.COMMAND_DATA;
						v2icommunication.sendMessage();
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
					if (currentPosition.getAdditionalDistance() > 15) {
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();

						// send message to infrastructure
						dataForm = 0;
						command = Protocol.COMMAND_DATA;
						v2icommunication.sendMessage();
					}
				}
			}

			// lane change successful, normal travelling
			currentVelocity = velocityBefore;

			// if the robot is currently on the left highway lane
		} else {
			currentLane = Lane.RIGHT;

			long turnDuration = 0;
			float velocityLineChange = (currentVelocity + velocityAfterLineChange) / 2;
			if (velocityLineChange <= 50) {
				turnDuration = 300;
			}
			if (velocityLineChange > 50) {
				turnDuration = 400;
			}
			// turn around slightly (for 300 ms)
			motorRight.setPower(20);
			motorLeft.setPower(70);
			motorLeft.forward();
			motorRight.forward();
			long startManeuver = new Date().getTime();
			while (new Date().getTime() < startManeuver + turnDuration) {

				// observe position markings
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
				currentPosition.setAdditionalDistance(motorLeft.getTachoCount()
						/ 360 * Math.PI * 5.6);
				if (currentColor == Color.BLUE) {
					if (currentPosition.getAdditionalDistance() > 15) {
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();

						// send message to infrastructure
						dataForm = 0;
						command = Protocol.COMMAND_DATA;
						v2icommunication.sendMessage();

						if (Position.marshallPosition(currentPosition) >= 500) {
							dataForm = Protocol.FORM_DATA;
							command = Protocol.COMMAND_CHECKOUT;
							v2icommunication.sendMessage();
						}
					}
				}
			}

			// driving straightforward until the right highway lane is reached
			motorLeft.setPower((int) velocityLineChange);
			motorRight.setPower((int) velocityLineChange);
			while (redSample[0] > defaultRightColor + 0.1
					|| redSample[0] < defaultRightColor - 0.1) {
				redMode.fetchSample(redSample, 0);
				colorMode.fetchSample(colorSample, 0);
				currentColor = (int) colorSample[0];
				currentPosition.setAdditionalDistance((double) motorLeft
						.getTachoCount() / 360 * Math.PI * 5.6);
				if (currentColor == Color.BLUE) {
					if (currentPosition.getAdditionalDistance() > 15) {
						currentPosition.setMarkerNumber(++markerCount);
						currentPosition.setAdditionalDistance(0);
						motorLeft.resetTachoCount();

						// send message to infrastructure
						dataForm = 0;
						command = Protocol.COMMAND_DATA;
						v2icommunication.sendMessage();

						if (Position.marshallPosition(currentPosition) >= 500) {
							dataForm = Protocol.FORM_DATA;
							command = Protocol.COMMAND_CHECKOUT;
							v2icommunication.sendMessage();
						}
					}
				}
			}
			// lane change successful, normal travelling
			currentVelocity = velocityAfterLineChange;
		}

		// reset attribute and send message to platoon that line changing was
		// finished
		shallChangeLine = false;
		if (isInPlatoon && v2vCommunicationEnabled) {
			v2vcommunication.sendMessage(getVehicleNumber()
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
		float currentKp = 0;
		float currentKi = 0;
		float currentKd = 0;
		
		if(lineFollowingMode == LineFollowingMode.DONT_EXIT){
			currentKp = 1F;
		}
		else{
			currentKp = kp;
			currentKi = ki;
			currentKd = kd;
		}
		

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
		return Integer.toString(number);
	}

	@Override
	public void setVelocity(int velocity) {
		this.currentVelocity = velocity;
		System.out.println("Velocity set to: " + velocity);

	}

	public int getVelocity() {
		return (int) currentVelocity;
	}

	@Override
	public void changeLine(Lane lane, float desiredVelocity) {
		if (lane != currentLane) {
			this.shallChangeLine = true;
		}
		velocityAfterLineChange = desiredVelocity;

	}

	@Override
	public void joinPlatoon(int platoonNumber, boolean isLead) {
		this.isLead = isLead;
		System.out.println("isLead = " + isLead);
		if (v2vCommunicationEnabled) {
			v2vcommunication.joinGroup(Integer.toString(platoonNumber));
			System.out.println("Joined V2V communication of platoon "
					+ platoonNumber);
			v2vcommunication.sendMessage(getVehicleNumber()
					+ ": Hello, I'm new!", false);
		}
		isInPlatoon = true;
		this.platoonNumber = platoonNumber;
	}

	@Override
	public void leavePlatoon() {
		if (isInPlatoon && v2vCommunicationEnabled) {
			v2vcommunication.sendMessage(getVehicleNumber()
					+ ": I leave the platoon", false);
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
		if (isInPlatoon && v2vCommunicationEnabled) {
			v2vcommunication.sendMessage(getVehicleNumber()
					+ ": I have to stop", false);
		}
		motorLeft.setPower(0);
		motorRight.setPower(0);
		shallStop = true;
		System.out.println("Stopped driving");

	}

	@Override
	public void exitNextRamp() {
		if (isInPlatoon && v2vCommunicationEnabled) {
			v2vcommunication.sendMessage(getVehicleNumber()
					+ ": I will exit next ramp", false);
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

	/**
	 * Returns the robot's IP address as String representation
	 * 
	 * @return The robot's IP as a String
	 */
	public String getIPAsString() {
		return robot_ip;
	}

	@Override
	public int getPlatoonNumber() {
		return platoonNumber;
	}

	@Override
	public int getCommand() {
		return command;
	}

	@Override
	public int getCurrentDataForm() {
		return dataForm;
	}

	@Override
	public int getHighwayLane() {
		return currentLane.getValue();
	}

	@Override
	public int getDestination() {
		return destination;
	}

	@Override
	public int getExit() {
		return desiredExit;
	}
	
	@Override
	public void setExit(int desiredExit){
		this.desiredExit = desiredExit;
	}

	@Override
	public boolean getLane() {
		return isLead;
	}

	@Override
	public boolean getLead() {
		// TODO Auto-generated method stub
		return isLead;
	}

	@Override
	public void setLead(boolean isLead) {
		this.isLead = isLead;
		
	}

}
