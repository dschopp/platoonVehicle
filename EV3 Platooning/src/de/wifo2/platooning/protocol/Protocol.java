package de.wifo2.platooning.protocol;

import java.util.StringTokenizer;

import de.wifo2.platooning.robot.PlatooningVehicle;
import de.wifo2.platooning.utils.DecodedData;
import de.wifo2.platooning.utils.Position;

/**
 * Protocol class which defines the commands which are exchanged between the
 * platooning vehicle and the infrastructure. This class is also used to decode
 * data from received messages and to convert data into messages.
 * 
 * @author Martin
 * @author danielschopp
 */
public class Protocol {

	// If this command is in use, the message contains data only (no command)
	public static final int COMMAND_DATA = 0;

	// Create a platoon
	public static final int COMMAND_CREATE = 1;

	// Join an existing platoon
	public static final int COMMAND_JOIN = 2;

	// Leave a platoon by exiting the highway
	public static final int COMMAND_LEAVE_EXIT = 3;

	// Leave a platoon by overtaking the platoon
	public static final int COMMAND_LEAVE_OVER = 4;

	// Start overtaking (change lane)
	public static final int COMMAND_OVERTAKING_START = 5;

	// End overtaking (change lane)
	public static final int COMMAND_OVERTAKING_END = 6;

	// Checkin command (indicates that a new highway segment is reached)
	public static final int COMMAND_CHECKIN = 7;

	// Checkout command (indicates that a certain highway segment is passed now)
	public static final int COMMAND_CHECKOUT = 8;

	// Notification that a new member has joined a platoon
	public static final int COMMAND_NEW_MEMBER = 9;

	// Two different Forms

	// 1. Form: Data
	public static final int FORM_DATA = 0;

	// 2. Form: Platoon specific message
	public static final int FORM_PLATOON = 1;

	/**
	 * Creates a message according to the protocol by using the data of a
	 * platooning vehicle
	 * 
	 * @param vehicle
	 *            The platooning vehicle which will send this message
	 * @return The String representation of the final message (fits to the
	 *         protocol)
	 */
	public static String createString(PlatooningVehicle vehicle) {

		StringBuilder builder = new StringBuilder();

		// Message number. default: 1
		builder.append("1" + "_");

		// Data form
		builder.append(vehicle.getCurrentDataForm() + "_");

		// Platoon number
		builder.append(vehicle.getPlatoonNumber() + "_");

		// Vehicle number
		builder.append(vehicle.getVehicleNumber() + "_");

		// Command
		builder.append(vehicle.getCommand() + "_");

		// Velocity
		builder.append(vehicle.getVelocity() + "_");

		// Current Position
		builder.append(Position.marshallPosition(vehicle.getPosition()) + "_");

		// Current highway lane
		builder.append(vehicle.getHighwayLane() + "_");
		
		builder.append(vehicle.getDestination() + "_");
		
		builder.append(vehicle.getExit());

		return builder.toString();
	}

	/**
	 * Basic method to decode a received message according to the protocol
	 * 
	 * @param receivedMessage
	 *            The received Message
	 * @return The data and commands of the message as an instance of the class
	 *         DecodedData
	 */
	public static DecodedData decodeString(String receivedMessage) {
		StringTokenizer splitString = new StringTokenizer(receivedMessage, "_");

		// init
		int packageID = 0;
		int dataForm = 0;
		int platoonNumber = 0;
		int vehicleNumber = 0;
		int command = 0;
		float velocity = 0;
		int position = 0;
		int n = 0;
		int lane = 0;
		int destination = 0;
		int exit = 0;

		// split the received message and decode parts of it
		while (splitString.hasMoreElements()) {
			String str = splitString.nextToken();
			switch (n) {
			case 0:
				packageID = 1;
				break;
			case 1:
				dataForm = Integer.parseInt(str);
				break;
			case 2:
				platoonNumber = Integer.parseInt(str);
				break;
			case 3:
				vehicleNumber = Integer.parseInt(str);
				break;
			case 4:
				command = Integer.parseInt(str);
				break;
			case 5:
				velocity = Float.parseFloat(str);
				break;
			case 6:
				position = Integer.parseInt(str);
				break;
			case 7:
				lane = Integer.parseInt(str);
				break;
			case 8:
				destination = Integer.parseInt(str);
				break;
			case 9:
				exit = Integer.parseInt(str);
				break;
			default:
				break;
			}
			n++;
		}
		return new DecodedData(packageID, dataForm, platoonNumber,
				vehicleNumber, command, velocity, position, lane, destination, exit);
	}

}
