package robot;

import org.jgroups.JChannel;
import org.jgroups.Message;
import org.jgroups.Message.Flag;
import org.jgroups.ReceiverAdapter;
import org.jgroups.View;

/**
 * Module used to multicast messages to all platoon members.
 * 
 * @author Martin
 *
 */
public class V2VCommunicationModule extends ReceiverAdapter {

	/** The corresponding robot */
	private PlatooningVehicle vehicle;

	/** Indicates whether the module should currently transmit a message */
	private boolean hasToSend = false;

	/** String containing the message which should be sent */
	private String message;

	/** The server thread */
	private V2VServer server;

	/**
	 * The channel used for V2V communication. One Channel per platoon.
	 */
	private JChannel communicationChannel;

	/**
	 * Indicates whether the current message should be transmitted out of band
	 * or not. True, if the message should be delivered without concerning
	 * ordering. Used for time critical messages such as emergency
	 * notifications.
	 */
	private boolean isOOB;

	/**
	 * Standard constructor
	 * 
	 * @param robot
	 *            The robot which corresponds to this module.
	 * @param platoonName
	 *            The name of the platoon which shall be joined.
	 */
	protected V2VCommunicationModule(Robot robot) {

		// Initialization
		this.vehicle = robot;
		server = new V2VServer();

		// The used JGroup configuration file
		String config = "udp_No" + robot.getName() + ".xml";
		try {
			communicationChannel = new JChannel(config);
			communicationChannel.setReceiver(this);

			// Discard own messages
			communicationChannel.setDiscardOwnMessages(true);
			System.out.println("V2V Communication configured");
		} catch (Exception e) {
			System.err.println("V2V communication configuration failed!");
			e.printStackTrace();
		}
	}

	/**
	 * The inner server class which sends messages
	 * 
	 * @author Martin
	 *
	 */
	private class V2VServer extends Thread {
		public void run() {
			while (!isInterrupted()) {

				// If a message needs to be sent
				if (hasToSend) {
					Message newMessage = new Message(null, null, message);

					// If this message shall be transmitted out of band
					if (isOOB) {
						newMessage.setFlag(Flag.OOB);
					}
					try {

						// Send message
						communicationChannel.send(newMessage);
						System.out.println("Sent message: " + message);
						hasToSend = false;
					} catch (Exception e) {
						System.err.println("Failed to send message!");
						e.printStackTrace();
					}
				}
			}
		}

	}

	/**
	 * Provides access to send a message specified by the parameters
	 * 
	 * @param message
	 *            The message which shall be sent
	 * @param isOOB
	 *            True, if the message shall be transmitted out-of-band
	 */
	protected void sendMessage(String message, boolean isOOB) {
		hasToSend = true;
		this.isOOB = isOOB;
		this.message = message;
	}

	/**
	 * If a new vehicle joins the V2V communication group, this method is
	 * triggered
	 */
	@Override
	public void viewAccepted(View new_view) {
		System.out.println("Node joined, current group view: " + new_view);
	}

	/**
	 * This method is triggered if a vehicle receives V2V messages
	 */
	@Override
	public void receive(Message msg) {
		System.out.println("Received message: " + msg.getObject());
		// TODO: handle received V2V messages here
		// in case of emergency--> brake
		if (msg.getObject().toString().contains("EMERGENCY")) {
			vehicle.stopDriving();
		}
	}

	/**
	 * Terminates the current V2V communication and closes the module
	 */
	protected void close() {
		System.out.println("V2V module closed.");
		server.interrupt();
	}

	/**
	 * The vehicle joins a communication group (i.e. a platoon) specified by the
	 * groupName parameter
	 * 
	 * @param groupName
	 *            The name of the communication group
	 */
	protected void joinGroup(String groupName) {
		try {
			communicationChannel.connect(groupName);
			System.out.println("Connection with Group established.");
			server.start();
			System.out.println("V2V communication started");
		} catch (Exception e) {
			System.err.println("Connection with Group failed");
			e.printStackTrace();
		}
	}

	/**
	 * Returns the current platoon size (based on the number of participants in
	 * the communication group)
	 * 
	 * @return The platoon size
	 */
	protected int getPlatoonSize() {
		return communicationChannel.getView().size();
	}

}