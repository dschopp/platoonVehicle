package de.wifo2.platooning.communication;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.ArrayList;

import de.wifo2.platooning.protocol.Protocol;
import de.wifo2.platooning.robot.PlatooningVehicle;
import de.wifo2.platooning.utils.DecodedData;
import de.wifo2.platooning.utils.Lane;

/**
 * Module representing the communication between the vehicle and the roadside
 * infrastructure
 * 
 * @author Martin
 *
 */
public class V2ICommunicationModule {

	/** The corresponding vehicle for this module */
	private PlatooningVehicle vehicle;

	/** The socket which is used to communicate */
	private Socket socket;

	/** Stream writer used to send messages */
	private PrintWriter out;

	/** Stream reader used to receive messages */
	private BufferedReader in;

	/**
	 * Indicates whether this module received a desired velocity by the
	 * infrastructure
	 */
	private boolean hasReceivedVelocity = false;

	/** The sending thread */
	private V2ISender sender;

	/** The receiving thread */
	private V2IReceiver receiver;

	/**
	 * Message queue containing all messages which need to be sent. This message
	 * queue is needed to avoid inconsistencies in communication
	 */
	private ArrayList<String> messageQueue = new ArrayList<String>();

	/**
	 * Standard constructor of a V2I communication module
	 * 
	 * @param vehicle
	 *            The vehicle which shall use this new module
	 * @param ip
	 *            The desired IP address of the module
	 * @param port
	 *            The desired port which is used for V2I communication
	 */
	public V2ICommunicationModule(PlatooningVehicle vehicle, String ip, int port) {
		this.vehicle = vehicle;
		try {

			// initialize writer,socket, and reader
			socket = new Socket(ip, port);
			in = new BufferedReader(new InputStreamReader(
					socket.getInputStream()));
			out = new PrintWriter(socket.getOutputStream(), true);

		} catch (NumberFormatException | IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Helper method to put a message in the message queue
	 * 
	 * @param message
	 *            The message which will be transferred in the queue
	 */
	private synchronized void putMessageInQueue(String message) {
		messageQueue.add(message);
	}

	/**
	 * Retrieves the message from the queue which shall be sent next.
	 * 
	 * @return The message which shall be sent or null, if no message available
	 */
	private synchronized String popMessageFromQueue() {
		if (messageQueue.size() > 0) {
			String message = messageQueue.remove(0);
			messageQueue.trimToSize();
			return message;
		} else {
			return null;
		}
	}

	/**
	 * Triggers the sending process
	 */
	public void sendMessage() {
		putMessageInQueue(Protocol.createString(vehicle));
	}

	/**
	 * Getter for hasReceivedVelocity
	 * 
	 * @return hasReceivedVelocity
	 */
	public boolean getHasReceivedVelocity() {
		return hasReceivedVelocity;
	}

	/**
	 * Initial method to start V2I communication
	 */
	public void startCommunication() {
		sender = new V2ISender();
		receiver = new V2IReceiver();
		sender.start();
		receiver.start();
	}

	/**
	 * Terminates this V2I communication module
	 */
	public void closeCommunication() {
		sender.interrupt();
		receiver.interrupt();
		System.out.println("V2I communication terminated");
	}

	/**
	 * Inner class which is listening for new messages permanently
	 * 
	 * @author Martin
	 *
	 */
	private class V2IReceiver extends Thread {
		@Override
		public void run() {
			System.out.println("V2I communication ready to receive");
			String message;

			// as long as the communication module is running
			while (!this.isInterrupted()) {
				try {

					// receive message
					message = in.readLine();
					System.out.println("Received: " + message);

					// decode message
					DecodedData receivedData = Protocol.decodeString(message);

					// vary behavior by command
					switch (receivedData.getCommand()) {

					// create a new platoon
					case Protocol.COMMAND_CREATE:
						vehicle.setVelocity((int) receivedData.getVelocity());
						hasReceivedVelocity = true;
						vehicle.setLead(receivedData.getLead());
						if (receivedData.getPlatoonNumber() != 0) {
							vehicle.joinPlatoon(
									receivedData.getPlatoonNumber(), vehicle.getLead());
						}
						break;

					// join an existing platoon
					case Protocol.COMMAND_JOIN:
						vehicle.setVelocity((int) receivedData.getVelocity());
						hasReceivedVelocity = true;
						vehicle.setLead(receivedData.getLead());
						if (receivedData.getPlatoonNumber() != 0) {
							vehicle.joinPlatoon(
									receivedData.getPlatoonNumber(), vehicle.getLead());
						}
						break;

					// leave a platoon by exiting the highway
					case Protocol.COMMAND_LEAVE_EXIT:
						vehicle.setVelocity((int) receivedData.getVelocity());
						vehicle.setExit((int) receivedData.getExit());
						hasReceivedVelocity = true;
						vehicle.exitNextRamp();
						vehicle.leavePlatoon();
						break;

					// leave a platoon by overtaking
					case Protocol.COMMAND_LEAVE_OVER:
						vehicle.leavePlatoon();
						vehicle.changeLine(Lane.OVERTAKING,
								(int) receivedData.getVelocity());
						break;

					// start overtaking
					case Protocol.COMMAND_OVERTAKING_START:
						vehicle.changeLine(Lane.OVERTAKING,
								(int) receivedData.getVelocity());
						break;

					// end overtaking
					case Protocol.COMMAND_OVERTAKING_END:
						vehicle.changeLine(Lane.RIGHT,
								(int) receivedData.getVelocity());
						break;
					}

				} catch (IOException e) {
					e.printStackTrace();
				}

			}
		}
	}

	/**
	 * Inner class which is ready to send messages permanently. This sender
	 * sends messages to the infrastructure.
	 * 
	 * @author Martin
	 *
	 */
	private class V2ISender extends Thread {
		@Override
		public void run() {
			System.out.println("V2I communication ready to send");

			// as long as this module is running
			while (!this.isInterrupted()) {
				String message = popMessageFromQueue();

				// if a message is stored in the queue --> send it
				if (message != null) {
					System.out.println("Sent via V2I: " + message);
					out.println(message);
				}

			}
		}
	}

}

