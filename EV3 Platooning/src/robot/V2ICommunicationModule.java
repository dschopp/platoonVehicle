package robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.ArrayList;

import tools.DecodedData;
import tools.Lane;
import Protocol.Protocol;

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

	/** Stream writers and readers which are used to send messages */
	private PrintWriter out;
	private BufferedReader in;

	private boolean hasReceivedVelocity = false;
	private V2ISender sender;
	private V2IReceiver receiver;
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

			// send the robots name to the server
			out.println(vehicle);

			// TODO: initial message via protocol

		} catch (NumberFormatException | IOException e) {
			e.printStackTrace();
		}
	}

	private synchronized void putMessageInQueue(String message) {
		messageQueue.add(message);
		System.out.println("Message queue status: " + messageQueue);
	}

	private synchronized String popMessageFromQueue() {
		if (messageQueue.size() > 0) {
			String message = messageQueue.remove(0);
			messageQueue.trimToSize();
			System.out.println("Message queue status: " + messageQueue);
			return message;
		} else {
			return null;
		}
	}

	public void sendMessage() {
		putMessageInQueue(Protocol.createString(vehicle));
	}

	public boolean getHasReceivedVelocity() {
		return hasReceivedVelocity;
	}

	public void startCommunication() {
		sender = new V2ISender();
		receiver = new V2IReceiver();
		sender.start();
		receiver.start();
	}

	public void closeCommunication() {
		sender.interrupt();
		receiver.interrupt();
		System.out.println("V2I communication terminated");
	}

	// /**
	// * Central method which listens for new commands and sends position
	// updates
	// */
	// @Override
	// public void run() {
	// String command;
	// lastPositionUpdate = new Date().getTime();
	// System.out.println("vor dem while");
	// while (!this.isInterrupted()) {
	// try {
	//
	// System.out.println("Listening to command");
	// // receive commands from server and execute them
	// command = in.readLine();
	// System.out.println("Received: " + command);
	// if (command.equals(Commands.CHANGE_LINE_OVERTAKE)) {
	// vehicle.changeLine(Lane.OVERTAKING);
	// } else if (command.equals(Commands.CHANGE_LINE_RIGHT)) {
	// vehicle.changeLine(Lane.RIGHT);
	// } else if (command.equals(Commands.EXIT_RAMP)) {
	// vehicle.exitNextRamp();
	// } else if (command.equals(Commands.STOP)) {
	// vehicle.stopDriving();
	// } else if (command.equals(Commands.START)) {
	// vehicle.startDriving();
	// } else if (command.equals(Commands.JOIN_PLATOON)) {
	// vehicle.joinPlatoon("Mein erstes Platoon");
	//
	// } else if (command.equals(Commands.LEAVE_PLATOON)) {
	//
	//
	// vehicle.leavePlatoon();
	// }
	// // send position update
	// if (lastPositionUpdate + timeInterval < new Date().getTime()) {
	// out.println(((Robot) vehicle).getPosition().toString());
	// lastPositionUpdate = new Date().getTime();
	// } else {
	// out.println();
	// }
	// } catch (IOException e) {
	// e.printStackTrace();
	// System.exit(1);
	// }
	// }
	//
	// }

	private class V2IReceiver extends Thread {
		@Override
		public void run() {
			System.out.println("V2I communication ready to receive");
			String message;
			while (!this.isInterrupted()) {
				try {
					message = in.readLine();
					System.out.println("Received: " + message);
					DecodedData receivedData = Protocol.decodeString(message);
					vehicle.setVelocity((int) receivedData.getVelocity());
					hasReceivedVelocity = true;

					switch (receivedData.getCommand()) {
					case Protocol.COMMAND_CREATE:
					case Protocol.COMMAND_JOIN:
						if (receivedData.getPlatoonNumber() != 0) {
							vehicle.joinPlatoon(Integer.toString(receivedData
									.getPlatoonNumber()));
						}
						break;

					case Protocol.COMMAND_LEAVE_EXIT:
						vehicle.exitNextRamp();
						System.out.println("Exit branch");
						vehicle.leavePlatoon();
						break;
					case Protocol.COMMAND_LEAVE_OVER:
						vehicle.leavePlatoon();
						vehicle.changeLine(Lane.OVERTAKING);
						break;
					case Protocol.COMMAND_OVERTAKING:
						vehicle.changeLine(Lane.OVERTAKING);
						break;
					}

					// TODO: handle messages by applying protocol
				} catch (IOException e) {
					e.printStackTrace();
				}

			}
		}
	}

	private class V2ISender extends Thread {

		@Override
		public void run() {

			System.out.println("V2I communication ready to send");
			while (!this.isInterrupted()) {
				String message = popMessageFromQueue();
				if (message != null) {
					out.println(message);
				}

			}
		}
	}

}

