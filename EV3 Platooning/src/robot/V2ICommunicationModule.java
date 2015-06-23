package robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.util.Date;

import tools.Commands;
import tools.Lane;

/**
 * Module representing the communication between the vehicle and the roadside
 * infrastructure
 * 
 * @author Martin
 *
 */
public class V2ICommunicationModule extends Thread {

	/** The corresponding vehicle for this module */
	private PlatooningVehicle vehicle;

	/** The socket which is used to communicate */
	private Socket socket;

	/** Stream writers and readers which are used to send messages */
	private PrintWriter out;
	private BufferedReader in;

	/** The used port */
	private int servicePort;

	/** The time interval between two localization messages */
	private long timeInterval = 10;
	// TODO: currently localization messages are sent every 10 ms. This can be
	// changed.

	/** The time of the last position update in ms */
	private long lastPositionUpdate;

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

			// receive the port for service delivery
			servicePort = Integer.parseInt(in.readLine());

			// connect to the service
			socket = new Socket(ip, servicePort);
			out = new PrintWriter(socket.getOutputStream(), true);
			in = new BufferedReader(new InputStreamReader(
					socket.getInputStream()));
		} catch (NumberFormatException | IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Central method which listens for new commands and sends position updates
	 */
	@Override
	public void run() {
		String command;
		lastPositionUpdate = new Date().getTime();
		while (!this.isInterrupted()) {
			try {
				// TODO: enter the handling of messages here

				// receive commands from server and execute them
				command = in.readLine();
				if (command.equals(Commands.CHANGE_LINE_OVERTAKE)) {
					vehicle.changeLine(Lane.OVERTAKING);
				} else if (command.equals(Commands.CHANGE_LINE_RIGHT)) {
					vehicle.changeLine(Lane.RIGHT);
				} else if (command.equals(Commands.EXIT_RAMP)) {
					vehicle.exitNextRamp();
				} else if (command.equals(Commands.STOP)) {
					vehicle.stopDriving();
				} else if (command.equals(Commands.START)) {
					vehicle.startDriving();
				} else if (command.equals(Commands.JOIN_PLATOON)) {
					vehicle.joinPlatoon("Mein erstes Platoon");
					// TODO: change this
				} else if (command.equals(Commands.LEAVE_PLATOON)) {

					// TODO: enter leaving scenario
					vehicle.leavePlatoon();
				}
				// send position update
				if (lastPositionUpdate + timeInterval < new Date().getTime()) {
					out.println(((Robot) vehicle).getPosition().toString());
					lastPositionUpdate = new Date().getTime();
				} else {
					out.println();
				}
			} catch (IOException e) {
				e.printStackTrace();
				System.exit(1);
			}
		}

	}
}
