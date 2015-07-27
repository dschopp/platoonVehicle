package de.wifo2.platooning.remote;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashMap;

/**
 * server which connects with new robots and launches services for each robot.
 * @deprecated Use the protocol instead
 * @author Martin
 *
 */
public class RemoteServer extends Thread {

	/** the client socket */
	private Socket clientSocket;

	/** the server socket */
	private ServerSocket serverSocket;
	private PrintWriter out;
	private BufferedReader in;

	/** hash map containing all robot names and the corresponding services */
	private HashMap<String, RemoteService> services = new HashMap<String, RemoteService>();

	/**
	 * indicates a free port where the next service will be started if a robot
	 * connects
	 */
	private int freePort = 5000;

	private Remote remote;

	public RemoteServer(Remote remote) {
		try {
			this.remote = remote;
			serverSocket = new ServerSocket(5000);
		} catch (IOException e) {
			e.printStackTrace();
			System.exit(1);
		}
	}

	public void run() {
		String robotName;
		try {
			while (!isInterrupted()) {

				// initiate connection
				clientSocket = serverSocket.accept();

				out = new PrintWriter(clientSocket.getOutputStream(), true);
				in = new BufferedReader(new InputStreamReader(
						clientSocket.getInputStream()));

				// get the robots name
				robotName = in.readLine();

				// send new port to the client
				out.println(Integer.toString(++freePort));

				// start new service for the client
				RemoteService service = new RemoteService(freePort);
				service.start();
				services.put(robotName, service);

				// refresh UI
				remote.addRobot(robotName);

				// close connection
				clientSocket.close();
				out.close();
				in.close();
				Thread.sleep(100);

			}
			interruptServices();
		} catch (IOException | InterruptedException e) {
			e.printStackTrace();
			System.exit(1);
		}

	}

	public HashMap<String, RemoteService> getServices() {
		return this.services;
	}

	/** method used to terminate all services if remote server is shut down */
	public void interruptServices() {
		for (int i = 0; i < services.size(); i++) {
			services.get(i).interrupt();
		}
	}
}
