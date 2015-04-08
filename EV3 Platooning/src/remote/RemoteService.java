package remote;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;

/**
 * a single service for one robot. the actual communication is done here.
 * 
 * @author Martin
 *
 */
public class RemoteService extends Thread {

	/** the sockets */
	private ServerSocket serverSocket;
	private Socket clientSocket;

	/** the command that shall be sent to the robot */
	private String command;

	/** indicates whether a command shall be sent at the moment or not */
	private boolean hasToSend;

	public RemoteService(int port) {
		try {
			serverSocket = new ServerSocket(port);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void setCommand(String command) {
		this.command = command;
	}

	public void setHasToSend(boolean hasToSend) {
		this.hasToSend = hasToSend;
	}

	public void run() {
		try {

			// initiate connection
			clientSocket = serverSocket.accept();
			System.out.println("Service started!");
			PrintWriter out = new PrintWriter(clientSocket.getOutputStream(),
					true);
			BufferedReader in = new BufferedReader(new InputStreamReader(
					clientSocket.getInputStream()));

			while (!isInterrupted()) {

				// send command if necessary
				if (hasToSend) {
					out.println(command);
					hasToSend = false;
				}
				Thread.sleep(100);
			}
		} catch (IOException | InterruptedException e) {
			e.printStackTrace();
		}
	}

}
