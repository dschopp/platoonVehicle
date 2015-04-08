package robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;

import settings.Commands;

/** thread handling the communication of one robot 
 * 
 * @author Martin
 *
 */
public class RobotCommunication extends Thread {
	private Robot robot;
	private Socket socket;
	private PrintWriter out;
	private BufferedReader in;
	private int servicePort;
	
	public RobotCommunication(Robot robot, String ip, int port){
		this.robot = robot;
		try {
			socket = new Socket(ip, port);
			in = new BufferedReader(new InputStreamReader(
					socket.getInputStream()));
			out = new PrintWriter(socket.getOutputStream(), true);
			
			//send the robots name to the server
			out.println(robot);
			
			//receive the port for service delivery
			servicePort = Integer.parseInt(in.readLine());
			
			//connect to the service
			socket = new Socket(ip, servicePort);
			out = new PrintWriter(socket.getOutputStream(), true);
			in = new BufferedReader(new InputStreamReader(
					socket.getInputStream()));
		} catch (UnknownHostException e) {
			this.robot.drawString("dont know host");
			System.exit(1);
		} catch (IOException e) {
			this.robot.drawString("io exception");
			System.exit(1);
		}
	}
	
	public void run(){
		String command;
		while(!this.isInterrupted()){
			try {
				
				//receive commands from server and execute them
				command = in.readLine();
				if(command.equals(Commands.CHANGE_LINE)){
					this.robot.setShallChangeLine(true);
				}
				else if(command.equals(Commands.EXIT_RAMP)){
					this.robot.setShallExit(true);
				}
				else if (command.equals(Commands.STOP)){
					this.interrupt();
					this.robot.setShallStop(true);
				}
				else if (command.equals(Commands.JOIN_PLATOON)){
					//TODO: check this
					this.robot.setIsLead(false);
				}
				else if (command.equals(Commands.LEAVE_PLATOON)){
					//TODO: check this
					this.robot.setShallLeavePlatoon(true);
				}
				Thread.sleep(100);

			} catch (IOException | InterruptedException e) {
				robot.drawString("io exception");
				System.exit(1);
			}
		}

	}
}
