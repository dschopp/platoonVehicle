package robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Date;

import settings.Commands;
import settings.Lane;

/** the class managing the communication between the robot and the roadside infrastructure 
 * 
 * @author Martin
 *
 */
public class V2ICommunicationModule extends Thread {
	
	/** the corresponding robot for this module */
	private Robot robot;
	
	/** the socket which is used to communicate */
	private Socket socket;
	
	/** stream writers and readers which are used to send messages */
	private PrintWriter out;
	private BufferedReader in;
	
	/** the used port */
	private int servicePort;
	
	/** the time interval between two localization messages */
	private long timeInterval = 10;
	//TODO: currently localization messages are sent every 10 ms. This can be changed.
	
	/** the time of the last position update in ms */
	private long lastPositionUpdate;
	
	public V2ICommunicationModule(Robot robot, String ip, int port){
		this.robot = robot;
		try {
			
			//initialize writer,socket, and reader
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
			((Robot) this.robot).drawString("dont know host");
			System.exit(1);
		} catch (IOException e) {
			((Robot) this.robot).drawString("io exception");
			System.exit(1);
		}
	}
	
	@Override
	public void run(){
		String command;
		lastPositionUpdate = new Date().getTime();
		while(!this.isInterrupted()){
			try {
				//TODO: enter the handling of messages here
				
				//receive commands from server and execute them
				command = in.readLine();
				if(command.equals(Commands.CHANGE_LINE_OVERTAKE)){
					robot.changeLine(Lane.OVERTAKING);
				}
				else if(command.equals(Commands.CHANGE_LINE_RIGHT)){
					robot.changeLine(Lane.RIGHT);
				}
				else if(command.equals(Commands.EXIT_RAMP)){
					robot.exitNextRamp();
				}
				else if (command.equals(Commands.STOP)){
					robot.stopDriving();
				}
				else if (command.equals(Commands.START)){
					robot.startDriving();
				}
				else if (command.equals(Commands.JOIN_PLATOON)){
				//	robot.joinPlatoon(null);
				//TODO: change this
				}
				else if (command.equals(Commands.LEAVE_PLATOON)){
					
					//TODO: enter leaving scenario
					robot.leavePlatoon();
					robot.closeV2VCommunication();
				}
				//send position update 
				if(lastPositionUpdate + timeInterval < new Date().getTime()){
					out.println(((Robot) robot).getPosition().toString());
					lastPositionUpdate = new Date().getTime();
				}
				else{
					out.println();
				}
				//Thread.sleep(100);

		//	} catch (IOException | InterruptedException e) {
			} catch(IOException e){
				((Robot) robot).drawString("io exception");
				System.exit(1);
			}
		}

	}
}
