package robot;

import org.jgroups.JChannel;
import org.jgroups.Message;
import org.jgroups.Message.Flag;
import org.jgroups.ReceiverAdapter;
import org.jgroups.View;


/** Module used to multicast messages to all platoon members. 
 * 
 * @author Martin
 *
 */
public class V2VCommunicationModule extends ReceiverAdapter {

	/** The corresponding robot */
	@SuppressWarnings("unused")
	private Robot robot;

	/** Indicates whether the module should currently transmit a message */
	private boolean hasToSend = false;

	/** String containing the message which should be sent */
	private String message;

	/** The server thread */
	private V2VServer server;
	
	/** The channel used for V2V communication.
	 * One Channel per platoon.
	 */
	private JChannel communicationChannel;
	
	/**Indicates whether the current message should be transmitted out of band or not.
	 * True, if the message should be delivered without concerning ordering.
	 * Used for time critical messages such as emergency notifications.
	 */
	private boolean isOOB;

	/**
	 * Standard constructor
	 * @param robot The robot which corresponds to this module.
	 * @param platoonName The name of the platoon which shall be joined.
	 */
	protected V2VCommunicationModule(Robot robot){
		
			// Initialization
			this.robot = robot;
			server = new V2VServer();
			
			// The used JGroup configuration file
			String config = "udp_No" + robot.getName() + ".xml";
			try {
				communicationChannel = new JChannel(config);
				communicationChannel.setReceiver(this);

				//Discard own messages
				communicationChannel.setDiscardOwnMessages(true);
				System.out.println("V2V Communication configured");
			} catch (Exception e) {
				System.err.println("V2V communication configuration failed!");
				e.printStackTrace();
			}
	}


	/**
	 * The inner server class which sends messages
	 * @author Martin
	 *
	 */
	private class V2VServer extends Thread {
		public void run() {
			while (!isInterrupted()) {
				
				//If a message needs to be sent
				if (hasToSend) {
					Message newMessage = new Message(null, null, message);
					
					//If this message shall be transmitted out of band
					if(isOOB){
						newMessage.setFlag(Flag.OOB);
					}
					try {
						
						//Send message
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
	
	protected void sendMessage(String message, boolean isOOB) {
		hasToSend = true;
		this.isOOB = isOOB;
		this.message = message;
	}
	
	public void viewAccepted(View new_view) {
	    System.out.println("Node joined, current group view: " + new_view);
	}

	public void receive(Message msg) {
		System.out.println("Received message: " + msg.getObject());
		
		//in case of emergency--> brake
		if(msg.getObject().toString().contains("EMERGENCY")){
			robot.stopDriving();
		}
	}

	/** terminate V2V communication (used for platoon leaving) */
	public void close() {
		System.out.println("V2V module closed.");
		server.interrupt();
	}
	
	public void joinGroup(String groupName){
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

	public int getPlatoonSize() {
		return communicationChannel.getView().size();
	}

}