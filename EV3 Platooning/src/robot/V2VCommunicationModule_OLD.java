package robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import lejos.hardware.lcd.LCD;

/** class using multicasts to transmit messages to all platoon members */
public class V2VCommunicationModule_OLD {

	/** the corresponding robot */
	private Robot robot;

	/** the socket for communication */
	private MulticastSocket v2vSocket;

	/** indicates whether the module should currently transmit a message */
	private boolean hasToSend;

	/** string containing the message which should be sent */
	private String message;

	/** the port which is used for V2V communication. default: 6000 */
	private final int clientPort = 6000;

	/** the client thread */
	private V2VClient client;

	/** the server thread */
	private V2VServer server;

	/** the address of the multicast group of the platoon */
	private InetAddress platoonAddress;

	/** the robots active IP address(es) (except localhost) */
	private List<InetAddress> robotAddress = new ArrayList<InetAddress>();

	protected V2VCommunicationModule_OLD(Robot robot, String platoonAddress) {
		try {

			// initialization
			this.robot = robot;
			client = new V2VClient();
			server = new V2VServer();
			v2vSocket = new MulticastSocket(clientPort);

			// join the platoons multicast group
			this.platoonAddress = InetAddress.getByName(platoonAddress);
			v2vSocket.joinGroup(this.platoonAddress);

			// get robot ip address
			Enumeration<NetworkInterface> interfaces = NetworkInterface
					.getNetworkInterfaces();
			while (interfaces.hasMoreElements()) {
				NetworkInterface iface = interfaces.nextElement();
				// filter localhost and inactive interfaces
				if (iface.isLoopback() || !iface.isUp())
					continue;
				Enumeration<InetAddress> addresses = iface.getInetAddresses();
				while (addresses.hasMoreElements()) {
					robotAddress.add(addresses.nextElement());
				}
			}
		} catch (IOException e) {
			LCD.clear();
			LCD.drawString(e.getMessage(), 0, 0);
			try {
				Thread.sleep(5000);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}
		}
	}

	/**
	 * the client thread which is used to receive messages (implemented as a
	 * thread
	 */
	private class V2VClient extends Thread {
		public void run() {
			DatagramPacket packet = new DatagramPacket(new byte[1024], 1024);
			// try to receive messages constantly
			while (!isInterrupted()) {
				try {
					v2vSocket.receive(packet);
					byte[] data = packet.getData();
					if (!robotAddress.contains(packet.getAddress())) {
						// TODO handle message
						System.out.println(new String(data));
						if(new String(data).equals("EMERGENCY BRAKE")){
							robot.stopDriving();
							robot.drawString("EMERGENCY");
						}
						// enter code here
//						if(Integer.parseInt(new String(data).trim()) != ++expected){
//							System.out.println("expected: " + expected + " real: " + new String(data));
//							expected = Integer.parseInt(new String(data).trim());
//							if(expected > 1000){
//								System.exit(1);
//							}
//							allReceivedNumbers[++expected] = Integer.parseInt(new String(data).trim());
//						}

					}
					Thread.sleep(0);
				} catch (IOException | InterruptedException e) {
					LCD.clear();
					LCD.drawString(e.getMessage(), 0, 0);
					try {
						Thread.sleep(5000);
					} catch (InterruptedException e1) {
						e1.printStackTrace();
					}
				}

			}
		}
	}

	/**
	 * the server thread which is used to multicast messages (implemented as a
	 * thread
	 */
	private class V2VServer extends Thread {
		public void run() {
			try {
				while (!isInterrupted()) {
					// if a message has to be sent: send it
					if (hasToSend) {
						DatagramPacket packet = new DatagramPacket(
								message.getBytes(), message.getBytes().length,
								platoonAddress, clientPort);
						v2vSocket.send(packet);
						hasToSend = false;
					}
					Thread.sleep(0);
				}
			} catch (IOException | InterruptedException e) {
				LCD.clear();
				LCD.drawString(e.getMessage(), 0, 0);
				try {
					Thread.sleep(5000);
				} catch (InterruptedException e1) {
					e1.printStackTrace();
				}
			}

		}
	}

	/** start V2V communication */
	public void startCommunication() {
		server.start();
		client.start();
		robot.drawString("V2V");
	}

	protected void setHasToSend(boolean hasToSend) {
		this.hasToSend = hasToSend;
	}

	protected void sendMessage(String message) {
		hasToSend = true;
		this.message = message;
	}

	protected void setMessage(String message) {
		this.message = message;
	}

	/** terminate V2V communication (used for platoon leaving) */
	public void close() {
		client.interrupt();
		server.interrupt();
	}
	
	public static void main(String[] args){
		V2VCommunicationModule_OLD v2v = new V2VCommunicationModule_OLD(null, "224.0.0.1");
		v2v.startCommunication();

	}

}
