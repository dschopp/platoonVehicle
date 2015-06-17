package remote;

import settings.Commands;
import settings.Position;

/**
 * main class for remote steering. launches a server where all robot connect.
 * creates a UI to control robots
 * 
 * @author Martin
 *
 */
public class Remote {

	/** the remote server for all robots */
	private RemoteServer server = new RemoteServer(this);
	
	
	/** 15 + 0.5 velocity */
	private int overtakeDistance = 40;
	private int reenterDistance = 10;

	/** the UI */
	private RemoteUI ui;

	public RemoteServer getServer() {
		return this.server;
	}

	public Remote() {
		server.start();
		ui = new RemoteUI(this);
		ui.setVisible(true);
	}

	public static void main(String[] args) {
		Remote remote = new Remote();
		while (!remote.server.isInterrupted()) {
			remote.ui.refreshRobotText();
//			if (remote.server.getServices().get("2") != null
//					&& remote.server.getServices().get("3") != null) {
//				String position2 = remote.server.getServices().get("2")
//						.getRobotPosition();
//				String position3 = remote.server.getServices().get("3")
//						.getRobotPosition();
//				if (!position2.equals("not on highway")
//						&& !position3.equals("not on highway")) {
//					int marker2 = Integer.parseInt(position2.split(" ")[2]);
//					double additionalDistance2 = Double.parseDouble(position2
//							.split(" ")[5]);
//
//					int marker3 = Integer.parseInt(position3.split(" ")[2]);
//					double additionalDistance3 = Double.parseDouble(position3
//							.split(" ")[5]);
//					System.out.println("----------------------");
//					System.out.println("Pos 2: " + marker2 + ", "
//							+ additionalDistance2);
//					System.out.println("Pos 3: " + marker3 + ", "
//							+ additionalDistance3);
//					System.out.println("Gap Size: "
//							+ (marker2 * 31 + additionalDistance2 - marker3
//									* 31 - additionalDistance3));
//					System.out.println("----------------------");
//					
//					
//					//change line left
//					if (marker2 * 31 + additionalDistance2 > marker3 * 31
//							+ additionalDistance3) {
//						if (marker2 * 31 + additionalDistance2 < marker3 * 31
//								+ additionalDistance3 + remote.overtakeDistance) {
//							System.out.println("Too close, check position...");
//							try {
//								Thread.sleep(300);
//							} catch (InterruptedException e) {
//								e.printStackTrace();
//							}
//							position2 = remote.server.getServices().get("2")
//									.getRobotPosition();
//							position3 = remote.server.getServices().get("3")
//									.getRobotPosition();
//							marker2 = Integer.parseInt(position2.split(" ")[2]);
//							additionalDistance2 = Double.parseDouble(position2
//									.split(" ")[5]);
//
//							marker3 = Integer.parseInt(position3.split(" ")[2]);
//							additionalDistance3 = Double.parseDouble(position3
//									.split(" ")[5]);
//							if (marker2 * 31 + additionalDistance2 < marker3
//									* 31 + additionalDistance3 + remote.overtakeDistance) {
//								System.out
//										.println("Still too close. Overtake!");
//								remote.server
//										.getServices()
//										.get("3")
//										.setCommand(
//												Commands.CHANGE_LINE_OVERTAKE);
//								remote.server.getServices().get("3")
//										.setHasToSend(true);
//							} else {
//								System.out.println("Not too close anymore");
//							}
//						}
//					}
//					
//					//change line right
//					if(marker2 * 31 + additionalDistance2 < marker3 * 31 + additionalDistance3){
//						if (marker2 * 31 + additionalDistance2 + remote.reenterDistance < marker3
//								* 31 + additionalDistance3) {
//							System.out.println("Overtaking complete?");
//							try {
//								Thread.sleep(300);
//							} catch (InterruptedException e) {
//								e.printStackTrace();
//							}
//							position2 = remote.server.getServices().get("2")
//									.getRobotPosition();
//							position3 = remote.server.getServices().get("3")
//									.getRobotPosition();
//							marker2 = Integer.parseInt(position2.split(" ")[2]);
//							additionalDistance2 = Double.parseDouble(position2
//									.split(" ")[5]);
//
//							marker3 = Integer.parseInt(position3.split(" ")[2]);
//							additionalDistance3 = Double.parseDouble(position3
//									.split(" ")[5]);
//							if (marker2 * 31 + additionalDistance2 + remote.reenterDistance < marker3
//									* 31 + additionalDistance3) {
//								System.out
//										.println("I think it's time to change line");
//								remote.server.getServices().get("3")
//										.setCommand(Commands.CHANGE_LINE_RIGHT);
//								remote.server.getServices().get("3")
//										.setHasToSend(true);
//							}
//						}
//					}
//				}
//				}
			

			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	public String getRobotPosition(String name) {
		return server.getServices().get(name).getRobotPosition();
	}

	/**
	 * if a new robot connects to the system, the UI shall display the new robot
	 */
	public void addRobot(String robotName) {
		ui.addRobot(robotName);
	}

}
