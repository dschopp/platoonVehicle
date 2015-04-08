package remote;

/**main class for remote steering.
 * launches a server where all robot connect.
 * creates a UI to control robots
 * @author Martin
 *
 */
public class Remote {
	
	/** the remote server for all robots */
	private RemoteServer server = new RemoteServer(this);
	
	/** the UI */
	private RemoteUI ui;
	
	public RemoteServer getServer(){
		return this.server;
	}
	
	public Remote(){
		server.start();
		ui = new RemoteUI(this);
		ui.setVisible(true);
	}
	
	public static void main(String[] args){
		Remote remote = new Remote();
	}

	/**if a new robot connects to the system, the UI shall display the new robot */
	public void addRobot(String robotName) {
		ui.addRobot(robotName);		
	}
		
}
