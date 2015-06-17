package robot;

/**
 * Application startup. Passes needed parameters to the robot.
 * @author Martin
 *
 */
public class Simulation {

	public static void main(String[] args) {
		String[] arguments = {"-Djava.net.preferIPv4Stack=true", "-Djgroups.bind_addr=192.168.1.5"};
		Robot.main(arguments);
	}

}
