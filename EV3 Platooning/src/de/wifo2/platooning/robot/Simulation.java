package de.wifo2.platooning.robot;

/**
 * Application startup. Passes needed parameters to the robot.
 * 
 * @author Martin
 *
 */
public class Simulation {

	public static void main(String[] args) {

		// necessary parameters for JGroups based V2V communication
		String[] arguments = { "-Djava.net.preferIPv4Stack=true" };
		Robot.main(arguments);
	}

}
