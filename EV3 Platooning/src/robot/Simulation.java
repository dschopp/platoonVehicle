package robot;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

/**
 * Application startup. Passes needed parameters to the robot.
 * 
 * @author Martin
 *
 */
public class Simulation {

	/** The file name of the config file */
	private static final String CONFIG_FILE_NAME = "config.txt";

	public static void main(String[] args) {

		// load properties from config file
		Properties properties = new Properties();
		BufferedInputStream stream;
		String robot_ip = new String();
		try {
			stream = new BufferedInputStream(new FileInputStream(
					CONFIG_FILE_NAME));
			properties.load(stream);
			stream.close();
			// retrieve robot IP address from config file
			robot_ip = properties.getProperty("robot_ip");
		} catch (IOException e) {
			e.printStackTrace();
		}

		// necessary parameters for JGroups based V2V communication
		String[] arguments = { "-Djava.net.preferIPv4Stack=true",
				"-Djgroups.bind_addr=" + robot_ip };
		Robot.main(arguments);
	}

}
