package robot;

import settings.Settings;
import settings.Settings_1;
import settings.Settings_2;
import settings.Settings_3;

/**
 * main class where robot is managed
 * @author Martin 
 * 
 */
public class Simulation {

	public static void main(String[] args) {
		Settings settings = new Settings_2();
		Robot robot = new Robot(settings);
	}

}
