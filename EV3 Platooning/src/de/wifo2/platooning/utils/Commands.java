package de.wifo2.platooning.utils;

/**
 * Helper class which defines all commands which are transmitted between the roadside infrastructure and the platooning vehicle.
 * Only used by early versions of the Remote steering. 
 * This class is not used while realizing platooning with a Platoon Control System.
 * Use the protocol for correct and current commands.
 * @deprecated Use the protocol instead!
 * @author Martin
 *
 */
public class Commands {
	
	public static final String LEAVE_PLATOON = "leave platoon";
	public static final String JOIN_PLATOON = "join platoon";
	public static final String EXIT_RAMP = "exit ramp";
	public static final String STOP = "stop";
	public static final String START = "start";
	public static final String CHANGE_LINE_RIGHT = "change line right";
	public static final String CHANGE_LINE_OVERTAKE = "change line overtake";
	public static final String[] allCommands = {CHANGE_LINE_RIGHT, CHANGE_LINE_OVERTAKE, LEAVE_PLATOON, JOIN_PLATOON, EXIT_RAMP, STOP, START};



}
