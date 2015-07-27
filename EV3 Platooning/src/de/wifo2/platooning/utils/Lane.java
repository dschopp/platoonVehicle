package de.wifo2.platooning.utils;

/**
 * Basic enum to specify all possible highway lanes
 * @author Martin
 *
 */
public enum Lane {
	
	OVERTAKING(2), RIGHT(1);
	
	/** The lane's number. This number is used in V2I messages */
	private int value;

	/**
	 * Constructor
	 * @param value The lane's number
	 */
	private Lane(int value){
		this.value = value;
	}
	
	/**
	 * Retrieves the number of a lane
	 * @return The number
	 */
	public int getValue(){
		return value;
	}
}
