package robot;

import tools.Lane;

/**
 * Interface to access main (driving) features of a vehicle which is capable of
 * platooning
 * 
 * @author Martin
 *
 */
public interface PlatooningVehicle {
	/**
	 * Sets the velocity of the vehicle to the specified value
	 * 
	 * @param velocity
	 *            The desired velocity
	 */
	public void setVelocity(int velocity);

	/**
	 * The vehicle changes the highway lane according to the passed lane
	 * 
	 * @param lane
	 *            The target lane
	 */
	public void changeLine(Lane lane);

	/**
	 * The vehicle joins the platoon in front and establishes V2V communication
	 * to this platoon
	 * 
	 * @param platoonID
	 *            The Id of the desired platoon. Necessary for communication
	 *            purposes
	 */
	public void joinPlatoon(String platoonID);

	/**
	 * The vehicle leaves the current platoon
	 */
	public void leavePlatoon();

	/**
	 * The vehicles starts driving
	 */
	public void startDriving();

	/**
	 * The vehicle immediately stops driving
	 */
	public void stopDriving();

	/**
	 * The vehicle will exit the next available exit ramp
	 */
	public void exitNextRamp();

	/**
	 * Adjusts the gap size of a vehicle if it is part of a platoon
	 * 
	 * @param gapSize
	 *            The desired gap size
	 */
	public void setGapSize(float gapSize);

	/**
	 * If the vehicle is part of a platoon, it multicasts the message to all
	 * platoon members
	 * 
	 * @param message
	 *            The message which will be sent
	 * @param isOOB
	 *            True, if the message should be transmitted out-of-band (i.e.
	 *            immediately without ordering)
	 * @return
	 */
	public boolean sendMessageToPlatoon(String message, boolean isOOB);
}
