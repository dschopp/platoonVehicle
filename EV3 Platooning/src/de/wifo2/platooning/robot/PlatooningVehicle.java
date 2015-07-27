package de.wifo2.platooning.robot;

import de.wifo2.platooning.utils.Lane;
import de.wifo2.platooning.utils.Position;

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
	public void changeLine(Lane lane, float desiredVelocity);

	/**
	 * The vehicle joins the platoon in front and establishes V2V communication
	 * to this platoon
	 * 
	 * @param platoonID
	 *            The Id of the desired platoon. Necessary for communication
	 *            purposes
	 */
	
	public void joinPlatoon(int platoonID, boolean isLead);

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

	/**
	 * Returns the current data form which shall be used by the next message
	 * sent to the infrastructure
	 * 
	 * @return The current data form.
	 */
	public int getCurrentDataForm();

	/**
	 * Returns the current platoon number
	 * 
	 * @return The current platoon number
	 */
	public int getPlatoonNumber();

	/**
	 * Returns the vehicle number
	 * 
	 * @return The vehicle number
	 */
	public int getVehicleNumber();

	/**
	 * Returns the current command which shall be used by the next message sent
	 * to the infrastructure
	 * 
	 * @return The current command
	 */
	public int getCommand();

	/**
	 * Returns the current velocity
	 * 
	 * @return The current velocity
	 */
	public int getVelocity();

	/**
	 * Returns the current position
	 * 
	 * @return The current position
	 */
	public Position getPosition();

	/**
	 * Returns the current highway lane
	 * 
	 * @return The current highway lane
	 */
	public int getHighwayLane();
}
