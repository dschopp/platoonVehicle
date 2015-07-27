package de.wifo2.platooning.utils;
/**
 * Simple Container class used to represent data received from the Platoon Control System.
 * Refer to {@link de.wifo2.platooning.protocol.Protocol} for further information.
 * @author Martin
 *
 */
public class DecodedData {
	
	//data 
	private int packageID;
	private int dataForm;
	private int platoonNumber;
	private int vehicleNumber;
	private int command; 
	private float velocity;
	private int position;
	private int lane;
	
	/**
	 * Basic constructor.
	 * @param packageID
	 * @param dataForm
	 * @param platoonNumber
	 * @param vehicleNumber
	 * @param command
	 * @param velocity
	 * @param position
	 * @param lane
	 */
	public DecodedData(int packageID, int dataForm, int platoonNumber,
			int vehicleNumber, int command, float velocity, int position, int lane) {
		this.packageID = packageID;
		this.dataForm = dataForm;
		this.platoonNumber = platoonNumber;
		this.vehicleNumber = vehicleNumber;
		this.command = command;
		this.velocity = velocity;
		this.position = position;
		this.lane = lane;
	}

	
	//Getter and setter
	public int getPackageID() {
		return packageID;
	}

	public void setPackageID(int packageID) {
		this.packageID = packageID;
	}

	public int getDataForm() {
		return dataForm;
	}

	public void setDataForm(int dataForm) {
		this.dataForm = dataForm;
	}

	public int getPlatoonNumber() {
		return platoonNumber;
	}

	public void setPlatoonNumber(int platoonNumber) {
		this.platoonNumber = platoonNumber;
	}

	public int getVehicleNumber() {
		return vehicleNumber;
	}

	public void setVehicleNumber(int vehicleNumber) {
		this.vehicleNumber = vehicleNumber;
	}

	public int getCommand() {
		return command;
	}

	public void setCommand(int command) {
		this.command = command;
	}

	public float getVelocity() {
		return velocity;
	}

	public void setVelocity(float velocity) {
		this.velocity = velocity;
	}

	public int getPosition() {
		return position;
	}

	public void setPosition(int position) {
		this.position = position;
	}

	public int getLane() {
		return lane;
	}

	public void setLane(int lane) {
		this.lane = lane;
	}


}
