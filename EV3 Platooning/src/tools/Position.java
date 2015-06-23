package tools;

/**
 * Container class which saves the position data of a robot
 * 
 * @author Martin
 *
 */
public class Position {

	/** The number of position markings which were passed */
	private int markerNumber;

	/**
	 * The additional distance (in cm) based on wheel rotation after the last
	 * marking
	 */
	private double additionalDistance;

	/**
	 * Constructor to create an empty position
	 */
	public Position() {
		markerNumber = 0;
		additionalDistance = 0;
	}

	/**
	 * Standard constructor to create a new position
	 * 
	 * @param markerNumber
	 *            The number of position markings which were passed
	 * @param additionalDistance
	 *            The additional distance (in cm) based on wheel rotation after
	 *            the last marking
	 */
	public Position(int markerNumber, double additionalDistance) {
		this.markerNumber = markerNumber;
		this.additionalDistance = additionalDistance;
	}

	/**
	 * Sets the attribute markerNumber to the passed value
	 * 
	 * @param markerNumber
	 *            The desired value for the attribute markerNumber
	 */
	public void setMarkerNumber(int markerNumber) {
		this.markerNumber = markerNumber;
	}

	/**
	 * Retrieves the additional distance (in cm) based on wheel rotation after
	 * the last marking
	 * 
	 * @return The additional distance (in cm) based on wheel rotation after the
	 *         last marking
	 */
	public double getAdditionalDistance() {
		return additionalDistance;
	}

	/**
	 * Sets the additionalDistance attribute to the passed value
	 * 
	 * @param additionalDistance
	 *            The desired value for the attribute additionalDistance
	 */
	public void setAdditionalDistance(double additionalDistance) {
		this.additionalDistance = additionalDistance;
	}

	@Override
	public String toString() {
		if (markerNumber == 0) {
			return "not on highway";
		} else {
			return "Marker No. " + markerNumber + " , Distance: "
					+ additionalDistance;
		}
	}

}
