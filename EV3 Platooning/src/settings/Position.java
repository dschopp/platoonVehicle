package settings;

public class Position {
	
	private int markerNumber;
	private double additionalDistance;
	
	public Position(){
		markerNumber = 0;
		additionalDistance = 0;
	}
	public Position(int markerNumber, double additionalDistance) {
		super();
		this.markerNumber = markerNumber;
		this.additionalDistance = additionalDistance;
	}

	public int getMarkerNumber() {
		return markerNumber;
	}
	public void setMarkerNumber(int markerNumber) {
		this.markerNumber = markerNumber;
	}
	public double getAdditionalDistance() {
		return additionalDistance;
	}
	public void setAdditionalDistance(double additionalDistance) {
		this.additionalDistance = additionalDistance;
	}
	
	public String toString(){
		if(markerNumber == 0){
			return "not on highway";
		}
		else{
			return "Marker No. " + markerNumber + " , Distance: " + additionalDistance;
		}
	}

}
