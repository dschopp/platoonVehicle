package robot;

import settings.Lane;

public interface PlatooningVehicle {
		public void setVelocity(int velocity);
		public void changeLine(Lane lane);
		public void joinPlatoon(String platoonIP);
		public void leavePlatoon();
		public void startDriving();
		public void stopDriving();
		public void exitNextRamp();
		public void setGapSize(float gapSize);
		public boolean sendMessageToPlatoon(String message);
}
