package settings;

/**abstract class too specify needed settings for every robot */
public abstract class Settings {
	public abstract String getName();
	public abstract boolean isLead();
	public abstract float getKp();
	public abstract float getKi();
	public abstract float getKd();
	public abstract boolean shallExit();
	public abstract float getGapSize();
	public abstract boolean hasTwoColorSensors();
	public abstract float getVelocity();
	public abstract String getServerIP();
	public abstract int getServerPort();
}
