package settings;

/**settings for robot 3 */
public class Settings_3 extends Settings{
	private String name = "3";
	private boolean isLead = false;
	private float kp = 2.3F;
	private float ki = 0.5F;
	private float kd = 0.4F;
	private boolean shallExit = false;
	private float gapSize = 0.15F;
	private boolean hasTwoColorSensors = false;
	private float velocity = 50F;
	private String server_ip = "192.168.1.5";
	private int server_port = 5000;
	
	@Override
	public boolean isLead() {
		return this.isLead;
	}
	@Override
	public float getKp() {
		return this.kp;
	}
	@Override
	public float getKi() {
		return this.ki;
	}
	@Override
	public float getKd() {
		return this.kd;
	}
	@Override
	public boolean shallExit() {
		return this.shallExit;
	}
	@Override
	public float getGapSize() {
		return this.gapSize;
	}
	@Override
	public boolean hasTwoColorSensors() {
		return this.hasTwoColorSensors;
	}
	@Override
	public float getVelocity() {
		return this.velocity;
	}
	@Override
	public String getServerIP() {
		return this.server_ip;
	}
	@Override
	public int getServerPort() {
		return this.server_port;
	}
	@Override
	public String getName() {
		return this.name;
	}
	
}
