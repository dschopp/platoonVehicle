package settings;

/** settings for robot 1 */
public class Settings_1 extends Settings{
	private String name = "1";
	private boolean isLead = true;
	private float kp = 2F;
	private float ki = 0F;
	private float kd = 0F;
	private boolean shallExit = false;
	private float gapSize = 0.15F;
	private boolean hasTwoColorSensors = false;
	private float velocity = 50F;
	private String server_ip = "192.168.1.3";
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
