package Protocol;

/**Protocol class which defines the String
 * 
 * @author danielschopp
 */

import java.util.StringTokenizer;
import LogicData.MonitorData;

public class Protocol {
	//Commands
	public static final int COMMAND_CREATE = 1;
	public static final int COMMAND_JOIN = 2;
	public static final int COMMAND_LEAVE = 3;
	public static final int COMMAND_OVERTAKING = 4;
	//Form
	public static final int FORM_DATA = 1;
	public static final int FORM_PLATOON = 2;
	
	
	public String createProtocol(int form, int command, ProtocolParameter parameter){
		
		return null;
	}
	
	public String createString(MonitorData m){
		
		StringBuilder builder = new StringBuilder();
		builder.append(m.getPackageID() +"_");
		builder.append(m.getDataForm() +"_");
		builder.append(m.getPlatoonNumber() +"_");
		builder.append(m.getVehicleNumber() +"_");
		builder.append(m.getCommand() +"_");
		builder.append(m.getVelocity());
		
		return builder.toString();
	}
	
	public MonitorData decodeString(String s){
		
		String[] sString = s.split("_");
		//Decoder depends on command
		int command = Integer.parseInt(sString[4]);

		switch (command) {
		case COMMAND_CREATE : 
			return decodeCreateCommand(s);
		case COMMAND_JOIN : 
			return decodeCreateCommand(s);
		default:
			break;
		}
		return null;
	}

	private MonitorData decodeCreateCommand(String s) {
		StringTokenizer splitString = new StringTokenizer(s,"_");
		int packageID = 0;
		int dataForm = 0;
		int platoonNumber = 0;
		int vehicleNumber = 0;
		int command = 0; 
		int velocity = 0;
		int n = 0;
		while (splitString.hasMoreElements()){
			String str = splitString.nextToken();
			switch (n) {
			case 0: packageID = 1;
				break;
			case 1: dataForm = Integer.parseInt(str);
				break;
			case 2: platoonNumber = Integer.parseInt(str); 
				break;
			case 3: vehicleNumber = Integer.parseInt(str);
				break;
			case 4: command = Integer.parseInt(str);
				break;
			case 5: velocity = Integer.parseInt(str);
				break;
			default:
				break;
			}
			n++;
		}
		MonitorData monitorData = new MonitorData(packageID, dataForm, platoonNumber, vehicleNumber, command, velocity);

		return monitorData;	
	}

}
