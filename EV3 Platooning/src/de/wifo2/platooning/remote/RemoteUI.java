package de.wifo2.platooning.remote;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextArea;

import de.wifo2.platooning.utils.Commands;



/** simple UI used to send commands to the robot
 * @deprecated Use the protocol instead
 * @author Martin
 *
 */

public class RemoteUI extends JFrame {
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	/**ui elements */
	private JComboBox<String> robotBox = new JComboBox<String>();
	private JComboBox<String> commandBox;
	private JTextArea informationText;
	
	private Remote remote;
	
	public RemoteUI(Remote remote){
		this.remote = remote;
		this.setTitle("Lego EV3 Remote");
		this.setSize(new Dimension(500,300));
		this.setLayout(new BorderLayout());
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		createComponents();
		robotBox.addActionListener(new ActionListener(){

			@Override
			// if a robot is selected, display its information (e.g. name)
			public void actionPerformed(ActionEvent e) {
				RemoteUI.this.refreshRobotText();
				
			}
			
		});
	}

	/** setup of ui components */
	private void createComponents() {
		commandBox = new JComboBox<String>(Commands.allCommands);
		commandBox.setSize(200, 50);
		robotBox.setSize(200,50);
		
		JPanel panel1 = new JPanel();
		panel1.setLayout(new GridLayout(1,2));
		panel1.add(robotBox);
		panel1.add(commandBox);
		
		
		JPanel buttonPanel = new JPanel();
		JButton sendButton = new JButton("Send");
		sendButton.setSize(100, 50);
		buttonPanel.add(sendButton);
		sendButton.addActionListener(new ActionListener(){

			@Override
			//if the send button is clicked
			public void actionPerformed(ActionEvent arg0) {
				int selectedRobot = robotBox.getSelectedIndex();
				int selectedCommand = commandBox.getSelectedIndex();
				
				//show error message if no robot or no command is selected
				if(selectedRobot == -1 || selectedCommand == -1){
					JOptionPane.showMessageDialog(RemoteUI.this,
						    "Please select a robot and a command!",
						    "Could not send command",
						    JOptionPane.ERROR_MESSAGE);
				}
				
				//send selected command to the selected robot
				else{
					RemoteService service = remote.getServer().getServices().get(robotBox.getItemAt(selectedRobot));
					service.setCommand(commandBox.getItemAt(selectedCommand));
					service.setHasToSend(true);
				}
				
			}
			
		});
		
		informationText = new JTextArea();
		informationText.setText("Robot information will be displayed here!");
		informationText.setEditable(false);
		
		this.add(panel1, BorderLayout.NORTH);
		this.add(informationText, BorderLayout.CENTER);
		this.add(buttonPanel, BorderLayout.SOUTH);
		
		
	}
	
	protected void refreshRobotText(){
		//TODO: display whole text(IP, #persons etc)
		if(robotBox.getSelectedItem() != null){
		StringBuilder text = new StringBuilder("Name: ");
		text.append(robotBox.getSelectedItem());
		text.append("\n");
		text.append("Position: ");

			text.append(remote.getRobotPosition(robotBox.getSelectedItem().toString()));
		
		informationText.setText(text.toString());
		}
		else{
			informationText.setText("Robot information will be displayed here");
		}
		
	}
	
	/** a new robot is added to the combo box */
	public void addRobot(String robotName){
		robotBox.insertItemAt(robotName, robotBox.getItemCount());
	}

}
