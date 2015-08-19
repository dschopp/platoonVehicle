package de.wifo2.platooning.utils;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;

public class Map {

	public Map(){
		retrieveExitRampsFromMap();
	}
	private ArrayList<ExitRamp> exitRamps = new ArrayList<ExitRamp>();
	private HashSet<Integer> starts = new HashSet<Integer>();
	private HashSet<Integer> ends = new HashSet<Integer>();
	
	public ArrayList<ExitRamp> getExitRamps(){
		return exitRamps;
	}
	
	private void retrieveExitRampsFromMap(){
		int index = 0;
		String line = "";
		String name = "";
		int start = 0;
		int end = 0;
		try {
			BufferedReader reader = new BufferedReader(new FileReader("map.txt"));
			line = reader.readLine();
			while(line != null){
					String[] splitString = line.split("_");
					
					index = Integer.parseInt(splitString[0]);
					name = splitString[1];
					start = Integer.parseInt(splitString[2]);
					end = Integer.parseInt(splitString[3]);
					
					starts.add(start);
					ends.add(end);	
					exitRamps.add(new ExitRamp(index,name, start, end));
					line = reader.readLine();
			}
				reader.close();
	
		} catch (IOException e) {
			e.printStackTrace();
		}
	}


}
