package de.wifo2.platooning.utils;

public class ExitRamp {
	
	private int index;
	private String name;
	private int start;
	private int end;
	
	
	public ExitRamp(int index, String name, int start, int end) {
		this.index = index;
		this.name = name;
		this.start = start;
		this.end = end;
	}
	public String getName() {
		return name;
	}
	public void setName(String name) {
		this.name = name;
	}
	public int getStart() {
		return start;
	}
	public void setStart(int start) {
		this.start = start;
	}
	public int getEnd() {
		return end;
	}
	public void setEnd(int end) {
		this.end = end;
	}
	public int getIndex() {
		return index;
	}
	public void setIndex(int index) {
		this.index = index;
	}
	

}
