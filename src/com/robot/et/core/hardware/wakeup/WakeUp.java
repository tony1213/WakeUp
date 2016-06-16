package com.robot.et.core.hardware.wakeup;

public class WakeUp {
	
	static {
		System.loadLibrary("WakeUp");
	}

	public static native int open(String path, int oFlag); 
	public static native int getWakeUpState(int fid);
	public static native int getWakeUpDegree();
	public static native int setGainDirection(int direction);
    public static native int close(int fId); 
    
}
