package com.ebstor.robot.corefunctions;

import com.ebstor.robot.communication.Communicator;

/**
 * Created by johannes on 3/29/15.
 */
public abstract class SensorCondition {
	public Robot robot;
	
	public SensorCondition(Robot robot) {
		this.robot = robot;
	}
	
    /**
     * important: must check sensors of the robot so that it stops!
     * @return if the condition holds
     */
    public abstract boolean holds();
    
    public int getInt() {
    	return 0;
    }
    public double getDouble() {
    	return 0;
    }
    
    public void reset() {
    	
    }
}
