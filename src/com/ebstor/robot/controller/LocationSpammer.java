package com.ebstor.robot.controller;

import com.ebstor.robot.MainActivity;
import com.ebstor.robot.corefunctions.Robot;

public class LocationSpammer implements Runnable{
	boolean stop;
	Robot robot;
	
	public LocationSpammer(Robot robot) {
		stop = false;
		this.robot = robot;
	}
	
	@Override
	public void run() {
		while(!stop) {
			try {Thread.sleep(50);} catch (InterruptedException e) {}
			System.out.println(robot.robotLocation);
		}
		
	}
	
	public void stop() {
		stop = true;
	}

}
