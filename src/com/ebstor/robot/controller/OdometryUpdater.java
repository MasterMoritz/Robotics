package com.ebstor.robot.controller;

import com.ebstor.robot.corefunctions.Location;
import com.ebstor.robot.corefunctions.Robot;

import static java.lang.Thread.sleep;

/**
 * Created by johannes on 4/7/15.
 */
public class OdometryUpdater implements Runnable {

    private static final long INTERVAL = 1000;

    private RobotAction action;
    private Location robotLocation;
    public boolean isRunning;

    public OdometryUpdater(Location location) {
        action = null;
        this.robotLocation = location;
        isRunning = true;
    }

    public void setAction(RobotAction action) {
        this.action = action;
    }

    @Override
    public void run() {
        long t0 = System.currentTimeMillis();
        while(isRunning) {
            long dt = System.currentTimeMillis() - t0;
            t0 = System.currentTimeMillis();
            if (action != null)
                switch(action) {
                    case MOVE_FORWARD:
                        robotLocation.translate(Robot.timeToDistance(dt));
                        break;
                    case MOVE_BACKWARD:
                        robotLocation.translate(-Robot.timeToDistance(dt));
                        break;
                    case TURN_LEFT:
                        robotLocation.rotate(Robot.timeToDegrees(dt));
                        break;
                    case TURN_RIGHT:
                        robotLocation.rotate(-Robot.timeToDegrees(dt));
                        break;
                }
            try {
                sleep(INTERVAL);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void stop() {
        isRunning = false;
    }


}
