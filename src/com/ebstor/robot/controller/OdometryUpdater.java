package com.ebstor.robot.controller;

import com.ebstor.robot.corefunctions.Location;
import com.ebstor.robot.corefunctions.Robot;

import static java.lang.Thread.sleep;

/**
 * Created by johannes on 4/7/15.
 */
public class OdometryUpdater implements Runnable {

    private static final long INTERVAL = 5;

    public RobotAction action;
    private Location location;
    public boolean isRunning;

    public OdometryUpdater(Location location) {
        action = null;
        this.location = location;
        isRunning = true;
    }

    @Override
    public void run() {
        long t0 = System.currentTimeMillis();
        while(isRunning) {
            long dt = System.currentTimeMillis() - t0;
            switch(action) {
                case MOVE_FORWARD:
                    location.translate(Robot.timeToDistance(dt));
                    break;
                case MOVE_BACKWARD:
                    location.translate(-Robot.timeToDistance(dt));
                    break;
                case TURN_LEFT:
                    location.rotate(Robot.timeToDegrees(dt));
                    break;
                case TURN_RIGHT:
                    location.rotate(-Robot.timeToDegrees(dt));
                    break;
            }
            try {
                sleep(INTERVAL);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            t0 = System.currentTimeMillis();
        }
    }

    public void stop() {
        isRunning = false;
    }


}
