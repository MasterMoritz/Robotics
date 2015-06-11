package com.ebstor.robot.controller;

import com.ebstor.robot.State;
import com.ebstor.robot.corefunctions.Robot;

/**
 * Created by johannes on 6/11/15.
 */
public class ObstacleAvoid implements Runnable {

    public State state;
    public Robot robot;
    final static String TAG = "ObstacleAvoid";


    @Override
    public void run() {
        for (;;) {
            avoidObstacle();
        }
    }

    private void avoidObstacle() {
        robot.stop();
        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


    }
}
