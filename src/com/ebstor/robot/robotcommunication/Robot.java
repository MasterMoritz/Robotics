package com.ebstor.robot.robotcommunication;

import android.hardware.usb.UsbManager;
import android.widget.TextView;
import jp.ksksue.driver.serial.FTDriver;

import static java.lang.Thread.sleep;

/**
 * Created by johannes on 3/23/15.
 */
public class Robot {

    private FTDriver com;
    private TextView textLog ;
    private float TRANSLATION_COEFFICIENT = 12f/9f;
    private float ROTATION_COEFFICIENT = 34.5f/30f;
    /**
     * the circumference in cm from the within which the robot assumes he has reached it
     */
    private final int CIRCUMFERENCE_GOAL = 10;
    /**
     * the threshold in cm where the robot starts avoiding an obstacle
     */
    private int RANGE_THRESHOLD = 20;
    public Location robotLocation;

    /**
     * seconds per cm
     */
    private float TRANSLATION_SLEEPTIME = 6f/90f+1;
    /**
     * seconds per degree
      */
    private float ROTATION_SLEEPTIME = 1.8f/90f;

    public void setTRANSLATION_SLEEPTIME(float TRANSLATION_SLEEPTIME) {
        this.TRANSLATION_SLEEPTIME = TRANSLATION_SLEEPTIME;
    }

    public void setROTATION_COEFFICIENT(float ROTATION_COEFFICIENT) {
        this.ROTATION_COEFFICIENT = ROTATION_COEFFICIENT;
    }

    public void setTRANSLATION_COEFFICIENT(float TRANSLATION_COEFFICIENT) {
        this.TRANSLATION_COEFFICIENT = TRANSLATION_COEFFICIENT;
    }

    public void setROTATION_SLEEPTIME(float ROTATION_SLEEPTIME) {
        this.ROTATION_SLEEPTIME = ROTATION_SLEEPTIME;
    }

    public Robot(TextView textLog, FTDriver com) {
        this.textLog = textLog;
        this.com = com;
        robotLocation = new Location();
    }

    public void connect() {
        if (com.begin(9600)) textLog.setText("connected! ");
        else textLog.setText("connection not okay");
    }

    public void disconnect() {
        com.end();
        if (!com.isConnected()) textLog.append("disconnected! ");
    }

    private void comWrite(byte[] data) {
        if (com.isConnected()) {
            com.write(data);
        } else {
            textLog.append("not connected\n");
        }
    }

    private String comRead() {
        String s = "";
        int i = 0;
        int n = 0;
        while (i < 3 || n > 0) {
            byte[] buffer = new byte[256];
            n = com.read(buffer);
            s += new String(buffer, 0, n);
            i++;
        }
        return s;
    }

    private String comReadWrite(byte[] data) {
        com.write(data);
        try {
            sleep(100);
        } catch (InterruptedException e) {
            // ignore
        }
        return comRead();
    }

    public void wait(Integer dist_cm, Integer velocity, Integer angle) {
        // TODO implement
        Integer sleeptime = Math.max((int)(1000*dist_cm*TRANSLATION_SLEEPTIME),(int)(1000*angle*TRANSLATION_SLEEPTIME));
        try {
            sleep(sleeptime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


    public void moveForward() {
        textLog.append("moving forward...\n");
        comWrite(new byte[] {'w', '\r','\n'});
    }

    public void drive(Integer distance_cm) {
        distance_cm = (int)(TRANSLATION_COEFFICIENT * distance_cm);
        for (int i = 0; i < distance_cm/10; i++) {
            comReadWrite(
                    new byte[] { 'k',(byte)90,'\r', '\n' }
            );
            wait(90,0, 0);
            robotLocation.translate(90);
        }
        comReadWrite(
                new byte[] { 'k',(byte) (distance_cm%90), '\r', '\n' }
        );
        wait(distance_cm%90,0,0);
        robotLocation.translate(distance_cm%90);

    }

    public void turn(Integer degree) {
        degree = (int) (ROTATION_COEFFICIENT * degree);
        for (int i = 0; i < degree/90; i++) {
            comReadWrite(
                    new byte[] { 'l',(byte) 90, '\r', '\n' }
            );
            wait(0,0,90);
            robotLocation.rotate(90);
        }
        comReadWrite(
                new byte[] { 'l',(byte) (degree%90), '\r', '\n' }
        );
        wait(0,0,degree%90);
        robotLocation.rotate(90);

    }


    public void stop() {
        textLog.append("stopping...\n");
        comWrite(new byte[] {'s', '\r','\n'});
    }


    public int checkRange() {
        // TODO implement
        String answer = comReadWrite(
                new byte[] {'q','\r','\n'}
        );
        return 100;
    }

    public void driveAndStopForObstacles(int dist_cm) {
        for (int i = 0; i < dist_cm/10; i++) {
            drive(10);
            if (isObstacle()) stop();
        }

    }

    private boolean isObstacle() {
        return (checkRange() <= RANGE_THRESHOLD);
    }

    /**
     *
     * @return if the goal has been reached
     */
    public boolean bug0(Location goal) {
        turnToGoal(goal);
        while(!withinCircumferenceOfGoal(goal)) {
            drive(10);
            while (isObstacle()) {
                turn(90);
                drive(10);
                turn(-90);
            }
        }

        return true;
    }

    private boolean withinCircumferenceOfGoal(Location goal) {
        return (euclideanDistance(robotLocation,goal) <= CIRCUMFERENCE_GOAL);
    }

    private void turnToGoal(Location goal) {
        int angle = (int)Math.toDegrees(Math.atan2(goal.getY()-robotLocation.getY(),goal.getX()-robotLocation.getX()));
        int turningAngle = robotLocation.getTheta() - angle;
        turn(turningAngle);
    }

    private int euclideanDistance(Location a, Location b) {
        return (int) Math.sqrt(Math.pow(a.getX()-b.getX(),2) + Math.pow(a.getY() - b.getY(),2));
    }

}
