package com.ebstor.robot.corefunctions;

import android.widget.TextView;
import com.ebstor.robot.communication.Communicator;
import jp.ksksue.driver.serial.FTDriver;

import static java.lang.Thread.sleep;

/**
 * this class contains all abilities the robot has
 */
public class Robot {
    
    //private float TRANSLATION_COEFFICIENT = 12f/9f;
    //private float ROTATION_COEFFICIENT = 34.5f/30f;
    // TODO add default values
    /**
     * degrees turned per millisecond
     */
    public static double DEGREE_PER_MILLISECOND;
    /**
     * cm travelled per millisecond
     */
    public static double CM_PER_MILLISECOND;
    /**
     * the interval in ms after which conditions are checked and odometry is updated (when moving forward)
     */
    private long DRIVE_INTERVAL = 500;
    /**
     * the interval in ms after which conditions are checked and odometry is updated (when turning)
     */
    private long TURN_INTERVAL = 250;
    /**
     * the circumference in cm from the within which the robot assumes he has reached it
     */
    private static final int CIRCUMFERENCE_GOAL = 10;
    /**
     * the threshold in cm where the robot starts avoiding an obstacle
     */
    private static final int RANGE_THRESHOLD = 15;

    public Location robotLocation;
    private Communicator com;

    public Robot(TextView textLog, FTDriver driver) {
        this.com = new Communicator(driver,textLog);
        robotLocation = new Location();
    }

    private long distanceToTime(double distance_cm) {
        return (long) (CM_PER_MILLISECOND/distance_cm);
    }

    private double timeToDistance(long distanceToTimeMillis) {
        return distanceToTimeMillis*CM_PER_MILLISECOND;
    }

    private long degreesToTime(double degrees) {
        return (long) (DEGREE_PER_MILLISECOND/degrees);
    }

    private double timeToDegrees(long turnTimeMillis) {
        return turnTimeMillis*DEGREE_PER_MILLISECOND;
    }

    public void connect() {
        com.connect();
    }

    public void disconnect() {
        com.disconnect();
    }

    public void stop() {
        com.stop();
    }
   
    public void drive(int distance_cm) {
        long time = distanceToTime(distance_cm);
        drive();
        sleep_h(time);
        robotLocation.translate(distance_cm);
    }

    /**
     * sets both wheels to 30/100
     */
    public void drive() {
        com.setVelocity((byte) 30, (byte) 30);
    }

    /**
     * drives until a certain sensor condition is met (e.g. obstacle in front)
     */
    public void driveUntil(SensorCondition condition) {
        while (!condition.reached()) {
            drive();
            long t0 = System.currentTimeMillis();
            sleep_h(DRIVE_INTERVAL);
            long dt = System.currentTimeMillis() - t0;
            robotLocation.translate(timeToDistance(dt));
        }
        com.stop();
    }

    /**
     * drives until a certain sensor condition is met and maximally distance_cm
     */
    public void driveUntil(SensorCondition condition, int distance_cm) {
        int time = (int) (1/CM_PER_MILLISECOND/distance_cm);
        boolean reached = false;
        for (int i = 0; i < time/DRIVE_INTERVAL; i++) {
            if (reached = condition.reached()) break;
            drive();
            long t0 = System.currentTimeMillis();
            sleep_h(DRIVE_INTERVAL);
            long dt = System.currentTimeMillis() - t0;
            robotLocation.translate(timeToDistance(dt));
        }
        if (!reached) {    // drive the rest of the time, provided that the stopping condition has not been reached
            drive();
            long t0 = System.currentTimeMillis();
            sleep_h(time%DRIVE_INTERVAL);
            long dt = System.currentTimeMillis() - t0;
            robotLocation.translate(timeToDistance(dt));
        }
    }

    public void turn(double degree) {
        int time = (int) (degreesToTime(degree));
        if (degree <= 0) com.setVelocity((byte)30,(byte) -30);
        else com.setVelocity((byte)-30,(byte)30);
        sleep_h(time);
        com.stop();
        robotLocation.rotate(degree);
    }

    public void turnLeft() {
        com.setVelocity((byte) -30, (byte) 30);
    }

    public void turnRight() {
        com.setVelocity((byte) 30, (byte) -30);
    }

    /**
     * turns left until a certain sensor condition is met (e.g. obstacle parallel to object)
     */
    public void turnLeftUntil(SensorCondition condition) {
        while (!condition.reached()) {
            turnLeft();
            long t0 = System.currentTimeMillis();
            sleep_h(DRIVE_INTERVAL);
            long dt = System.currentTimeMillis() - t0;
            robotLocation.rotate(timeToDegrees(dt));
        }
        com.stop();
    }

    /**
     * turns right until a certain sensor condition is met (e.g. obstacle parallel to object)
     */
    public void turnRightUntil(SensorCondition condition)  {
        while (!condition.reached()) {
            turnRight();
            long t0 = System.currentTimeMillis();
            sleep_h(DRIVE_INTERVAL);
            long dt = System.currentTimeMillis() - t0;
            robotLocation.rotate(-timeToDegrees(dt));
        }
        com.stop();
    }

    /**
     *
     * @return if the goal has been reached
     */
    public boolean bug0(Location goal) {
        // TODO implement (or implement some other bug)
        return true;
    }

    private boolean withinCircumferenceOfGoal(Location goal) {
        return (euclideanDistance(robotLocation,goal) <= CIRCUMFERENCE_GOAL);
    }

    private void turnToGoal(Location goal) throws InterruptedException {
        double angle = Math.toDegrees(Math.atan2(goal.getY()-robotLocation.getY(),goal.getX()-robotLocation.getX()));
        double turningAngle = robotLocation.getTheta() - angle;
        turn(turningAngle);
    }

    private double euclideanDistance(Location a, Location b) {
        return Math.sqrt(Math.pow(a.getX()-b.getX(),2) + Math.pow(a.getY() - b.getY(),2));
    }

    private void sleep_h(long millis) {
        try {
            sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void driveAndStopForObstacles(Integer dist) {
        SensorCondition isObstacle = new SensorCondition() {
            @Override
            public boolean reached() {
                int[] sensors = com.getSensors();
                int left = sensors[0];
                int center = sensors[1];
                int right = sensors[2];

                return (Math.min(left,Math.min(center,right)) <= RANGE_THRESHOLD);
            }
        };
        driveUntil(isObstacle,dist);
    }

}
