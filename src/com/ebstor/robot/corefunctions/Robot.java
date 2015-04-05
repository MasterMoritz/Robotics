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
    /** degrees turned per millisecond */
    public static double DEGREE_PER_MILLISECOND;
    
    /** cm travelled per millisecond */
    public static double CM_PER_MILLISECOND;
    
    /** the interval in ms after which conditions are checked and odometry is updated (when moving forward) */
    private long DRIVE_INTERVAL = 500;
    
    /** the interval in ms after which conditions are checked and odometry is updated (when turning) */
    private long TURN_INTERVAL = 250;
    
    /** the circumference in cm from the within which the robot assumes he has holds it */
    private static final int CIRCUMFERENCE_GOAL = 5;
    
    /** the threshold in cm where the robot starts avoiding an obstacle */
    private static final int RANGE_THRESHOLD = 15;

    /** holds the pose of the robot */
    public Location robotLocation;
    
    /** holds the desired pose of the robot after reaching its destination */
    public Location goal;
    
    /** serial communicator */
    public Communicator com;

    /** setup serial communicator and initialize locations */
    public Robot(TextView textLog, FTDriver driver) {
        this.com = new Communicator(driver,textLog);
        robotLocation = new Location(0, 0, 0);
        goal = new Location(0, 0, 0);
    }

    /**
     * @return time[ms] needed to drive the given distance[cm]
     */
    private long distanceToTime(double distance_cm) {
        return (long) ( (1/CM_PER_MILLISECOND) * distance_cm);
    }

    /**
     * @return distance[cm] covered in the given time[ms]
     */
    private double timeToDistance(long time_ms) {
        return time_ms*CM_PER_MILLISECOND;
    }

    /**
     * @return time[ms] needed to turn the given angle[�]
     */
    private long degreesToTime(double degrees) {
        return (long) ( (1/DEGREE_PER_MILLISECOND) * degrees);
    }

    /**
     * @return angle[�] covered in the given time[ms]
     */
    private double timeToDegrees(long time_ms) {
        return time_ms*DEGREE_PER_MILLISECOND;
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
   
    /**
     * drive straight a certain distance and update robot location 
     * @param distance_cm : the distance[cm] to drive
     * @param velocity : the speed of the robot
     */
    public void drive(int distance_cm, int velocity) {
    	//drive distance
        long time = distanceToTime(distance_cm);
        com.setVelocity(velocity, velocity);
        sleep_h(time);
        com.stop();
        
        //update robot location
        robotLocation.translate(distance_cm);
    }

    /**
     * drive straight with 30 velocity
     */
    public void drive() {
        com.setVelocity((byte) 30, (byte) 30);
    }

    /**
     * drives until a certain sensor condition is met (e.g. obstacle in front)
     */
    public void driveUntil(SensorCondition condition) {
        while (!condition.holds()) {
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
            if (reached = condition.holds()) break;
            drive();
            long t0 = System.currentTimeMillis();
            sleep_h(DRIVE_INTERVAL);
            long dt = System.currentTimeMillis() - t0;
            robotLocation.translate(timeToDistance(dt));
        }
        if (!reached) {    // drive the rest of the time, provided that the stopping condition has not been holds
            drive();
            long t0 = System.currentTimeMillis();
            sleep_h(time%DRIVE_INTERVAL);
            long dt = System.currentTimeMillis() - t0;
            robotLocation.translate(timeToDistance(dt));
        }
        com.stop();
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
        while (!condition.holds()) {
            turnLeft();
            long t0 = System.currentTimeMillis();
            sleep_h(TURN_INTERVAL);
            long dt = System.currentTimeMillis() - t0;
            robotLocation.rotate(timeToDegrees(dt));
        }
        com.stop();
    }

    /**
     * turns right until a certain sensor condition is met (e.g. obstacle parallel to object)
     */
    public void turnRightUntil(SensorCondition condition)  {
        while (!condition.holds()) {
            turnRight();
            long t0 = System.currentTimeMillis();
            sleep_h(DRIVE_INTERVAL);
            long dt = System.currentTimeMillis() - t0;
            robotLocation.rotate(-timeToDegrees(dt));
        }
        com.stop();
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
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void driveAndStopForObstacles(Integer dist) {
        SensorCondition isObstacle = new SensorCondition() {
            @Override
            public boolean holds() {
                int[] sensors = com.getSensors();
                int left = sensors[0];
                int center = sensors[1];
                int right = sensors[2];

                return (Math.min(left,Math.min(center,right)) <= RANGE_THRESHOLD);
            }
        };
        driveUntil(isObstacle,dist);
    }

    public void bug2(Location goal) {
        this.goal = goal;
    }

    private boolean mlineEncountered() {
        return distanceToMline() <= 3; // (cm) this depends on how far robot travels between checks, can therefore be reduced
    }

    /**
     * computes the shortest distance between the current robot location and the m-line
     */
    private double distanceToMline() {
        double x = robotLocation.getX();
        double y = robotLocation.getY();
        double normalLength = Math.sqrt(Math.pow(goal.getX(),2)+Math.pow(goal.getY(),2));
        return Math.abs(x*goal.getY() - y*(goal.getX()))/normalLength;
    }

    private void followObstacleUntil(SensorCondition leavingCondition) {
        // condition holds when the robot is aligned parallel to an obstacle
        SensorCondition alignedParallel = new SensorCondition() {
            @Override
            public boolean holds() {
                int sensors[] = com.getSensors();
                int left = sensors[0];
                int center = sensors[1];
                int right = sensors[2];

                //TODO implement
                return false;
            }
        };
        // holds when the leaving condition holds or the robot is no longer correctly following the obstacle
        SensorCondition driveCondition = new SensorCondition() {
            @Override
            public boolean holds() {
                return !alignedParallel.holds() || leavingCondition.holds();
            }
        };

        turnLeftUntil(alignedParallel); // left-turning robot

        while(true) {
            /* this are the sensor values when the robot is nicely aligned */
            int sensors[] = com.getSensors();
            int alignedRight = sensors[2];

            driveUntil(driveCondition);

            if (leavingCondition.holds())
                return;
            else {
            /* here the robot is not aligned correctly,
            now decide whether to correct the orientation to the right or to the left */
                sensors = com.getSensors();
                int badlyAlignedRight = sensors[2];
                if (badlyAlignedRight > alignedRight)   // new distance to the right is greater so we must turn right
                    turnRightUntil(alignedParallel);
                else
                    turnLeftUntil(alignedParallel);
            }
        }

    }
}
