package com.ebstor.robot.corefunctions;

import android.widget.TextView;

import com.ebstor.robot.communication.Communicator;

import jp.ksksue.driver.serial.FTDriver;
import static java.lang.Thread.sleep;

/**
 * this class contains all abilities the robot has
 */
public class Robot {
	
	/** speed of robot */
	public static int VELOCITY = 20;
	
    /** degrees turned per millisecond for velocity */
    public static double DEGREE_PER_MILLISECOND = 0.11;
    
    /** cm travelled per millisecond for velocity */
    public static double CM_PER_MILLISECOND = 0.02055;
    
    /** the interval in ms after which conditions are checked and odometry is updated (when moving forward) */
    private long DRIVE_INTERVAL = 500;
    
    /** the interval in ms after which conditions are checked and odometry is updated (when turning) */
    private long TURN_INTERVAL = 250;
    
    /** the circumference in cm from the within which the robot assumes he has holds it */
    private static final int CIRCUMFERENCE_GOAL = 5;
    
    /** the threshold in cm where the robot starts avoiding an obstacle */
    private static final int RANGE_THRESHOLD = 15;

    /** the threshold in cm at which the robot may start driving again */
    private static final int SOFT_THRESHOLD = 30;
    
    //TODO measure the real angle between the various LOS
    /** the angle between the left end of the LOS of the front sensor and the side sensor */
    private static final double ANGLE_FRONT_SIDE = 20;
    private static final double ANGLE_RIGHT_LEFT = 40;
    
    /** the minimum angle we have to turn to pass an obstacle point in front without collision */
    private static final int MINIMUM_TURN = 20;
    
    /** the minimum distance we have to drive after detecting a wall end, should be about the length of the robot */
    private static final int MINIMUM_DRIVE = 20;
    
    /** holds the pose of the robot */
    public Location robotLocation;
    
    /** holds the desired pose of the robot after reaching its destination */
    public Location goal;
    
    /** the last point on the m-line that the robot hit */
    public Location m_point;
    
    /** serial communicator */
    public Communicator com;

    /** setup serial communicator and initialize locations */
    public Robot(TextView textLog, FTDriver driver) {
        this.com = new Communicator(driver,textLog);
        robotLocation = new Location(0, 0, 0);
        goal = new Location(0, 0, 0);
        m_point = new Location(0, 0, 0);
    }

    /**
     * @return time[ms] needed to drive the given distance[cm]
     */
    public static long distanceToTime(double distance_cm) {
        return (long) Math.abs((1 / CM_PER_MILLISECOND) * distance_cm);
    }

    /**
     * @return distance[cm] covered in the given time[ms]
     */
    public static double timeToDistance(long time_ms) {
        return (time_ms)*CM_PER_MILLISECOND;
    }

    /**
     * @return time[ms] needed to turn the given angle[�]
     */
    public static long degreesToTime(double degrees) {
        return (long) Math.abs((1/DEGREE_PER_MILLISECOND) * degrees);
    }

    /**
     * @return angle[�] covered in the given time[ms]
     */
    public static double timeToDegrees(long time_ms) {
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
     */
    public void drive(int distance_cm) {
    	//drive distance
        long time = distanceToTime(distance_cm);
        
        //drive backward if distance is negative
        if (distance_cm < 0) {
        	VELOCITY *= -1;
        }
        
        com.setVelocity(VELOCITY, VELOCITY);
        sleep_h(time);
        com.stop();
        
        //update robot location
        robotLocation.translate(distance_cm);
    }

    /**
     * drive straight
     */
    public void drive() {
        com.setVelocity((byte) VELOCITY, (byte) VELOCITY);
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
        long t0 = System.currentTimeMillis();
        
        drive();
        while(!condition.holds()) {}
        
        long dt = System.currentTimeMillis() - t0;
        com.stop();
            
        //update robot pose
        robotLocation.translate(timeToDistance(dt));
    }

    public void turn(double degree) {
        if (degree != 0) {
            long time = degreesToTime(degree);
            if (degree < 0) com.setVelocity((byte)VELOCITY,(byte) -VELOCITY);
            else com.setVelocity((byte)-VELOCITY,(byte)VELOCITY);
            sleep_h(time);
            com.stop();
            robotLocation.rotate(degree);
        }

    }

    public void turnLeft() {
        com.setVelocity((byte) -VELOCITY, (byte) VELOCITY);
    }

    public void turnRight() {
        com.setVelocity((byte) VELOCITY, (byte) -VELOCITY);
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

    public void turnToGoal() {
    	try {
	        double angle = Math.toDegrees(Math.atan2(goal.getY()-robotLocation.getY(),goal.getX()-robotLocation.getX()));
            double turningAngle =  angle - robotLocation.getTheta();
            if (turningAngle > 180) turningAngle = 360 - turningAngle;
            if (turningAngle < -180) turningAngle = 360 + turningAngle;
            turn(turningAngle);
            com.setText(angle + " | " + turningAngle + " | " + robotLocation.getTheta());
    	} catch(Exception e) {
    		com.setText("failed to turn towards goal");
            com.setText(e.toString());
    	}
    }

    private double euclideanDistance(Location a, Location b) {
        return Math.sqrt(Math.pow(a.getX()-b.getX(),2) + Math.pow(a.getY() - b.getY(),2));
    }

    /** sleep wrapper */
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
            
            @Override
            public void init() {}
        };
        driveUntil(isObstacle,dist);
    }

    public void setGoal(Location goal) {
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

    /** the robot should realign to whatever obstacle it is facing now <br>
     * @param direction : +1 -> turn right, -1 -> turn left
     */
    private void realign(int direction) {
    	
    	//left/right sensor now measures the distance of front, this should save time because front sensor is slow like shiet
    	turn(ANGLE_FRONT_SIDE * direction);
    	
    	turn(MINIMUM_TURN);
    	
    	int turnDirection = 0; //left sensor if turning right
    	if (direction < 0) {
    		turnDirection = 2; //right sensor if turning left
    	}
    	
    	//sensor[0] = left
    	//sensor[1] = middle
    	//sensor[2] = right
    	int [] sensor = com.getSensors();
    	
    	//get front into soft treshold
    	while(sensor[turnDirection] < SOFT_THRESHOLD) {
    		turn(3 * (direction));
    		sensor = com.getSensors();
    	}
    	
    	//front sensor now measures front again
    	turn(-ANGLE_FRONT_SIDE * (-direction));
    	
    	//get left/right into soft threshold
    	sensor = com.getSensors();
    	while(sensor[turnDirection] < SOFT_THRESHOLD) {
    		turn(3 * (direction));
    		sensor = com.getSensors();
    	}
    }
    
    /**
     * Calculate the right corner point of the Obstacle in front of the robot <br>
     * this method assumes that the front sensor is facing the obstacle
     */
    public Location calculateObstaclePoint() {
    	//turn right until the left sensor can't detect the obstacle anymore
    	int leftSensor_new = com.getSensors()[0];
    	int leftSensor_old = leftSensor_new;
    	int rotated = 0;
    	int accuracy = -1;
    	
    	while ( (leftSensor_new - leftSensor_old) < 30) { //maybe have to play with the < value, but 30 should be alright
    		turn(accuracy);
    		rotated += accuracy;
    		leftSensor_old = leftSensor_new;
    		leftSensor_new = com.getSensors()[0];
    	}
    	
    	//update robot location
    	robotLocation.rotate(rotated);
    	
    	//calculate the corner point which we want to pass
    	double alpha = robotLocation.getTheta() + ANGLE_FRONT_SIDE;
    	double hyp = leftSensor_old;
    	double gk = Math.sin(alpha) * hyp; //difference of Robot_y and Point_y
    	double ak = Math.cos(alpha) * hyp; //difference of Robot_x and Point_x
    	
    	return new Location(ak + robotLocation.getX(), gk + robotLocation.getY(), alpha);
    	
    }
    
    /**
     * The Roboter should change into following mode 
     * leave following mode if it hits the m-line (closer to goal than m_point) again
     * @param turnDirection : 1 ->turn clockwise, -1 -> turn counterclockwise <br>
     * note: for now always clockwise turning should be used because the param is not fully implemented
     */
    public void followObstacle(int turnDirection) {
    	com.setText("following obstacle");
        
    	int[] sensors = com.getSensors();
    	int distance = sensors[1] - RANGE_THRESHOLD - 1;
    	
    	//drive to the obstacle
    	if (distance > 0) {
    		drive(distance);
    	}
    	
    	//now realign
        realign(turnDirection);
        

        // stop if misaligned or m-line hit or wall ends
        SensorCondition driveCondition = new SensorCondition() {
        	public int[] sensors_old;
        	public int[] sensors_new;
        	
            @Override
            public boolean holds() {
            	sensors_new = com.getSensors();

            	if ((sensors_new[0] - sensors_old[0]) > 20 || sensors_new[0] < RANGE_THRESHOLD || sensors_new[1] < RANGE_THRESHOLD 
            		|| mlineEncountered()) {
            		return true;
            	}
            	
            	sensors_old[0] = sensors_new[0];
            	sensors_old[1] = sensors_new[1];
            	sensors_old[2] = sensors_new[2];
                return false;
            }
            
            @Override
            public void init() {
            	sensors_old = com.getSensors();
            	sensors_new = com.getSensors();
            }
        };

        while(true) {
        	driveCondition.init();
            driveUntil(driveCondition);

            //m-line hit
            if (mlineEncountered()) {
            	turnToGoal();
                return;
            }
            //misaligned or wall ends
            else {
            	sensors = com.getSensors();
            	if (sensors[0] < RANGE_THRESHOLD || sensors[1] < RANGE_THRESHOLD) {
            		com.setText("Realign");
            		realign(turnDirection);
            	}
            	else {
            		com.setText("Wall ended");
            		//the current wall has ended, thus turn around to continue following the obstacle
            		drive(MINIMUM_DRIVE);
            		turn(-90);
            	}
            }
        }

    }
}
