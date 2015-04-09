 package com.ebstor.robot.corefunctions;

import java.util.EmptyStackException;

import android.widget.TextView;

import com.ebstor.robot.communication.Communicator;

import jp.ksksue.driver.serial.FTDriver;
import static java.lang.Thread.sleep;

/**
 * this class contains all abilities the robot has
 */
public class Robot {
	
	/** speed of robot */
	public static int VELOCITY = 15;
	
    /** degrees turned per millisecond for velocity */
    public static double DEGREE_PER_MILLISECOND = 0.0820;
    
    /** cm travelled per millisecond for velocity */
    public static double CM_PER_MILLISECOND = 0.0142;
    
    /** the interval in ms after which conditions are checked and odometry is updated (when moving forward) */
    private long DRIVE_INTERVAL = 500;
    
    /** the interval in ms after which conditions are checked and odometry is updated (when turning) */
    private long TURN_INTERVAL = 250;
    
    private static final int ROBOT_WIDTH = 20;
	private static final int ROBOT_LENGTH = 16;
    
    /** the circumference in cm from the within the robot assumes he has reached the goal*/
    private static final int CIRCUMFERENCE_GOAL = 5;
	
    private static final int CIRCUMFERENCE_MLINE = 5;
	
    /** the threshold in cm where the robot starts avoiding an obstacle */
    private static final int RANGE_THRESHOLD = 20;

    /** the threshold in cm at which the robot may start driving again */
    private static final int SOFT_THRESHOLD = 30;
    
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
     * @return whether the robot has reached the goal
     */
    public boolean reachedGoal() {
    	if (Math.abs(robotLocation.getX() - goal.getX()) <= CIRCUMFERENCE_GOAL
    		&& Math.abs(robotLocation.getY() - goal.getY()) <= CIRCUMFERENCE_GOAL)
    	{
    		return true;
    	}
    	
    	else {
    		return false;
    	}
    }
    
    
    /**
     * drive straight a certain distance and update robot location 
     * @param distance_cm : the distance[cm] to drive
     */
    public void drive(double distance_cm) {
    	int velocity = VELOCITY;
    	
    	//drive distance
        long time = distanceToTime(distance_cm) - 50;
        if (time < 0) {
        	time = 0;
        }
        
        //drive backward if distance is negative
        if (distance_cm < 0) {
        	velocity *= -1;
        }
        
        com.setVelocity(velocity, velocity);
        sleep_h(time-50);
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
    	
        com.setVelocity((byte)VELOCITY, (byte)VELOCITY);
        
        long t0 = System.currentTimeMillis();
        while(!condition.holds()) {}
        
        com.stop();
        
        long dt = System.currentTimeMillis() - t0;
        
        //update robot pose
        robotLocation.translate(timeToDistance(dt));
    }

    /**
     * drives until a certain sensor condition is met and maximally distance_cm
     * @return whether condition was fulfilled or not
     */
    public boolean driveUntil(SensorCondition condition, int distance_cm) {
    	int velocity = VELOCITY * Integer.signum(distance_cm);
        boolean reached = false;

        long time = distanceToTime(distance_cm);

        for (int i = 0; i < time/DRIVE_INTERVAL; i++) {
            if (condition.holds()) {
            	reached = true;
            	break;
            }
            com.setVelocity(velocity, velocity);
            long t0 = System.currentTimeMillis();
            sleep_h(DRIVE_INTERVAL);
            long dt = System.currentTimeMillis() - t0;
            robotLocation.translate(timeToDistance(dt));
        }
        // drive the rest of the time, provided that the stopping condition has not been hold
        if (!reached) {    
            long t0 = System.currentTimeMillis();
            sleep_h(time%DRIVE_INTERVAL);
            long dt = System.currentTimeMillis() - t0;
            robotLocation.translate(timeToDistance(dt));
        }
        com.stop();
        
        return reached;
    }

    public void turn(double degree) {
        if (degree != 0) {
            long time = degreesToTime(degree);
            if (time < 0) {
            	time = 0;
            }
            if (degree < 0) com.setVelocity((byte)VELOCITY,(byte) -VELOCITY);
            else com.setVelocity((byte)-VELOCITY,(byte)VELOCITY);
            sleep_h(time-50);
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

    public void turnToLocation(Location point) {
    	try {
	        double angle = Math.toDegrees(Math.atan2(point.getY()-robotLocation.getY(),point.getX()-robotLocation.getX()));
            double turningAngle =  angle - robotLocation.getTheta();
            if (turningAngle > 180) turningAngle = 360 - turningAngle;
            if (turningAngle < -180) turningAngle = 360 + turningAngle;
            turn(turningAngle);
            com.setText(angle + " | " + turningAngle + " | " + robotLocation.getTheta());
    	} catch(Exception e) {
    		com.setText("failed to turn towards location");
            com.setText(e.toString());
    	}
    }
    
    public double euclideanDistance(Location a, Location b) {
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

    public boolean mlineEncountered() {
        return distanceToMline() <= 3; // (cm) this depends on how far robot travels between checks, can therefore be reduced
    }

	/** alternative to mlineEncountered */
	public boolean encounteredMline() {
		double rx = robotLocation.getX();
		double ry = robotLocation.getY();
		
		double gx = goal.getX();
		double gy = goal.getY();
		
		int rho = (int)(Math.atan(ry / rx));
		int gho = (int)(Math.atan(gy / gx));
		
		return (Math.abs(rho - gho) <= CIRCUMFERENCE_MLINE);
	}
	public boolean encounteredMline(int tolerance) {
		double rx = robotLocation.getX();
		double ry = robotLocation.getY();
		
		double gx = goal.getX();
		double gy = goal.getY();
		
		int rho = (int)(Math.atan(ry / rx));
		int gho = (int)(Math.atan(gy / gx));
		
		return (Math.abs(rho - gho) <= tolerance);
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

    /**
     * Use At Your Own Risk (or the risk of the robot)
     */
    public void randomBerserkerMode() {
    	int b = 12;
    	int m = 0;
    	while(m < 4) {
    		com.setVelocity((int)(Math.random() * 255), (int)(Math.random() * 255));
    		sleep_h((long) (Math.random() * 500 + 200));
    		if (b > (int)(Math.random()*100)) {
    			m += 1;
    		}
    	}
    }
    
    /** drives a certain distance and checks afterwards if condition is fulfilled */
    private boolean driveAndCheck(double distance_cm, SensorCondition condition) {
    	drive(distance_cm);
    	return condition.holds();
    }
    
    /**
     * keep distance to an obstacle
     * @param direction : 1 -> turn counter-clockwise , -1 -> turn clockwise
     * @return the last measured distance
     */
    private int keepDistance(int direction) {
    	com.append("keeping distance " + Integer.toString(direction));
		int[] s_new;
		int s_old = 0;
		
		int turnDirection = 2; //right sensor if turning left
    	if (direction < 0) {
    		turnDirection = 0; //left sensor if turning right
    	}
		
		for (int i = 0; i < 4; i++) {
			s_new = com.getSensors();
			s_old = s_new[turnDirection];
			
			//keep distance to wall
			while( (s_new[turnDirection] - s_old) < 21) {
				drive(s_new[turnDirection] - RANGE_THRESHOLD);
				turn(3*direction);
				s_old = s_new[turnDirection];
				s_new = com.getSensors();
			}
			
			//wall ended, drive around corner
			Location temp = new Location(robotLocation);
			turn(20*direction);
			drive(s_old + ROBOT_LENGTH);
			
			turnToLocation(temp);
			
			s_new = com.getSensors();
			s_old = s_new[turnDirection];
			
			while( (s_new[turnDirection] - s_old) < 21) {
				turn(3*direction);
				s_old = s_new[turnDirection];
				s_new = com.getSensors();
			}
			
			turn(20*direction);
			drive(s_old + ROBOT_LENGTH);
			
			turnToLocation(temp);
			turn(3*direction);
		}
		
		return s_old;
	}
	
    
    public void driveUntilObstacle() {
        int[] s = new int[3];
        
        com.setVelocity(VELOCITY, VELOCITY);
        long t0 = System.currentTimeMillis();
        while(true) {
        	s = com.getSensors();
        	if (s[0] <= RANGE_THRESHOLD || s[1] <= RANGE_THRESHOLD || s[2] <= RANGE_THRESHOLD) {
        		break;
        	}
        }
        
        com.stop();
        long dt = System.currentTimeMillis() - t0;
        
        //update robot pose
        robotLocation.translate(timeToDistance(dt));
    }
    
    /** drive the given distance but stop after encountering an obstacle <br>
     * @return whether an obstacle has been encountered
     */
    public boolean driveUntilObstacle(double distance_cm) {
    	int velocity = (int)(VELOCITY * Math.signum(distance_cm));
    	double currentDistance = 0;
    	boolean encounteredObstacle = false;
    	
        int[] s = new int[3];
        
        com.setVelocity(velocity, velocity);
        long t0 = System.currentTimeMillis();
        
        while(distance_cm > currentDistance) {
        	s = com.getSensors();
        	if (s[0] <= RANGE_THRESHOLD || s[1] <= RANGE_THRESHOLD || s[2] <= RANGE_THRESHOLD) {
        		encounteredObstacle = true;
        		break;
        	}
        	currentDistance = (timeToDistance(System.currentTimeMillis() - t0));
        }  
        
        com.stop();
        
        //correct small distance mistakes that may occurr
        if (!encounteredObstacle) {
        	drive(distance_cm - currentDistance);
        }
        
        long dt = System.currentTimeMillis() - t0;
        
        //update robot pose
        robotLocation.translate(timeToDistance(dt));
        
        return encounteredObstacle;
    }
    
    
    /**
     * The Roboter should change into following mode 
     * leave following mode if it hits the m-line (closer to goal than m_point) again
     * @param direction : 1 ->turn counter-clockwise, -1 -> turn clockwise <br>
     * 
     */
    public void followObstacle(int direction, SensorCondition leavingCondition) {
    	com.append("following obstacle");

    	int[] s_new = com.getSensors();
    	
    	//cancel if there is no obstacle to follow
    	if(s_new[0] == 255 && s_new[1] == 255 && s_new[2] == 255) {
    		return;
    	}

    	//else start circling around obstacle until leaving condition is fulfilled
		
    	int turnDirection = 2; //right sensor if turning left
    	if (direction < 0) {
    		turnDirection = 0; //left sensor if turning right
    	}
		
    	int s_old = s_new[turnDirection];
    	
		while(!leavingCondition.holds()) {
			
			//keep distance to wall
			while( (s_new[turnDirection] - s_old) < 21) {
				drive(s_new[turnDirection] - RANGE_THRESHOLD);
				if (leavingCondition.holds()) {
					return;
				}
				turn(3*direction);
				s_old = s_new[turnDirection];
				s_new = com.getSensors();
			}
			
			//wall ended, drive around corner
			Location temp = new Location(robotLocation);
			
			turn(20*direction);
			drive(s_old + ROBOT_LENGTH);
			if (leavingCondition.holds()) {
				return;
			}
			turnToLocation(temp);
			
			s_new = com.getSensors();
			s_old = s_new[turnDirection];
			
			while( (s_new[turnDirection] - s_old) < 21) {
				turn(3*direction);
				s_old = s_new[turnDirection];
				s_new = com.getSensors();
			}
			
			turn(20*direction);
			drive(s_old + ROBOT_LENGTH);
			if (leavingCondition.holds()) {
				return;
			}
			
			turnToLocation(temp);
			turn(3*direction);
		}
		
		//get new sensor values
		s_new = com.getSensors();
		s_old = s_new[turnDirection];
    }

}
