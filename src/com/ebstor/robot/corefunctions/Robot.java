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
    public static double DEGREE_PER_MILLISECOND = 0.08;
    
    /** cm travelled per millisecond for velocity */
    public static double CM_PER_MILLISECOND = 0.0136;
    
    /** considering the 50ms wait buffer this is the minimum angle that has to be turned */
    private static int MINIMUM_TURN = 4;
    
    /** considering the 50ms wait buffer this is the minimum distance that has to be driven */
    private static int MINIMUM_DRIVE = 1;
    
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
    private static final int RANGE_THRESHOLD = 15;

    /** the threshold in cm at which the robot may start driving again */
    private static final int SOFT_THRESHOLD = 30;
    
    /** the angle between the left end of the LOS of the front sensor and the side sensor */
    private static final double ANGLE_FRONT_SIDE = 10;
    private static final double ANGLE_RIGHT_LEFT = 40;
    
    
    
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
    	
        com.setVelocity((byte)VELOCITY, (byte)VELOCITY);
        
        long t0 = System.currentTimeMillis();
        while(!condition.holds()) {}
        
        com.stop();
        
        long dt = System.currentTimeMillis() - t0;
        
        //update robot pose
        robotLocation.translate(timeToDistance(dt));
    }

    /**
     * drives until a certain sensor condition is met or distance_cm
     * @return whether condition was fulfilled or not
     */
    public boolean driveUntil(double distance_cm, SensorCondition condition) {
    	int velocity = VELOCITY * (int)(Math.signum(distance_cm));
        boolean reached = false;
        double currentDistance = 0;
        
        long t0 = System.currentTimeMillis() - 50;
        drive();
    	
        while (distance_cm > currentDistance && !condition.holds()) {
        	currentDistance = timeToDistance(System.currentTimeMillis() - t0);
        }
        
        com.stop();
        long dt = System.currentTimeMillis() - t0;
        currentDistance = timeToDistance(dt - t0);
        robotLocation.translate(dt*Integer.signum(velocity));
        
        sleep_h(100);
        //drive back if driven too much cause of overheads
        if(currentDistance > distance_cm) {
        	double diff = distance_cm - currentDistance;
        	if (diff < MINIMUM_DRIVE) {
        		drive(-MINIMUM_DRIVE);
        	}
        	else {
        		drive(distance_cm - currentDistance);
        	}
        }
        /*
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
        */
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
    	turnLeft();
    	long t0 = System.currentTimeMillis();
        while (!condition.holds()) {}
        
        com.stop();
        long dt = System.currentTimeMillis() - t0;
        robotLocation.rotate(timeToDegrees(dt));
    }
    public void turnLeftUntil(SensorCondition condition, double degree) {
    	double currentDegree = 0;
    	turnLeft();
    	long t0 = System.currentTimeMillis();
    	
        while (!condition.holds() || currentDegree < degree) {
        	currentDegree += (timeToDegrees(System.currentTimeMillis() - t0));
        }
        
        com.stop();
        long dt = System.currentTimeMillis() - t0;
        robotLocation.rotate(timeToDegrees(dt));
    }

    /**
     * turns right until a certain sensor condition is met (e.g. obstacle parallel to object)
     */
    public void turnRightUntil(SensorCondition condition)  {
    	turnRight();
    	long t0 = System.currentTimeMillis();
        while (!condition.holds()) {}
        
        com.stop();
        long dt = System.currentTimeMillis() - t0;
        robotLocation.rotate(-timeToDegrees(dt));
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
            com.append(angle + " | " + turningAngle + " | " + robotLocation.getTheta());
    	} catch(Exception e) {
    		com.append("failed to turn towards location");
            com.append(e.toString());
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
        SensorCondition isObstacle = new SensorCondition(this) {
            @Override
            public boolean holds() {
                int[] sensors = robot.com.getSensors();
                int left = sensors[0];
                int center = sensors[1];
                int right = sensors[2];

                return (Math.min(left,Math.min(center,right)) <= RANGE_THRESHOLD);
            }
        };
        driveUntil(dist, isObstacle);
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
    		com.setVelocity((int)(Math.random() * 128), (int)(Math.random() * 128));
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
    private void keepDistance(int direction) {
    	com.append("keeping distance " + Integer.toString(direction));
		int[] sensor = com.getSensors();
		
		int turnDirection = 2; //right sensor if turning left
		int cturnDirection = 0;
    	if (direction < 0) {
    		turnDirection = 0; //left sensor if turning right
    		cturnDirection = 2;
    	}
    	
    	SensorCondition endOfObstacle = new SensorCondition(this) {
			public int[] s_new = com.getSensors();
			public int s_old;
			
			@Override
			public boolean holds() {
				s_old = s_new[0];
				s_new = robot.com.getSensors();
				if (s_new[0] - s_old >= 20) {
					return true;
				}
				return false;
			}

			@Override
			public void reset() {
				s_new = com.getSensors();
				s_old = s_new[0];
			}

			@Override
			public int getInt() {
				return s_old;
			}
		};
		
		SensorCondition findObstacle = new SensorCondition(this) {
			public int[] s_new = robot.com.getSensors();
			
			private int nearestPoint;
			private double theta;
			private double rtheta;
			private long t0;
			
			@Override
			public boolean holds() {
				s_new = robot.com.getSensors();
				if (s_new[0] < nearestPoint) {
					nearestPoint = s_new[0];
					theta = timeToDegrees(System.currentTimeMillis() - t0);
				}
				rtheta = timeToDegrees(System.currentTimeMillis() - t0);
				if (rtheta > 90 ) {
					return true;
				}
				return false;
			}
			
			@Override
			public void reset() {
				nearestPoint = 255;
				t0 = System.currentTimeMillis();
			};

			@Override
			public double getDouble() {
				return (rtheta - theta);
			}
			
			@Override
			public int getInt() {
				return nearestPoint;
			}
		};
    	
		
		while(true) {
			sleep_h(100);
			
			
			endOfObstacle.reset();
			this.turnRightUntil(endOfObstacle);
			int distance = endOfObstacle.getInt();
			System.out.println(distance - RANGE_THRESHOLD);
			sensor = com.getSensors();
			System.out.println(sensor[cturnDirection]);
			
			if (sensor[cturnDirection] <= (distance)) {
				drive(8);
			}
			else {
				drive(distance);
			}
		
			findObstacle.reset();
			this.turnLeftUntil(findObstacle);
			turn(direction * (findObstacle.getDouble()));
			drive(findObstacle.getInt() - RANGE_THRESHOLD);
		
		}
		
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
     * note that direction should always be -1 for now (because of some lazy hardcodings)
     * 
     */
    public void followObstacle(int direction, SensorCondition leavingCondition) {
    	com.append("following obstacle");

    	int[] sensor = com.getSensors();
    	
    	//cancel if there is no obstacle to follow
    	if(sensor[0] == 255 && sensor[1] == 255 && sensor[2] == 255) {
    		return;
    	}
    	
    	int turnDirection = 2; //right sensor if turning left
		int cturnDirection = 0;
    	if (direction < 0) {
    		turnDirection = 0; //left sensor if turning right
    		cturnDirection = 2;
    	}
    	
    	drive(sensor[0] - RANGE_THRESHOLD);

    	//Keep Distance RANGE_THRESHOLD to obstacle while circling around it
    	SensorCondition endOfObstacle = new SensorCondition(this) {
			public int[] s_new = com.getSensors();
			public int s_old;
			
			@Override
			public boolean holds() {
				s_old = s_new[0];
				s_new = robot.com.getSensors();
				if (s_new[0] - s_old >= 20) {
					return true;
				}
				return false;
			}

			@Override
			public void reset() {
				s_new = com.getSensors();
				s_old = s_new[0];
			}

			@Override
			public int getInt() {
				return s_old;
			}
		};
		
		SensorCondition findObstacle = new SensorCondition(this) {
			public int[] s_new = robot.com.getSensors();
			
			private int nearestPoint;
			private double theta;
			private double rtheta;
			private long t0;
			
			@Override
			public boolean holds() {
				s_new = robot.com.getSensors();
				if (s_new[0] < nearestPoint) {
					nearestPoint = s_new[0];
					theta = timeToDegrees(System.currentTimeMillis() - t0);
				}
				rtheta = timeToDegrees(System.currentTimeMillis() - t0);
				if (rtheta > 90 ) {
					return true;
				}
				return false;
			}
			
			@Override
			public void reset() {
				nearestPoint = 255;
				t0 = System.currentTimeMillis();
			};

			@Override
			public double getDouble() {
				return (rtheta - theta);
			}
			
			@Override
			public int getInt() {
				return nearestPoint;
			}
		};
    	
		
		while(!leavingCondition.holds()) {
			sleep_h(100);
						
			endOfObstacle.reset();
			this.turnRightUntil(endOfObstacle);
			int distance = endOfObstacle.getInt();
			
			sensor = com.getSensors();
			if (sensor[cturnDirection] <= (distance)) {
				drive(8);
			}
			else {
				drive(distance);
			}
		
			findObstacle.reset();
			this.turnLeftUntil(findObstacle);
			turn(direction * (findObstacle.getDouble()));
			drive(findObstacle.getInt() - RANGE_THRESHOLD);
		
		}
    }

}
