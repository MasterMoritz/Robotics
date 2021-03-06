 package com.ebstor.robot.corefunctions;

import android.util.Log;

import org.opencv.core.*;

import com.ebstor.robot.communication.Communicator;

import jp.ksksue.driver.serial.FTDriver;

/**
 * this class contains all abilities the robot has
 */
public class Robot {
	
	private static String TAG = "Robot";

    /** speed of robot */
	public static int VELOCITY = 15;
	
    /** degrees turned per millisecond for velocity */
    public static double DEGREE_PER_MILLISECOND = 0.08242;
    
    /** cm travelled per millisecond for velocity */
    public static double CM_PER_MILLISECOND = 0.0136;
    
    /** considering the 50ms wait buffer this is the minimum angle that has to be turned <br>
     * is set in constructor */
    private static double MINIMUM_TURN ;
    
    /** considering the 50ms wait buffer this is the minimum distance that has to be driven <br>
     * is set in constructor */
    private static double MINIMUM_DRIVE;

    /** the circumference in cm from within the robot assumes he has reached the goal*/
    private static final int CIRCUMFERENCE_GOAL = 20;
	
    /** tolerance for m-line hitting in cm */
    private static final int CIRCUMFERENCE_MLINE = 5;
	
    /** the threshold in cm where the robot starts avoiding an obstacle */
    private static final int RANGE_THRESHOLD = 19;
    
    /** number of balls already caged */
    public int balls_in_cage = 0;
   
    /** if cage is currently lowered or not */
    public boolean cageOpen = true;
    
    /** holds the pose of the robot */
    public Location robotLocation;
    
    /** holds the desired pose of the robot after reaching its destination */
    public Location goal;
    
    /** the last point on the m-line that the robot hit */
    public Location m_point;
    
    /** serial communicator */
    public Communicator com;

    public SensorCondition isObstacle = new SensorCondition(this) {
        @Override
        public boolean holds() {
            int[] sensors = robot.com.getSensors();
            int left = sensors[0];
            int center = sensors[1];
            int right = sensors[2];

            return (Math.min(left,Math.min(center,right)) <= RANGE_THRESHOLD);
        }
    };
    
    /** setup serial communicator and initialize locations */
    public Robot(FTDriver driver) {
        robotLocation = new Location(0, 0, 0);
        goal = new Location(0, 0, 0);
        this.com = new Communicator(driver);
        
        MINIMUM_TURN = DEGREE_PER_MILLISECOND * Communicator.WAIT_BUFFER;
        MINIMUM_DRIVE = CM_PER_MILLISECOND * Communicator.WAIT_BUFFER;
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
        if (!com.isConnected()) com.connect();
    }
    public void disconnect() {
        if (com.isConnected()) com.disconnect();
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
        long time = distanceToTime(distance_cm) - Communicator.WAIT_BUFFER;
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
        
        drive();
        long t0 = System.currentTimeMillis();

        while (distance_cm > currentDistance - timeToDistance(Communicator.WAIT_BUFFER)) {
        	reached = condition.holds();
            if (reached) break;
            currentDistance = timeToDistance(System.currentTimeMillis() - t0);
        }
        
        com.stop();

        long dt = System.currentTimeMillis() - t0;
        robotLocation.translate(Robot.timeToDistance(dt) * Integer.signum(velocity));

        return reached;
    }

    public void turn(double degree) {
    	if (Math.abs(degree) < MINIMUM_TURN) {
    		Log.i("Robot", "can't turn " + degree + " because it is too small");
    		return;
    	}
    	
    	int velocity = (int)(Math.signum(degree))*VELOCITY;
    	
    	Log.i("Robot", "Turn " + degree + " degrees");
        for (int i = 0; i < Math.abs((int)(degree)/45); i++) {
            if (degree != 0) {
                long time = degreesToTime(45) - Communicator.WAIT_BUFFER;
                if (time < 0) {
                    time = 0;
                }
                com.setVelocity((byte)-velocity,(byte)velocity);
                sleep_h(time);
                com.stop();
                robotLocation.rotate(Math.signum(degree) * 45);
            }
        }
        
        double remainder = degree % 45;
        if (Math.abs(remainder) > MINIMUM_TURN) {
            long time = degreesToTime(remainder) - Communicator.WAIT_BUFFER;
            if (time < 0) {
                time = 0;
            }
            com.setVelocity((byte)-velocity,(byte)velocity);
            sleep_h(time);
            com.stop();
            robotLocation.rotate(Math.signum(degree)*remainder);
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

    public void turnToGoal() {
    	try {
	        double angle = Math.toDegrees(Math.atan2(goal.getY()-robotLocation.getY(),goal.getX()-robotLocation.getX()));
            double turningAngle =  angle - robotLocation.getTheta();
            if (turningAngle > 180) turningAngle = 360 - turningAngle;
            if (turningAngle < -180) turningAngle = 360 + turningAngle;
            turn(turningAngle);
    	} catch(Exception e) {
    		Log.e(TAG,"failed to turn towards goal");
    	}
    }

    public void turnToLocation(Location point) {
    	try {
	        double angle = Math.toDegrees(Math.atan2(point.getY()-robotLocation.getY(),point.getX()-robotLocation.getX()));
            double turningAngle =  angle - robotLocation.getTheta();
            if (turningAngle > 180) turningAngle = 360 - turningAngle;
            if (turningAngle < -180) turningAngle = 360 + turningAngle;
            turn(turningAngle);
            Log.v(TAG, angle + " | " + turningAngle + " | " + robotLocation.getTheta());
    	} catch(Exception e) {
    		Log.e(TAG,"failed to turn towards location");
    	}
    }

    /**
     *
     * @param egoBall egocentric ball coordinates
     * @return degrees to turn to the robert
     */
    public static double degreesToBall(Point egoBall){
        double turningAngle = Math.toDegrees(Math.atan2(egoBall.y,egoBall.x));
        if (turningAngle > 180) turningAngle = 360 - turningAngle;
        if (turningAngle < -180) turningAngle = 360 + turningAngle;
        Log.v("Robot", "turning angle to ball: " + turningAngle);
        return turningAngle;
    }

    /**
     *
     * @param egoBall egocentric ball coordinates
     * @return distance in cm to the ball
     */
    public static double distanceToBall(Point egoBall) {
        return Math.sqrt(Math.pow(egoBall.x,2) + Math.pow(egoBall.y,2));
    }
    
    public static double euclideanDistance(Location a, Location b) {
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

    public boolean driveAndStopForObstacles(double dist) {
        return driveUntil(dist, isObstacle);
    }

    public void setGoal(Location goal) {
        this.goal = goal;
    }

    /**
     * @param l
     * @return the angle[deg] from a given location to the origin
     */
    public double getAngle(Location l) {
    	return Math.toDegrees(Math.atan(l.getY()/l.getX()));
    }
    public double getAngleRad(Location l) {
    	return Math.atan(l.getY() / l.getX());
    }
    
    public boolean mlineEncountered() {
        return distanceToMline() <= 3; // (cm) this depends on how far robot travels between checks, can therefore be reduced
    }

	/** alternative to mlineEncountered */
	public boolean encounteredMline() {		
		int rho = (int)(getAngle(robotLocation));
		int gho = (int)(getAngle(goal));
		
		return (Math.abs(rho - gho) <= CIRCUMFERENCE_MLINE);
	}
	public boolean encounteredMline(int tolerance) {
		int rho = (int)(getAngle(robotLocation));
		int gho = (int)(getAngle(goal));
		
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

    	while(m < 6) {
    		com.setVelocity((int)(Math.random() * 90), (int)(Math.random() * 90));
    		sleep_h((long) (Math.random() * 500 + 200));
    		if (b > (int)(Math.random()*100)) {
    			m += 1;
    		}
    	}
    	
    	stop();
    }

    public void openCage() {
        com.setBarUp();
        // just to be sure :P
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        cageOpen = true;
    }

    public void closeCage() {
        com.setBarDown();
        // just to be sure :P
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        cageOpen = false;
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
        long dt = System.currentTimeMillis() - t0;
        
        //update robot pose
        robotLocation.translate(timeToDistance(dt));
        
        //correct small distance mistakes that may occurr
        if (!encounteredObstacle && (Math.abs(distance_cm - currentDistance) >= MINIMUM_DRIVE)) {
        	drive(distance_cm - currentDistance);
        }
        
        return encounteredObstacle;
    }
    
    public void passObstacle() {
        try {
            Thread.sleep(3000);
            if (!isObstacle.holds()) return;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        do {
            turn(-90);
            driveAndStopForObstacles(30);
            turn(90);
        } while (isObstacle.holds());

    }
    
    /**
     * The Roboter should change into following mode 
     * leave following mode if it hits the m-line (closer to goal than m_point) again
     * @param direction : 1 ->turn counter-clockwise, -1 -> turn clockwise <br>
     * 
     * note that direction should always be -1 for now (because of some lazy hardcodings) <br>
     * note that a leavingCondition can't be used until driveUntil is as accurate as drive
     */
    public void followObstacle(int direction) {
    	Log.i(TAG, "following obstacle");

    	int[] sensor = com.getSensors();
    	Location temp;
    	
    	//cancel if there is no obstacle to follow
    	if(sensor[0] == 255 && sensor[1] == 255 && sensor[2] == 255) {
    		return;
    	}
    	//put robot into a useful transformation
    	int right = sensor[2];
    	while ((sensor[0] > 50 && right < 50) && Math.abs(sensor[0] - right) >= 20) {
    		turn(-10);
    		sensor = com.getSensors();
    	}

    	//not used yet, as turn direction is hardcoded
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
    	
		//can't use a sensorcondition until driveUntil is as accurate as drive
		double alpha;
		double hyp;
		while(true) {
			sleep_h(100);
						
			endOfObstacle.reset();
			this.turnRightUntil(endOfObstacle);
			int distance = 12;
			drive(distance);

			//BEGIN check for mline that may have been encountered during driving distance
			//unfortunately driveUntil causes too much inaccuracy and can't be used yet
			
			Log.i(TAG, "robotlocation: " + (robotLocation));
			Log.i(TAG, "goallocation: " + goal);
			temp = new Location(robotLocation); //save current robotlocation
			alpha = getAngleRad(robotLocation);
			hyp = Math.abs(robotLocation.getY()/Math.sin(alpha));
			//AK = x
			//GK = y
			for(int i = 1; i <= Math.abs(distance); i ++) {
				hyp -= 1;
				robotLocation.setX(Math.cos(alpha) * hyp);
				robotLocation.setY(Math.sin(alpha) * hyp);
				
				if (encounteredMline(15) && euclideanDistance(robotLocation, goal) < euclideanDistance(m_point, goal)) {
					Log.v(TAG, "encountered mline on: " + robotLocation);
					robotLocation.setX(temp.getX());
					robotLocation.setY(temp.getY());
					
					drive(-i);
					m_point = new Location(robotLocation);
					return;
				}
			}
			robotLocation.setX(temp.getX());
			robotLocation.setY(temp.getY());

			//END check for mline

			findObstacle.reset();
			this.turnLeftUntil(findObstacle);
			turn(direction * (findObstacle.getDouble()));
			drive(findObstacle.getInt() - RANGE_THRESHOLD);
		
		}
    }
}
