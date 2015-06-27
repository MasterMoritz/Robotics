package com.ebstor.robot;

import android.os.Bundle;
import android.util.Log;
import android.util.Pair;
import android.view.*;
import android.view.View.OnTouchListener;
import android.widget.EditText;

import com.ebstor.robot.beacons.Beacon;
import com.ebstor.robot.beacons.BeaconColor;
import com.ebstor.robot.beacons.BeaconContour;
import com.ebstor.robot.beacons.BeaconDetector;
import com.ebstor.robot.corefunctions.*;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.*;

import static java.lang.Thread.sleep;

public class ColorBlobDetectionActivity extends MainActivity implements OnTouchListener, CvCameraViewListener2 {

    private static final String  TAG              = "ColorBlobActivity";
    private static List<Scalar> BALLS_HSV = new  ArrayList<>();
    private static final Scalar  GREEN_BALL_RGBA = new Scalar(12,75,12,255);
    private static final Scalar  RED_BALL_HSV = new Scalar(360,100,60); // TODO make this a correct default value
    private static final Scalar  LOWEST_POINT_RGBA = new Scalar(34,200,1,255);
    private static final boolean testmode = false;
    private static Mat           homographyMatrix;
    private static Comparator<Point> pointComparator = new Comparator<Point>() {
        @Override
        public int compare(Point lhs, Point rhs) {
            return Double.compare(lhs.y, rhs.y);
        }
    };
    private volatile Mat         mRgba;
    private volatile Scalar      greenBallHsv;
    private volatile Scalar      redBallHsv;
    /** currently chosen blob color */
    private Scalar               mBlobColorHsv;
    private ColorBlobDetector    mDetector;
    private BeaconDetector       beaconDetector;
    private Mat                  mSpectrum;
    private Size                 SPECTRUM_SIZE;
    private Scalar               CONTOUR_COLOR;
    private Thread               procedureThread;
    private volatile boolean     stateMachineRunning;
    /**
     * egocentric coordinates of nearest ball, null if no ball detected
     */
    public Point nearestBallEgo = null;
    /**
     * everytime we get a new location for the ball, this is set to true
     * everytime we call ballDetected() it is set to false
     * then as we proceed to calculate where the ball is at the moment, we wait for it to be set true again
     * this way we can be quite sure that we have the correct location
     */
    private boolean ballLocationUpdated = false;
    
    /**
     * where the robot should take the ball
     */
    public Location target = null;

    private CameraBridgeViewBase mOpenCvCameraView;

    private BaseLoaderCallback  mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    mOpenCvCameraView.setOnTouchListener(ColorBlobDetectionActivity.this);
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public ColorBlobDetectionActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        super.onCreate(savedInstanceState);
        robot.connect();

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.color_blob_detection_surface_view);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);

        mOpenCvCameraView.setCvCameraViewListener(this);

        BeaconColor.hsvColors.put(BeaconColor.GREEN, new Scalar(75.75, 226.921875, 79.734375, 0.0));
        BeaconColor.hsvColors.put(BeaconColor.BLUE, new Scalar(148.671875, 219.453125, 107.859375, 0.0));
        BeaconColor.hsvColors.put(BeaconColor.RED, new Scalar(1.796875, 227.0, 190.46875, 0.0));
        BeaconColor.hsvColors.put(BeaconColor.YELLOW, new Scalar(33.9375, 254.5, 185.859375, 0.0));
        for (Beacon b : Beacon.values()) {
            b.egocentricCoordinates = new Point(Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY);
        }

    }

    @Override
    public void onPause()
    {
        super.onPause();
        robot.disconnect();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();

        stateMachineRunning = false;
    }

    @Override
    public void onResume()
    {
        super.onResume();
        robot.connect();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        beaconDetector = new BeaconDetector(mDetector);
        mSpectrum = new Mat();
        SPECTRUM_SIZE = new Size(200, 64);
        CONTOUR_COLOR = new Scalar(255,0,0,255);

        greenBallHsv = convertScalarRgba2Hsv(GREEN_BALL_RGBA);
        redBallHsv = RED_BALL_HSV;
    }

    public void onCameraViewStopped() {
        mRgba.release();
        stateMachineRunning = false;
    }


    /** do something every frame */
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
        if (homographyMatrix == null) {
            homographyMatrix = getHomographyMatrix(mRgba);
        } else {
            if (mBlobColorHsv != null) {
                mDetector.setHsvColor(mBlobColorHsv);
                mDetector.process(mRgba);
                Imgproc.drawContours(mRgba, mDetector.getContours(), -1, new Scalar(255, 255, 255, 255));
                if (!mDetector.getContours().isEmpty()) {
                    List<Point> points = mDetector.getContours().get(0).toList();
                    Log.v("homography test", imageCoordToEgoCoord(Collections.max(points, pointComparator)).toString());
                }
            }

        }
        return mRgba;
    }

    /**
     * find the nearest ball in the given color
     * @param ballColorHSV
     */
    public void findBall(Scalar ballColorHSV) {
    	mDetector.setHsvColor(ballColorHSV);
        mDetector.process(mRgba);
        
        List<MatOfPoint> contours = mDetector.getContours();
        Log.v(TAG, "ball contour count: " + contours.size());
        Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);

        Point ball = new Point(0,Double.NEGATIVE_INFINITY);

        boolean ballfound = false;
        for (MatOfPoint m : contours) {
        	if (isCircle(m)) {
                Point nearest = Collections.max(m.toList(),pointComparator);
                if (!inRange(imageCoordToEgoCoord(nearest))) continue;
                ballfound = true;
                if (pointComparator.compare(nearest,ball) > 0)
                    ball = nearest;
            }
        }

        if (ballfound) {
            nearestBallEgo = imageCoordToEgoCoord(ball);
            Log.v(TAG, "nearest ball coordinates: " + nearestBallEgo);
        } else {
            nearestBallEgo = null;
        }

        ballLocationUpdated = true;
    }
    public void findBalls() {
    	Point ball = new Point(0,Double.NEGATIVE_INFINITY);
    	boolean ballfound = false;
    	
    	for (Scalar ballColorHSV: BALLS_HSV) {
	    	mDetector.setHsvColor(ballColorHSV);
	        mDetector.process(mRgba);
	        
	        List<MatOfPoint> contours = mDetector.getContours();
	        Log.v(TAG, "ball contour count: " + contours.size());
	        Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);
	
	       
	
	        
	        for (MatOfPoint m : contours) {
	        	if (isCircle(m)) {
	                Point nearest = Collections.max(m.toList(),pointComparator);
	                if (!inRange(imageCoordToEgoCoord(nearest))) continue;
	                ballfound = true;
	                if (pointComparator.compare(nearest,ball) > 0)
	                    ball = nearest;
	            }
	        }
    	}

        if (ballfound) {
            nearestBallEgo = imageCoordToEgoCoord(ball);
            Log.v(TAG, "nearest ball coordinates: " + nearestBallEgo);
        } else {
            nearestBallEgo = null;
        }

        ballLocationUpdated = true;
    }

    /**
     *
     * @param egoPoint
     * @return if the distance to the point is less than 3m
     */
    private boolean inRange(Point egoPoint) {
        return (Math.sqrt(Math.pow(egoPoint.x,2) + Math.pow(egoPoint.y,2)) < 300);
    }

    public boolean isInBeacon(MatOfPoint contour) {
    	//get highest middlepoint of contour
    	BeaconContour acontour = new BeaconContour(contour);
    	Point[] apoints = acontour.get4Tuple();
    	Point highestMiddle = new Point( (Math.abs(apoints[2].x) - Math.abs(apoints[1].x))/2 + apoints[1].x, apoints[3].y);
    	
    	//compare with lower beacon colors
        List<MatOfPoint> contours;
        
        //search for red
    	mDetector.setHsvColor(BeaconColor.RED.hsvColor());
        mDetector.process(mRgba);
        contours = mDetector.getContours();
        Point[] colorpoints;
        
        for (MatOfPoint c : contours) {
        	colorpoints = (new BeaconContour(c)).get4Tuple();
        	
        	// if highestMiddle is between colorpoints then contour is in beacon
        	if (highestMiddle.x >= colorpoints[1].x && highestMiddle.x <= colorpoints[2].x && highestMiddle.y <= colorpoints[0].y) {
        		return true;
        	}
        }
    	return false;
    }


    public static boolean isCircle(MatOfPoint thisContour) {

        Rect ret = null;

        MatOfPoint2f thisContour2f = new MatOfPoint2f();
        MatOfPoint approx = new MatOfPoint();
        MatOfPoint2f approxContour2f = new MatOfPoint2f();

        thisContour.convertTo(thisContour2f, CvType.CV_32FC2);

        Imgproc.approxPolyDP(thisContour2f, approxContour2f, Imgproc.arcLength(thisContour2f, true)*0.02, true);

        approxContour2f.convertTo(approx, CvType.CV_32S);

        Log.v("isCircle", "number of vertices: " + approx.toList().size());
        if (approx.toList().size() >= 8) {
            Log.v("isCircle", "yes, it's a circle");
            return true;
        } else {
            Log.v("isCircle","nope, not a circle");
            return false;
        }

    }
    
    /**
     * execute the tasks corresponding to the current state, then go into the next state and repeat
     */
    private void stateMachine(State start) {
        State state = start; // starting state
        int ball_count = 4; // number of balls in the field
        Location ball = new Location();
        
        while(stateMachineRunning) {
            Log.v("STATE_MACHINE", "current state: " + state);
            switch (state) {
                //turn around until enough beacons are in view to localize the robot
                case LOCALIZE:
                    Log.v("STATE_MACHINE", "start LOCALIZE");
                    if (testmode) {
                        beaconDetector.process(mRgba);
                        relocate();

                    }
                    int z;
                    for (z = 0; z < 10; z++) {
	                	beaconDetector.process(mRgba);
	                	if (beaconDetector.getBeacons() != null) {
	                		Log.v("STATE_MACHINE", "relocate");
	                		relocate();
	                		break;
	                	}
	                	else {
	                		Log.v("STATE_MACHINE", "not enough beacons -> turn");
	                		robot.turn(36);
                            try {
                                Thread.sleep(500);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }
                    }
                    // couldn't find enough beacons to relocate
                    if (z == 10) {
                    	Log.v("STATE_MACHINE", "no beacons around me");
                        break;
                    }
                    
                    // only attempt to cage 1 ball for now before returning to goal
                    if (robot.balls_in_cage > 0) {
                    	state = State.BALL_TO_TARGET;
                    }
                    // search for a ball
                    else {
                    	state = State.SEARCH_BALL;
                    }
                    break;

                //not sure what to do with it
                case TRY_LOCALIZE:
                    for (int i = 0; i < 10; i++) {
	                	beaconDetector.process(mRgba);
	                	if (beaconDetector.getBeacons() != null) {
	                		relocate();
	                		break;
	                	}
	                	else {
	                		robot.turn(36);
                            try {
                                Thread.sleep(500);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }
                    }
                    // only attempt to cage 1 ball for now before returning to goal
                    if (robot.balls_in_cage > 0) {
                    	state = State.BALL_TO_TARGET;
                    }
                    // search for a ball
                    else {
                    	state = State.SEARCH_BALL;
                    }
                    break;

                //search environment for a ball
                case SEARCH_BALL:
                	Log.v("STATE_MACHINE", "SEARCH_BALL");
                    int i;
                    for (i = 0; i < 8; i++) {
                        findBalls();

                        //detected ball
                        if (ballDetected()) {
                            Log.v("STATE_MACHINE", "detected ball");
                            try {
                                sleep(100);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            findBalls();
                            //ball magically teleported away
                            if (!ballDetected()) {
                                Log.v("STATE_MACHINE", "lost sight of ball");
                                i -= 1;
                                continue;
                            }
                            //ball is still there
                            ball = new Location(nearestBallEgo.x, nearestBallEgo.y);
                            Log.v("STATE_MACHINE", "ball at: " + ball);
                            break;
                        }
                        //no ball in sight, turn 45 degrees and try again
                        else {
                            Log.v("STATE_MACHINE", "no ball detected, turning");
                            try {
                                sleep(100);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            robot.turn(45);
                        }
                    }
                    if (i == 8) {
                        //already made a 360 by now
                        Log.v("STATE_MACHINE", "no ball around me");
                        robot.turnToLocation(new Location(0,0));
                        robot.driveAndStopForObstacles(30);
                        break;
                    }
                    state = State.GOTO_BALL;
                    break;

                case GOTO_BALL:
                	Log.v("STATE_MACHINE", "GOTO_BALL");
                	//not sure how it is intended
                    robot.turn(Robot.degreesToBall(nearestBallEgo));
                    boolean obstacle = robot.driveAndStopForObstacles(Robot.euclideanDistance(new Location(), ball) - 30);
                    if (obstacle) {
                        robot.passObstacle();
                        state = State.SEARCH_BALL;
                    } else {
                        robot.drive(15);
                        state = State.CAGE_BALL;
                    }
                    break;

                case CAGE_BALL:
                    Log.v("STATE_MACHINE", "CAGE_BALL");
                    if (!robot.cageOpen)
                        robot.openCage();
                    robot.closeCage();
                    robot.balls_in_cage += 1;
                    state = State.TRY_LOCALIZE;
                    break;

                // drive robot to target and drop it there
                case BALL_TO_TARGET:
                	Log.v("STATE_MACHINE", "BALL_TO_TARGET");
                	robot.turnToGoal();
                	Log.v("STATE_MACHINE", "turned to goal");
	                obstacle = robot.driveAndStopForObstacles(Robot.euclideanDistance(robot.robotLocation, robot.goal) - 15);
                    if (obstacle) {
                        robot.passObstacle();
                    }

                    break;

                // drop all balls in cage and search for new balls if existent
                case DROP_BALL:
                	Log.v("STATE_MACHINE", "DROP_BALL");
                    robot.openCage();
                    ball_count -= robot.balls_in_cage;
                    robot.balls_in_cage = 0;

                    if (ball_count == 0) {
                        state = State.FIN;
                    } else {
                        state = State.SEARCH_BALL;
                    }
                    break;

                // robot finished task
                case FIN:
                	Log.v("STATE_MACHINE", "FIN");
                    return;

            }
        }
    }


    public void relocate() {
        final String tag = "Relocate";
        Pair<Beacon, Beacon> beacons = beaconDetector.getBeacons();
        if (beacons == null) {
            Log.i(tag, "Not enough beacons detected");
            return;
        }
        double d, x, y, dist_left, dist_right;
        Point leftBeacon = beacons.first.egocentricCoordinates;
        Point rightBeacon = beacons.second.egocentricCoordinates;
        // distance to left beacon
        //dist_left = leftBeacon.y/Math.sin(Math.atan(leftBeacon.y/leftBeacon.x));
        dist_left = Math.sqrt(Math.pow(leftBeacon.x, 2) + Math.pow(leftBeacon.y, 2));
        // distance to right beacon
        //dist_right = rightBeacon.y/Math.sin(Math.atan(rightBeacon.y/rightBeacon.x));
        dist_right = Math.sqrt(Math.pow(rightBeacon.x, 2) + Math.pow(rightBeacon.y, 2));
        Log.v(tag, "distance to left beacon: " + dist_left + " distance to right beacon: " + dist_right);
        d = 125.0;
        x = (Math.pow(d, 2) - Math.pow(dist_right, 2) + Math.pow(dist_left, 2)) / (2 * d);
        y = 1/d * Math.sqrt((-d + dist_right - dist_left)*(-d - dist_right + dist_left)*(-d + dist_right + dist_left)*(d + dist_right + dist_left));
		y /= 2;
        Log.v(tag, "x is " + x + " y is " + y);
        Beacon cornerBeacon;
        boolean isLeft = false;
        double robotX = 0.0;
        double robotY = 0.0;
        
        /* orientation */
        double robotTheta = 90.0 - Math.toDegrees(Math.atan(rightBeacon.x / (-rightBeacon.y)));
        Log.v("robot orientation","first theta: " + robotTheta);
        robotTheta += Math.toDegrees(Math.acos((Math.pow(125.0, 2) + Math.pow(dist_right, 2) - Math.pow(dist_left, 2)) / (2 * 125.0 * dist_right))); //law of cosines
        Log.v("robot orientation", "second theta: " + robotTheta);
        if (beacons.first.coordinates.x != 0 && beacons.first.coordinates.y != 0) {
            cornerBeacon = beacons.first;
            isLeft = true;
        } else {
            cornerBeacon = beacons.second;
        }
        Log.v(tag, "corner beacon is " + cornerBeacon + ", isLeft: " + isLeft);
        switch (cornerBeacon) {

        case RED_YELLOW:    //beacon in upper right
            if (isLeft) {
                robotX = cornerBeacon.coordinates.x - y;
                robotY = cornerBeacon.coordinates.y - x;
                robotTheta += 270.0;
            } else {
                robotX = x;
                robotY = cornerBeacon.coordinates.y - y;
            }
            break;
        case YELLOW_RED: //beacon in lower right
            if (isLeft) {
                robotX = cornerBeacon.coordinates.x - x;
                robotY = cornerBeacon.coordinates.y + y;
                robotTheta += 180.0;
            } else {
                robotX = cornerBeacon.coordinates.x - y;
                if(x > 0){
                	robotY = -x;
                }else{
                	robotY = Math.abs(x);
                }
                robotTheta += 270.0;
            }
            break;
        case YELLOW_BLUE: //beacon in lower left
            if (isLeft) {
                robotX = cornerBeacon.coordinates.x + y;
                robotY = cornerBeacon.coordinates.y + x;
                robotTheta += 90.0;
            } else {
                if(x > 0){
                	robotX = -x;
                }else{
                	robotX = Math.abs(x);
                }
                robotY = cornerBeacon.coordinates.y + y;
                robotTheta += 180.0;
            }
            break;
        case BLUE_YELLOW: //beacon in upper left
            if (isLeft) {
                robotX = cornerBeacon.coordinates.x + x;
                robotY = cornerBeacon.coordinates.y - y;
            } else {
                robotX = cornerBeacon.coordinates.x + y;
                robotY = x;
                robotTheta += 90.0;
            }
            break;
    }
        robot.robotLocation.setX(robotX);
        robot.robotLocation.setY(robotY);
        robot.robotLocation.setTheta(robotTheta);
        Log.v("robot orientation", "final orientation: " + robotTheta);
        Log.i(tag, "New Location: " + robot.robotLocation);
    }

    /**
     * when this method is called, ballLocationUpdated is set to false again
     * @return if the last frame contains a ball that can be targeted
     */
    public boolean ballDetected() {
        ballLocationUpdated = false;
        return nearestBallEgo != null;
    }
    
    /**
     * heads straight for the ball and cages it
     * with it's arm
     *
     * @param distanceToBall the distance from the middle of the axis to the ball
     */
    public void driveAndCageBall(double distanceToBall){
    	robot.drive(distanceToBall-15);
        robot.closeCage();
    }
    
    /**
     * The robot will turn 45 degrees 8 times (for 360 degrees in total) and check if
     * the ball is in sight. If so, the robot will head for it and cage it.
     * 
     * @return true when the ball is caged, false otherwise
     */
    public boolean turnLookCage(){
        for(int i = 0; i < 8; i++){
        	
        	//detected ball
            if(ballDetected()){
                Log.v(TAG,"ball detected");
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                System.out.println("Detected ball");
                // now wait until the ball location has been updated again
                long timeSinceDetected = 0;
                boolean ballIsLost = false;
                while(!ballLocationUpdated) {
                    try {
                        sleep(500);
                        timeSinceDetected += 500;
                        if (timeSinceDetected >= 3000) {
                            ballIsLost = true;
                            break;
                        }
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                if (ballIsLost) continue;
                Point targetEgo = new Point(nearestBallEgo.x,nearestBallEgo.y);
                // necessary so that turn and drive have the same target
                Log.i(TAG, "nearest point is = " + targetEgo.x + " | " + targetEgo.y);
                Log.v(TAG, "turning degrees: " + Robot.degreesToBall(targetEgo));
                robot.turn(Robot.degreesToBall(targetEgo));
                robot.turn(-7);
                driveAndCageBall(Robot.distanceToBall(targetEgo));
                return true;

            //no ball detected, so turn 45 deg and try again
    		} else {
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.turn(45);
            }
    	}
    	return false;
    }
    
    /**
     * after caging the ball the robot will take the ball to the target and
     * raise the bar again
     */
    public void ballToTarget(){
    	robot.turnToLocation(target);
    	robot.drive(Robot.euclideanDistance(robot.robotLocation, target) - 15);
    	robot.openCage();
    }
    
    /**
     * after the ball's release the robot returns to the starting point
     */
    public void returnToStart(){
    	Location start = new Location(0,0);
    	robot.turnToLocation(start);
    	robot.drive(Robot.euclideanDistance(start, robot.robotLocation));
        robot.turn(-robot.robotLocation.getTheta());
    }
    
    /**
     * algorithm for finding a green/red ball in the workspace
     */
    public void searchEnvironment(){

    	if(turnLookCage()){
    		return;
    	}
    	robot.drive(100);

        while(true) {
            if(turnLookCage()){
                return;
            }
            robot.turn(90);
            robot.drive(100);
        }
    }

    /**
     * whole procedure required to pass the second examination
     */
    public void secondExamination(){
    	robot.robotLocation = new Location(0,0);
        System.out.println("searching environment");
    	searchEnvironment();
    	System.out.println("get ball to target");
    	ballToTarget();
    	System.out.println("return to start");
    	returnToStart();
    }


    /**
     *
     * @param imgPoint the point on the image
     * @return the egocentric coordinates in cm, x heads to the front y heads to the left
     */
    public static Point imageCoordToEgoCoord(Point imgPoint) {
        //if (testmode) return imgPoint;
        if (homographyMatrix == null) throw new RuntimeException("we don't even have a homography matrix yet!");
        if (imgPoint == null) return null;
        Mat src =  new Mat(1, 1, CvType.CV_32FC2);
        Mat dest = new Mat(1, 1, CvType.CV_32FC2);
        src.put(0, 0, imgPoint.x, imgPoint.y);
        Core.perspectiveTransform(src, dest, homographyMatrix);
        Point dest_point = new Point(dest.get(0, 0)[0]/10, dest.get(0, 0)[1]/10);
        Log.v(TAG, "coordinates: " + dest_point.x + ", " + dest_point.y);
        return dest_point;
    }

    public static Mat getHomographyMatrix(Mat mRgba) {
    	  final Size mPatternSize = new Size(6, 9); // number of inner corners in the used chessboard pattern 
          float x = 380.0f;
          float y = 105; // coordinates of first detected inner corner on chessboard
    	  float delta = 25.0f; // size of a single square edge in chessboard
    	  //y += 8*delta;
    	  LinkedList<Point> PointList = new LinkedList<>();

    	  // Define real-world coordinates for given chessboard pattern:
    	  for (int i = 0; i < mPatternSize.height; i++) {
    	    x = 382.0f;
    	    for (int j = 0; j < mPatternSize.width; j++) {
    	      PointList.addLast(new Point(x,y));
    	      x += delta;
    	    }
    	    y -= delta;
    	  }
    	  MatOfPoint2f RealWorldC = new MatOfPoint2f();
    	  RealWorldC.fromList(PointList);
    	 
    	  // Detect inner corners of chessboard pattern from image:
    	  Mat gray = new Mat();
    	  Imgproc.cvtColor(mRgba, gray, Imgproc.COLOR_RGBA2GRAY); // convert image to grayscale
    	  MatOfPoint2f mCorners = new MatOfPoint2f();
    	  boolean mPatternWasFound = Calib3d.findChessboardCorners(gray, mPatternSize, mCorners);
    	 
    	  // Calculate homography:
    	  if (mPatternWasFound){
    	    Calib3d.drawChessboardCorners(mRgba, mPatternSize, mCorners, mPatternWasFound); //for visualization
    	    Log.i(TAG,"homography was found");
            return Calib3d.findHomography(mCorners, RealWorldC);
    	  }else{
    	    return null;
    	  }
    	}


    /**
     * @return the point that is the nearest to the robot or null if both p1 and p2 are null
     */
    private Point getLowestPoint(Point p1, Point p2) {
        if (p1 == null)
            if (p2 == null)
                return null;
            else
                return p2;
        else if (p2 == null)
            return p1;
        else if (pointComparator.compare(p1,p2) <= 0)
            return p2;
        else return p1;
    }

    public boolean onTouch(View v, MotionEvent event) {
        int cols = mRgba.cols();
        int rows = mRgba.rows();

        int xOffset = (mOpenCvCameraView.getWidth() - cols) / 2;
        int yOffset = (mOpenCvCameraView.getHeight() - rows) / 2;

        int x = (int)event.getX() - xOffset;
        int y = (int)event.getY() - yOffset;

        Log.i(TAG, "Touch image coordinates: (" + x + ", " + y + ")");

        if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;

        Rect touchedRect = new Rect();

        touchedRect.x = (x>4) ? x-4 : 0;
        touchedRect.y = (y>4) ? y-4 : 0;

        touchedRect.width = (x+4 < cols) ? x + 4 - touchedRect.x : cols - touchedRect.x;
        touchedRect.height = (y+4 < rows) ? y + 4 - touchedRect.y : rows - touchedRect.y;

        Mat touchedRegionRgba = mRgba.submat(touchedRect);

        Mat touchedRegionHsv = new Mat();
        Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

        // Calculate average color of touched region
        mBlobColorHsv = Core.sumElems(touchedRegionHsv);
        int pointCount = touchedRect.width*touchedRect.height;
        for (int i = 0; i < mBlobColorHsv.val.length; i++)
            mBlobColorHsv.val[i] /= pointCount;

        /*mBlobColorRgba = convertScalarHsv2Rgba(mBlobColorHsv);

        Log.i(TAG, "Touched rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
                ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");*/


        //Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

        touchedRegionRgba.release();
        touchedRegionHsv.release();

        return false; // don't need subsequent touch events
    }

    /** adds the last touched color to the calibrated balls */
    public void calibrateAddBall(MenuItem item) {
        if (mBlobColorHsv != null) {
            BALLS_HSV.add(mBlobColorHsv);
            mBlobColorHsv = null;
        }
    }

    /** resets the ball calibrations */
    public void calibrateClearBalls(MenuItem item) {
        if (mBlobColorHsv != null) {
            BALLS_HSV.clear();
            mBlobColorHsv = null;
        }
    }

    public void calibrateRedBeacon(MenuItem item) {
        if (mBlobColorHsv != null) {
            BeaconColor.RED.setHsvColor(mBlobColorHsv);
            Log.i("colorcalibration", "red beacon: " + mBlobColorHsv);
            mBlobColorHsv = null;
        }
    }

    public void calibrateBlueBeacon(MenuItem item) {
        if (mBlobColorHsv != null) {
            BeaconColor.BLUE.setHsvColor(mBlobColorHsv);
            Log.i("colorcalibration", "blue beacon: " + mBlobColorHsv);
            mBlobColorHsv = null;
        }
    }

   /* public void calibratePurpleBeacon(MenuItem item) {
        if (mBlobColorHsv != null) {
            BeaconColor.GREEN.setHsvColor(mBlobColorHsv);
            Log.i("colorcalibration", "purple beacon: " + mBlobColorHsv);
            mBlobColorHsv = null;
        }
    }*/

    public void calibrateGreenBeacon(MenuItem item) {
        if (mBlobColorHsv != null) {
            BeaconColor.GREEN.setHsvColor(mBlobColorHsv);
            Log.i("colorcalibration", "green beacon: " + mBlobColorHsv);
            mBlobColorHsv = null;
        }
    }

    public void calibrateYellowBeacon(MenuItem item) {
        if (mBlobColorHsv != null) {
            BeaconColor.YELLOW.setHsvColor(mBlobColorHsv);
            Log.i("colorcalibration", "yellow beacon: " + mBlobColorHsv);
            mBlobColorHsv = null;
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.camera_menu, menu);
        return true;
    }

    private Scalar convertScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);
        return new Scalar(pointMatRgba.get(0, 0));
    }

    private Scalar convertScalarRgba2Hsv(Scalar rgbaColor) {
        Mat pointMatHsv = new Mat();
        Mat pointMatRgba = new Mat(1,1,CvType.CV_8UC3,rgbaColor);
        Imgproc.cvtColor(pointMatRgba, pointMatHsv, Imgproc.COLOR_RGB2HSV);
        return new Scalar(pointMatHsv.get(0,0));
    }

    public void startExam2(View view) {
        EditText x = (EditText) findViewById(R.id.target_x);
        EditText y = (EditText) findViewById(R.id.target_y);

        String sx = x.getText().toString();
        String sy = y.getText().toString();

        Double dx;
        Double dy;


        if (x.getText().toString().equals("x") || y.getText().toString().equals("y")) {
            Log.e(TAG,"no target specified");
            return;
        }
        System.out.println("parsing x and y of " + sx + " " + sy);
        // x20 should be parsed same as only 20
        try {
        	dx = Double.parseDouble(sx);
        } catch(Exception e) {
        	dx = Double.parseDouble(sx.substring(1));
        }

        // y20 should be parsed same as only 20
        try {
        	dy = Double.parseDouble(sy);
        } catch(Exception e) {
        	dy = Double.parseDouble(sy.substring(1));
        }

        target = new Location(dx,dy);

        procedureThread = new Thread() {
            @Override
            public void run() {
                secondExamination();
            }
        };
        procedureThread.start();
    }

    public void stopWorkerThread(View view) {
        stateMachineRunning = false;
        robot.stop();
    }

    public void startExam3(View view) {
        Log.v(TAG,"startExam2 called");
        EditText x = (EditText) findViewById(R.id.target_x);
        EditText y = (EditText) findViewById(R.id.target_y);

        String sx = x.getText().toString();
        String sy = y.getText().toString();

        Double dx;
        Double dy;


        if (x.getText().toString().equals("x") || y.getText().toString().equals("y")) {
            Log.e(TAG,"no target specified");
            return;
        }
        System.out.println("parsing x and y of " + sx + " " + sy);
        // x20 should be parsed same as only 20
        try {
            dx = Double.parseDouble(sx);
        } catch(Exception e) {
            dx = Double.parseDouble(sx.substring(1));
        }

        // y20 should be parsed same as only 20
        try {
            dy = Double.parseDouble(sy);
        } catch(Exception e) {
            dy = Double.parseDouble(sy.substring(1));
        }

        robot.goal = new Location(dx,dy);


        if (stateMachineRunning) return;
        stateMachineRunning = true;
        new Thread(){
            @Override
            public void run() {
                while(homographyMatrix == null) { // don't need this
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                stateMachine(com.ebstor.robot.State.LOCALIZE);
                stateMachineRunning = false;
            }
        }.start();
    }

    public void test(View view) {
        startExam3(view);
        /*beaconDetector.process(mRgba);
        relocate();
        if (mBlobColorHsv != null) {
            mDetector.setHsvColor(mBlobColorHsv);
            mDetector.process(mRgba);
            isCircle(mDetector.getContours().get(0));
        }*/
    }
}
