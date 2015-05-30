package com.ebstor.robot;

import android.os.Bundle;
import android.util.Log;
import android.util.Pair;
import android.view.*;
import android.view.View.OnTouchListener;
import android.widget.EditText;
import com.ebstor.robot.beacons.Beacon;
import com.ebstor.robot.beacons.BeaconColor;
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
    private static final Scalar  GREEN_BALL_RGBA = new Scalar(12,75,12,255);
    private static final Scalar  RED_BALL_HSV = new Scalar(360,100,60); // TODO make this a correct default value
    private static final Scalar  LOWEST_POINT_RGBA = new Scalar(34,200,1,255);
    private static final boolean testmode = true;
    private static Mat           homographyMatrix;
    private static Comparator<Point> pointComparator = new Comparator<Point>() {
        @Override
        public int compare(Point lhs, Point rhs) {
            return Double.compare(lhs.y, rhs.y);
        }
    };
    private Mat                  mRgba;
    private Scalar               greenBallHsv;
    private Scalar               redBallHsv;
    /** currently chosen blob color */
    private Scalar               mBlobColorHsv;
    private ColorBlobDetector    mDetector;
    private BeaconDetector beaconDetector;
    private Mat                  mSpectrum;
    private Size                 SPECTRUM_SIZE;
    private Scalar               CONTOUR_COLOR;
    private Thread               procedureThread;
    /**
     * egocentric coordinates of nearest ball, null if no ball detected
     * this is updated by onCameraFrame ->
     * every frame that does not contain any blobs results in this variable being null
     */
    public Point nearestBall = null;
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
        //robot = new Robot(new FTDriver((UsbManager) getSystemService(USB_SERVICE)));
        robot.connect();

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.color_blob_detection_surface_view);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);

        mOpenCvCameraView.setCvCameraViewListener(this);

    }

    @Override
    public void onPause()
    {
        super.onPause();
        robot.disconnect();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
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
    }


    /** do something every frame */
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
        if (homographyMatrix == null) {
            homographyMatrix = getHomographyMatrix(mRgba);
        } else {
        	
        }

        return mRgba;
    }

    /**
     * find the nearest ball in the given color
     * @param ballColorHSV
     */
    public void findBall (Scalar ballColorHSV) {
    	mDetector.setHsvColor(ballColorHSV);
        mDetector.process(mRgba);
        
        List<MatOfPoint> contours = mDetector.getContours();
        List<Point> points = new ArrayList<>();
        
        Log.v(TAG, "ball contour count: " + contours.size());
        Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);
        for (MatOfPoint m : contours)
            points.addAll(m.toList());

        if (!points.isEmpty()) {
            nearestBall = imageCoordToEgoCoord(Collections.max(points, pointComparator));
        }
        else {
        	nearestBall = null;
        }
        
        ballLocationUpdated = true;
    }
    
    /**
     * execute the tasks corresponding to the current state, then go into the next state and repeat
     */
    private void stateMachine() {
        State state = State.LOCALIZE; // starting state
        int ball_count = 10; // number of balls in the field
        Location ball = new Location();
        
        for(;;)
            switch (state) {
            
            	//turn around until enough beacons are in view to localize the robot
                case LOCALIZE:
                	int z = 0;
                    for (z = 0; z < 8; z++) {
	                	beaconDetector.process(mRgba);
	                	if (beaconDetector.getBeacons() != null) {
	                		relocate();
	                		break;
	                	}
	                	else {
	                		robot.turn(45);
	                	}
                    }
                    // couldn't find enough beacons to relocate
                    if (z == 8) {
                    	//hope that robot finds enough beacons next time, because we can't drive around without knowing our location
                    	continue;
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
                    // TODO look around etc
                    // if found relocate()
                    state = State.SEARCH_BALL;
                    break;
                    
                //search environment for a ball
                // TODO: TODO and eliminate unneccesary sleeps after confirming working algorithm because we gotta be fast
                case SEARCH_BALL:
                	
                    for(int i = 0; i < 8; i++){
                    	 findBall(greenBallHsv);
                    	    	 
                    	//detected ball
                        if(ballDetected()){
                        	Log.v(TAG, "detected ball");
                        	try {
                        		sleep(100);
   		                 	} catch (InterruptedException e) {
   		                 		e.printStackTrace();
   		                 	}
                        	findBall(greenBallHsv);
                        	//ball magically teleported away
                            if (!ballDetected()) {
                            	Log.v(TAG, "lost sight of ball");
                            	i -= 1;
                            	continue;
                            }
                            //ball is still there
                            ball = new Location(nearestBall.x, nearestBall.y);
                            break;
                        } 
                        //no ball in sight, turn 45 degrees and try again
                        else {
                        	Log.v(TAG, "no ball detected");
	                        try {
		                        sleep(100);
		                    } catch (InterruptedException e) {
		                            e.printStackTrace();
		                    }
		                    robot.turn(45);
		                    
		                    //already made a 360 by now
		                    if (i == 7) {
		                    	//TODO something to find a ball that is not within cam range
		                    }
	                    }        
                    }
                    
                    state = State.GOTO_BALL;
                    break;
                    
                case GOTO_BALL:
                    // TODO place all the shit we already implemented here (and improve it by recalculating path)
                    // if ball is lost state = SEARCH_BALL else state = CAGE_BALL
                	//^not sure how �tis intended
                	robot.turnToLocation(ball);
                	robot.drive(Robot.euclideanDistance(robot.robotLocation, ball) - 15);
                	
                	findBall(greenBallHsv);
                	if (ballDetected()) {
                		state = State.CAGE_BALL;
                	} else {
                		state = State.SEARCH_BALL;
                	}
                    break;
                
                // cage ball
                case CAGE_BALL:
                	if (!robot.cageOpen) {
                		robot.openCage();
                	}
                    robot.closeCage();
                    robot.balls_in_cage += 1;
                    state = State.LOCALIZE;
                    break;
                    
                // drive robot to target and drop it there
                case BALL_TO_TARGET:
                    robot.turnToGoal();
                    robot.drive(Robot.euclideanDistance(robot.robotLocation, robot.goal) - 15);
                    state = State.DROP_BALL;
                    break;
                    
                // drop all balls in cage and search for new balls if existant
                case DROP_BALL:
                    robot.openCage();
                    ball_count -= robot.balls_in_cage;
                    robot.balls_in_cage = 0;
                    
                    if (ball_count == 0) {
                    	state = State.FIN;
                    }
                    else {
                    	state = State.SEARCH_BALL;
                    }
                    break;
                    
                // robot finished task
                case FIN:
                    return;
            
        }
    }


    public void relocate() {
        Pair<Beacon,Beacon> beacons = beaconDetector.getBeacons();
        double d, x, y, r1, r2;
        Point lowestPoint1 = beacons.first.egocentricCoordinates;
        Point lowestPoint2 = beacons.second.egocentricCoordinates;
        // distance to left beacon
        r1 = lowestPoint1.y/Math.sin(Math.atan(lowestPoint1.y/lowestPoint1.x));
        // distance to right beacon
        r2 = lowestPoint2.y/Math.sin(Math.atan(lowestPoint2.y/lowestPoint2.x));
        d = 125.0;
        x = (Math.pow(d, 2) - Math.pow(r2, 2) + Math.pow(r1, 2))/(2*d);
        y = 1/(2*d) * Math.sqrt((-d + r2 - r1)*(-d - r2 + r1)*(-d + r2 + r1)*(d + r2 + r1));
        Beacon cornerBeacon;
        boolean isLeft = false;
        double robotX = 0.0;
        double robotY = 0.0;
        double robotTheta = 90.0 - Math.atan(beacons.second.egocentricCoordinates.x/(-(beacons.second.egocentricCoordinates.y)));
        robotTheta += Math.acos((Math.pow(125.0, 2) + Math.pow(r2, 2) - Math.pow(r1, 2))/(2 * 125.0 * r1)); //law of cosines
        if(beacons.first.coordinates.x != 0 && beacons.first.coordinates.y != 0){
        	cornerBeacon = beacons.first;
        	isLeft = true;
        }else{
        	cornerBeacon = beacons.second;
        }
        switch(cornerBeacon){
        
        case RED_BLACK:	//beacon in upper right
        	if(isLeft){
        		robotX = cornerBeacon.coordinates.x - y;
        		robotY = cornerBeacon.coordinates.y - x;
        		robotTheta += 270.0;
        	}else{
        		robotX = cornerBeacon.coordinates.x - x;
        		robotY = cornerBeacon.coordinates.y - y;
        	}
        	break;
        case BLACK_RED: //beacon in lower right
        	if(isLeft){
        		robotX = cornerBeacon.coordinates.x - x;
        		robotY = cornerBeacon.coordinates.y + y;
        		robotTheta += 180.0;
        	}else{
        		robotX = cornerBeacon.coordinates.x - y;
        		robotY = cornerBeacon.coordinates.y + x;
        		robotTheta += 270.0;
        	}
        	break;
        case BLACK_BLUE: //beacon in lower left
        	if(isLeft){
        		robotX = cornerBeacon.coordinates.x + y;
        		robotY = cornerBeacon.coordinates.y + x;
        		robotTheta += 90.0;
        	}else{
        		robotX = cornerBeacon.coordinates.x + x;
        		robotY = cornerBeacon.coordinates.y + y;
        		robotTheta += 180.0;
        	}
        	break;
        case BLUE_BLACK: //beacon in upper left
        	if(isLeft){
        		robotX = cornerBeacon.coordinates.x + x;
        		robotY = cornerBeacon.coordinates.y - y;
        	}else{
        		robotX = cornerBeacon.coordinates.y + x;
        		robotY = cornerBeacon.coordinates.x - y;
        		robotTheta += 90.0;
        	}
        	break;
        }
        robot.robotLocation.setX(robotX);
        robot.robotLocation.setY(robotY);
        robot.robotLocation.setTheta(robotTheta);	//this should be the right theta now
        
        //this should probably be the right theta, if not we have to use 3 beacons which is a chore
        /*double egoTheta = Math.atan(beacons.first.egocentricCoordinates.y / beacons.first.egocentricCoordinates.x);
        double absTheta = Math.atan(beacons.first.coordinates.y - robotY / beacons.first.coordinates.x - robotX);
        
        robot.robotLocation.setTheta(Math.abs(absTheta) - Math.abs(egoTheta));*/
    }

    /**
     * when this method is called, ballLocationUpdated is set to false again
     * @return if the last frame contains a ball that can be targeted
     */
    public boolean ballDetected() {
        ballLocationUpdated = false;
        return nearestBall != null;
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
                Point targetEgo = new Point(nearestBall.x,nearestBall.y);
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
        if (testmode) return imgPoint;
        if (homographyMatrix == null) throw new RuntimeException("we don't even have a homography matrix yet!");
        if (imgPoint == null) return null;
        Mat src =  new Mat(1, 1, CvType.CV_32FC2);
        Mat dest = new Mat(1, 1, CvType.CV_32FC2);
        src.put(0, 0, imgPoint.x, imgPoint.y);
        Core.perspectiveTransform(src, dest, homographyMatrix);
        Point dest_point = new Point(dest.get(0, 0)[1]/10, -dest.get(0, 0)[0]/10);
        Log.v(TAG, "coordinates: " + dest_point.x + ", " + dest_point.y);
        return dest_point;
    }

    public static Mat getHomographyMatrix(Mat mRgba) {
    	  final Size mPatternSize = new Size(6, 9); // number of inner corners in the used chessboard pattern 
    	  float x = -48.0f; // coordinates of first detected inner corner on chessboard
    	  float y = 309.0f;
    	  float delta = 12.0f; // size of a single square edge in chessboard
    	  LinkedList<Point> PointList = new LinkedList<Point>();
    	 
    	  // Define real-world coordinates for given chessboard pattern:
    	  for (int i = 0; i < mPatternSize.height; i++) {
    	    y = 309.0f;
    	    for (int j = 0; j < mPatternSize.width; j++) {
    	      PointList.addLast(new Point(x,y));
    	      y += delta;
    	    }
    	    x += delta;
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

    /** sets the last touched color as green ball value */
    public void calibrateGreenBall(MenuItem item) {
        if (mBlobColorHsv != null) {
            greenBallHsv = mBlobColorHsv;
            mBlobColorHsv = null;
        }
    }

    /** sets the last touched color as red ball value */
    public void calibrateRedBall(MenuItem item) {
        if (mBlobColorHsv != null) {
            redBallHsv = mBlobColorHsv;
            mBlobColorHsv = null;
        }
    }

    public void calibrateRedBeacon(MenuItem item) {
        if (mBlobColorHsv != null) {
            BeaconColor.RED.setHsvColor(mBlobColorHsv);
            mBlobColorHsv = null;
        }
    }

    public void calibrateBlueBeacon(MenuItem item) {
        if (mBlobColorHsv != null) {
            BeaconColor.BLUE.setHsvColor(mBlobColorHsv);
            mBlobColorHsv = null;
        }
    }

    public void calibratePurpleBeacon(MenuItem item) {
        if (mBlobColorHsv != null) {
            BeaconColor.PURPLE.setHsvColor(mBlobColorHsv);
            mBlobColorHsv = null;
        }
    }

    public void calibrateBlackBeacon(MenuItem item) {
        if (mBlobColorHsv != null) {
            BeaconColor.BLACK.setHsvColor(mBlobColorHsv);
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

        target = new Location(dx,dy);

        procedureThread = new Thread() {
            @Override
            public void run() {
                secondExamination();
            }
        };
        procedureThread.start();
    }

    public void stopExam2(View view) {
        procedureThread.stop();
        robot.stop();
    }
}
