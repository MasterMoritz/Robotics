package com.ebstor.robot;

import org.opencv.core.*;

import java.util.*;

import android.app.Activity;
import android.view.*;
import com.ebstor.robot.corefunctions.ColorBlobDetector;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.*;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgproc.Imgproc;
import org.opencv.calib3d.*;


import android.os.Bundle;
import android.util.Log;
import android.view.View.OnTouchListener;

public class ColorBlobDetectionActivity extends Activity implements OnTouchListener, CvCameraViewListener2 {
    private static final String  TAG              = "ColorBlobActivity";
    private static final Scalar GREEN_BALL_RGBA = new Scalar(12,75,12,255);
    private static final Scalar LOWEST_POINT_RGBA = new Scalar(34,200,1,255);

    private Mat                  mRgba;
    /** map from all color names to values */
    private Map<String,Scalar>   mBlobColorsRgba;
    private Map<String,Scalar>   mBlobColorsHsv;
    /** currently chosen blob colors */
    private Scalar               mBlobColorRgba = new Scalar(0);
    private Scalar               mBlobColorHsv = new Scalar(0);
    private ColorBlobDetector    mDetector;
    private Mat                  mSpectrum;
    private Size                 SPECTRUM_SIZE;
    private Scalar               CONTOUR_COLOR;

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

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.color_blob_detection_surface_view);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
        mOpenCvCameraView.setCvCameraViewListener(this);


    }

    @Override
    public void onPause()
    {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
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
        mSpectrum = new Mat();
        SPECTRUM_SIZE = new Size(200, 64);
        CONTOUR_COLOR = new Scalar(255,0,0,255);

        mBlobColorsHsv = new HashMap<>();
        mBlobColorsRgba = new HashMap<>();
        mBlobColorsRgba.put("green", GREEN_BALL_RGBA);
        mBlobColorsHsv.put("green", convertScalarRgba2Hsv(GREEN_BALL_RGBA));
    }

    public void onCameraViewStopped() {
        mRgba.release();
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


    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
        List<MatOfPoint> allContours = new LinkedList<>();
        for (Map.Entry<String,Scalar> color: mBlobColorsHsv.entrySet()) {
            mDetector.setHsvColor(color.getValue());
            mDetector.process(mRgba);
            List<MatOfPoint> contours = mDetector.getContours();
            allContours.addAll(contours);
            Log.e(TAG, "Contours count: " + contours.size());
            Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);
        }


        /*Mat colorLabel = mRgba.submat(4, 68, 4, 68);
        colorLabel.setTo(mBlobColorsRgba);

        Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
        mSpectrum.copyTo(spectrumLabel);*/

        List<Point> points = new LinkedList<>();
        for (MatOfPoint mat: allContours) {
            points.addAll(mat.toList());
        }

        if (!points.isEmpty()) {
            Log.v(TAG,"points: ");
            for (Point p: points) Log.v(TAG, p.toString());
            Point lowestPoint = Collections.max(points, new Comparator<Point>() {
                @Override
                public int compare(Point lhs, Point rhs) {
                    return Double.compare(lhs.y, rhs.y);
                }
            });

            List<MatOfPoint> lowestPointlist = new LinkedList<>();
            lowestPointlist.add(new MatOfPoint(
                    lowestPoint,
                    new Point(lowestPoint.x-1,lowestPoint.y),
                    new Point(lowestPoint.x,lowestPoint.y+1),
                    new Point(lowestPoint.x+1,lowestPoint.y),
                    new Point(lowestPoint.x,lowestPoint.y-1)));
            Imgproc.drawContours(mRgba,lowestPointlist,-1,LOWEST_POINT_RGBA);

            Log.v(TAG,"lowest point: " + lowestPoint.toString());
            /* now turn the robot until the lowest point is somewhere in the middle,
             then drive until it is far down in the image */
            Log.v(TAG,"is in middle: " + isInMiddle(lowestPoint));
            Log.v(TAG,"is at bottom: " + isAtBottom(lowestPoint));
            /*if(!isInMiddle(lowestPoint)){
            	
            	Mat myFrame = inputFrame.rgba();
            	if(lowestPoint.x < (myFrame.cols()/2)){
            		robot.turnRight();
            	}else{
            		robot.turnLeft();
            	}
            }else{
            	robot.stop();
            }
            if(isInMiddle(lowestPoint) && !isAtBottom(lowestPoint)){
            	robot.drive();
            }else if(isInMiddle(lowestPoint) && isAtBottom(lowestPoint)){
            	robot.stop();
            }*/
        }
        return mRgba;
    }
    
    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }
    public Boolean isInMiddle(Point p){
        return (p.x >= 2d/5*mRgba.cols() && p.x <= 3d/5*mRgba.cols());
    }
    
    public Boolean isAtBottom(Point p){
        return (p.y >= 4d/5*mRgba.rows());
    }

    public void pointCoordinates(View v){
    	
        Thread t = new Thread(){
        	public void run(){
                Mat frame = mRgba;
                Mat homography = getHomographyMatrix(frame);
                Point mp = new Point(frame.cols()/2, frame.rows()/2);
    	        while(homography.rows() <= 0 && homography.cols() <= 0){
    	        	homography = getHomographyMatrix(frame);
    	        }
    	        Mat src =  new Mat(1, 1, CvType.CV_32FC2);
    	        Mat dest = new Mat(1, 1, CvType.CV_32FC2);
    	        src.put(0, 0, new double[] { mp.x, mp.y }); // ps is a point in image coordinates
    	        Core.perspectiveTransform(src, dest, homography); //homography is your homography matrix
    	        Point dest_point = new Point(dest.get(0, 0)[0], dest.get(0, 0)[1]);
    	        Log.v(TAG, "coordinates: " + dest_point.x + ", " + dest_point.y);
            }
        };
        t.start();
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
    	    return Calib3d.findHomography(mCorners, RealWorldC);
    	  }else{
    	    return new Mat();
    	  }
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

        mBlobColorsHsv.put("current",mBlobColorHsv);
        mBlobColorRgba = convertScalarHsv2Rgba(mBlobColorHsv);

        Log.i(TAG, "Touched rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
                ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");


        //Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

        touchedRegionRgba.release();
        touchedRegionHsv.release();

        return false; // don't need subsequent touch events
    }

    /** sets the last touched color as green ball value */
    public void calibrateGreenBall(MenuItem item) {
        mBlobColorsHsv.put("green",mBlobColorHsv);
    }

    /** sets the last touched color as red ball value */
    public void calibrateRedBall(MenuItem item) {
        mBlobColorsHsv.put("red",mBlobColorHsv);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.camera_menu, menu);
        return true;
    }
}
