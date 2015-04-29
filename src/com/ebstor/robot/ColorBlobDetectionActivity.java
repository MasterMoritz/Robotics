package com.ebstor.robot;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.*;
import android.view.View.OnTouchListener;
import com.ebstor.robot.corefunctions.ColorBlobDetector;
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

public class ColorBlobDetectionActivity extends Activity implements OnTouchListener, CvCameraViewListener2 {
    private static final String  TAG              = "ColorBlobActivity";
    private static final Scalar  GREEN_BALL_RGBA = new Scalar(12,75,12,255);
    private static final Scalar  RED_BALL_HSV = new Scalar(360,100,60);
    private static final Scalar  LOWEST_POINT_RGBA = new Scalar(34,200,1,255);
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
    /** currently chosen blob colors */
    //private Scalar               mBlobColorRgba = new Scalar(0);
    private Scalar               mBlobColorHsv;
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

        greenBallHsv = convertScalarRgba2Hsv(GREEN_BALL_RGBA);
        redBallHsv = RED_BALL_HSV;
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
        if (homographyMatrix == null) {
            homographyMatrix = getHomographyMatrix(mRgba);
        } else {
            List<MatOfPoint> greenBallContours, redBallContours;
            Point lowestPointGreen = null, lowestPointRed = null, lowestPoint = null;
            List<Point> points = new LinkedList<>();
            List<MatOfPoint> lowestPointlist = new LinkedList<>();

            /* detect green balls */
            mDetector.setHsvColor(greenBallHsv);
            mDetector.process(mRgba);
            greenBallContours = mDetector.getContours();
            Log.e(TAG, "green contours count: " + greenBallContours.size());
            Imgproc.drawContours(mRgba, greenBallContours, -1, CONTOUR_COLOR);
            for (MatOfPoint m : greenBallContours)
                points.addAll(m.toList());

            if (!points.isEmpty()) {
                lowestPointGreen = Collections.max(points, pointComparator);
                lowestPoint = lowestPointGreen;
                lowestPointlist.add(new MatOfPoint(
                        lowestPointGreen,
                        new Point(lowestPointGreen.x - 1, lowestPointGreen.y),
                        new Point(lowestPointGreen.x, lowestPointGreen.y + 1),
                        new Point(lowestPointGreen.x + 1, lowestPointGreen.y),
                        new Point(lowestPointGreen.x, lowestPointGreen.y - 1)));
                Imgproc.drawContours(mRgba, lowestPointlist, -1, LOWEST_POINT_RGBA);
            }

            /* detect red balls*/

            mDetector.setHsvColor(redBallHsv);
            mDetector.process(mRgba);
            redBallContours = mDetector.getContours();
            Log.e(TAG, "red contours count: " + redBallContours.size());
            Imgproc.drawContours(mRgba, redBallContours, -1, CONTOUR_COLOR);

            points.clear();
            for (MatOfPoint m : redBallContours)
                points.addAll(m.toList());

            if (!points.isEmpty()) {
                lowestPointRed = Collections.max(points, pointComparator);

                lowestPointlist.clear();
                lowestPointlist.add(new MatOfPoint(
                        lowestPointRed,
                        new Point(lowestPointRed.x - 1, lowestPointRed.y),
                        new Point(lowestPointRed.x + 1, lowestPointRed.y),
                        new Point(lowestPointRed.x, lowestPointRed.y - 1)));
                Imgproc.drawContours(mRgba, lowestPointlist, -1, LOWEST_POINT_RGBA);
            }


            /* detect currently set color for calibration */
            if (mBlobColorHsv != null) {
                mDetector.setHsvColor(mBlobColorHsv);
                mDetector.process(mRgba);
                Imgproc.drawContours(mRgba, mDetector.getContours(), -1, new Scalar(255, 255, 255, 255));
            }

            lowestPoint = getLowestPoint(lowestPointGreen, lowestPointRed);
            if (lowestPoint != null) {
                Log.v(TAG, "lowest point on image: " + lowestPoint.toString());
                Log.v(TAG, "lowest point in egocentric coordinates: " + imageCoordToRealWorldCoord(lowestPoint).toString());
            }

        }

      /*Mat colorLabel = mRgba.submat(4, 68, 4, 68);
        colorLabel.setTo(mBlobColorsRgba);

        Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
        mSpectrum.copyTo(spectrumLabel);*/
        return mRgba;
    }

    /**
     *
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

    public Boolean isInMiddle(Point p){
        return (p.x >= 2d/5*mRgba.cols() && p.x <= 3d/5*mRgba.cols());
    }
    
    public Boolean isAtBottom(Point p){
        return (p.y >= 4d/5*mRgba.rows());
    }


    public Point imageCoordToRealWorldCoord(Point imgPoint) {
        Mat src =  new Mat(1, 1, CvType.CV_32FC2);
        Mat dest = new Mat(1, 1, CvType.CV_32FC2);
        src.put(0, 0, imgPoint.x, imgPoint.y);
        Core.perspectiveTransform(src, dest, homographyMatrix);
        Point dest_point = new Point(dest.get(0, 0)[0], dest.get(0, 0)[1]);
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

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.camera_menu, menu);
        return true;
    }
}
