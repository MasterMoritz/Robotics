package com.ebstor.robot;

import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

import com.ebstor.robot.corefunctions.ColorBlobDetector;
import com.ebstor.robot.R;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.*;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgproc.Imgproc;

import com.ebstor.robot.corefunctions.ColorBlobDetector;
import com.ebstor.robot.corefunctions.Robot;


import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;

public class ColorBlobDetectionActivity extends MainActivity implements CvCameraViewListener2 {
    private static final String  TAG              = "ColorBlobActivity";
    private static final Scalar GREEN_BALL_RGBA = new Scalar(0,88,21,255);

    private Mat                  mRgba;
    private Scalar               mBlobColorRgba;
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
                    //mOpenCvCameraView.setOnTouchListener(ColorBlobDetectionActivity.this);
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
        mBlobColorRgba = GREEN_BALL_RGBA;
        mBlobColorHsv = convertScalarRgba2Hsv(mBlobColorRgba);
        mDetector.setHsvColor(mBlobColorHsv);
        mSpectrum = new Mat();
        SPECTRUM_SIZE = new Size(200, 64);
        CONTOUR_COLOR = new Scalar(255,0,0,255);
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

        mDetector.process(mRgba);
        List<MatOfPoint> contours = mDetector.getContours();
        Log.e(TAG, "Contours count: " + contours.size());
        Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);

        Mat colorLabel = mRgba.submat(4, 68, 4, 68);
        colorLabel.setTo(mBlobColorRgba);

        Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
        mSpectrum.copyTo(spectrumLabel);
        
        List<Point> points = new LinkedList<>();
        for (MatOfPoint mat: contours) {
            points.addAll(mat.toList());
        }

        if (!points.isEmpty()) {
            Log.v(TAG,"points: ");
            for (Point p: points) Log.v(TAG, p.toString());
            Point lowestPoint = Collections.min(points, new Comparator<Point>() {
                @Override
                public int compare(Point lhs, Point rhs) {
                    return Double.compare(lhs.y,rhs.y);
                }
            });
            System.out.println("LOWEST POINT" + lowestPoint);
            /* now turn the robot until the lowest point is somewhere in the middle,
             then drive until it is far down in the image */
            if(!isInMiddle(inputFrame, lowestPoint)){
            	
            	Mat myFrame = inputFrame.rgba();
            	if(lowestPoint.x < (myFrame.cols()/2)){
            		robot.turnRight();
            	}else{
            		robot.turnLeft();
            	}
            }else{
            	robot.stop();
            }
            if(isInMiddle(inputFrame, lowestPoint) && !isAtBottom(inputFrame, lowestPoint)){
            	robot.drive();
            }else if(isInMiddle(inputFrame, lowestPoint) && isAtBottom(inputFrame, lowestPoint)){
            	robot.stop();
            }
        }
        
        return mRgba;
    }

    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }
    
    public Boolean isInMiddle(CvCameraViewFrame inputFrame, Point p){
    	int parts = 5; // odd number of partitions of the inputFrame
    	Mat myFrame = inputFrame.rgba();
    	// is in middle partition?
        return (p.x >= (parts/2)/parts*myFrame.cols() && p.x <= (parts/2+1)/parts*myFrame.cols());
    }
    
    public Boolean isAtBottom(CvCameraViewFrame inputFrame, Point p){
    	int parts = 7; // odd number of partitions of the inputframe
    	Mat myFrame = inputFrame.rgba();
    	// is in bottom partition?
        return (p.y <= 1/parts*myFrame.rows());
    }


  /* public boolean onTouch(View v, MotionEvent event) {
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

        mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);

        Log.i(TAG, "Touched rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
                ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");

        mDetector.setHsvColor(mBlobColorHsv);

        Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

        mIsColorSelected = true;

        touchedRegionRgba.release();
        touchedRegionHsv.release();

        return false; // don't need subsequent touch events
    }*/
}
