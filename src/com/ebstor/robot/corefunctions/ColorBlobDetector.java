package com.ebstor.robot.corefunctions;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ColorBlobDetector {
    // Minimum contour area in percent for contours filtering
    private static double mMinContourArea = 0.1;
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(25,50,50,0);
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();
    /**
     * green and red balls
     */
    private List<ColorBall> blobColorsHSV = new ArrayList<ColorBall>();

    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();

    public void setColorRadius(Scalar radius) {
        mColorRadius = radius;
    }

    public ColorBlobDetector() {

    }

    public void addColorBallHsv(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0]-mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0]+mColorRadius.val[0] <= 255) ? hsvColor.val[0]+mColorRadius.val[0] : 255;

        ColorBall ball = new ColorBall();

        Scalar lowerBound = new Scalar(0);
        Scalar upperBound = new Scalar(0);
        lowerBound.val[0] = minH;
        upperBound.val[0] = maxH;

        lowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
        upperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

        lowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
        upperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

        this.blobColorsHSV.add(ball);
    }


    public void setMinContourArea(double area) {
        mMinContourArea = area;
    }

    public void process(Mat rgbaImage) {
        Imgproc.pyrDown(rgbaImage, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        mContours.clear();
        for (ColorBall c: blobColorsHSV) {
            Scalar lowerBound = c.getmLowerBound();
            Scalar upperBound = c.getmUpperBound();

            Core.inRange(mHsvMat, lowerBound, upperBound, mMask);
            Imgproc.dilate(mMask, mDilatedMask, new Mat());

            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

            Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find max contour area
            double maxArea = 0;
            Iterator<MatOfPoint> each = contours.iterator();
            while (each.hasNext()) {
                MatOfPoint wrapper = each.next();
                double area = Imgproc.contourArea(wrapper);
                if (area > maxArea)
                    maxArea = area;
            }

            each = contours.iterator();
            while (each.hasNext()) {
                MatOfPoint contour = each.next();
                if (Imgproc.contourArea(contour) > mMinContourArea*maxArea) {
                    Core.multiply(contour, new Scalar(4,4), contour);
                    mContours.add(contour);
                }
            }
        }

    }

    public List<MatOfPoint> getContours() {
        return mContours;
    }
}
