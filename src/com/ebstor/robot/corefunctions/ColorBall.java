package com.ebstor.robot.corefunctions;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;

/**
 * Created by johannes on 4/20/15.
 */
public class ColorBall {

    public ColorBall(Scalar mLowerBound, Scalar mUpperBound, Scalar mColorRadius) {
        this.mLowerBound = mLowerBound;
        this.mUpperBound = mUpperBound;
        this.mColorRadius = mColorRadius;
    }

    public ColorBall() {
    }

    public Scalar getmLowerBound() {
        return mLowerBound;
    }

    public Scalar getmUpperBound() {
        return mUpperBound;
    }

    public Scalar getmColorRadius() {
        return mColorRadius;
    }

    public void setmLowerBound(Scalar mLowerBound) {
        this.mLowerBound = mLowerBound;
    }

    public void setmUpperBound(Scalar mUpperBound) {
        this.mUpperBound = mUpperBound;
    }

    public void setmColorRadius(Scalar mColorRadius) {
        this.mColorRadius = mColorRadius;
    }

    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);

    private Scalar mColorRadius = new Scalar(25,50,50,0);


}
