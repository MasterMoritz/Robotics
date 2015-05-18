package com.ebstor.robot.beacons;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by johannes on 5/17/15.
 */
public class BeaconContour {

    private MatOfPoint contour;
    /** rightest leftest and lowest point */
    private Point[] triple = null;

    public BeaconContour(MatOfPoint contour) {
        this.contour = contour;
    }

    /**
     *
     * @return [0]: lowest point, [1]: leftest point, [2]: rightest point
     */
    public Point[] getTriple() {
        if (triple == null) {
            List<Point> points = contour.toList();
            Point lowest, rightest, leftest;
            lowest = points.get(0);
            rightest = points.get(0);
            leftest = points.get(0);
            for (Point p: points) {
                if (p.y > lowest.y) lowest = p;
                if (p.x > rightest.x) rightest = p;
                if (p.x < leftest.x) leftest = p;
            }
            triple = new Point[3];
            triple[0] = lowest;
            triple[1] = leftest;
            triple[2] = rightest;
        }
        return triple;
    }
}
