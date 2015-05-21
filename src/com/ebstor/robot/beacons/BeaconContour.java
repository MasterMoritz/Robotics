package com.ebstor.robot.beacons;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

import java.util.List;

/**
 * Created by johannes on 5/17/15.
 */
public class BeaconContour {

    private MatOfPoint contour;
    /** rightest leftest and lowest point */
    private Point[] fourTuple = null;

    public BeaconContour(MatOfPoint contour) {
        this.contour = contour;
    }

    /**
     *
     * @return [0]: lowest point, [1]: leftest point, [2]: rightest point, [3]: top point
     */
    public Point[] get4Tuple() {
        if (fourTuple == null) {
            List<Point> points = contour.toList();
            Point lowest, rightest, leftest, top;
            lowest = points.get(0);
            rightest = points.get(0);
            leftest = points.get(0);
            top = points.get(0);

            for (Point p: points) {
                if (p.y > lowest.y) lowest = p;
                if (p.x > rightest.x) rightest = p;
                if (p.x < leftest.x) leftest = p;
                if (p.y < top.y) top = p;
            }
            fourTuple = new Point[4];
            fourTuple[0] = lowest;
            fourTuple[1] = leftest;
            fourTuple[2] = rightest;
            fourTuple[3] = top;
        }
        return fourTuple;
    }
}
