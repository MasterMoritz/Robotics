package com.ebstor.robot.beacons;

import android.util.Log;
import android.util.Pair;
import com.ebstor.robot.ColorBlobDetectionActivity;
import com.ebstor.robot.corefunctions.ColorBlobDetector;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import java.util.*;

/**
 * Created by johannes on 5/17/15.
 * process an rgba frame and store the detected beacons
 */
public class BeaconDetector {

    private static final String TAG = "BeaconDetector";
    private List<Beacon> beaconsDetected;
    private ColorBlobDetector blobDetector;
    private Map<BeaconColor, List<BeaconContour>> beaconContours;


    /**
     * @return 2 neighboring beacons or null if there are not enough beacons seen
     */
    public Pair<Beacon, Beacon> getBeacons() {
        // enums are ordered by their ordinals, this works as long as they are ordered correctly in the declaration
        Collections.sort(beaconsDetected);
        for (Beacon b : beaconsDetected)
            Log.v(TAG, "beacon detected: " + b.toString());
        try {
            return new Pair<>(beaconsDetected.get(0), beaconsDetected.get(1));
        } catch (IndexOutOfBoundsException e) {
            return null;
        }
    }

    public BeaconDetector(ColorBlobDetector blobDetector) {
        this.beaconsDetected = new LinkedList<>();
        this.blobDetector = blobDetector;
        this.beaconContours = new EnumMap<>(BeaconColor.class);
        for (BeaconColor color : BeaconColor.values())
            beaconContours.put(color, new LinkedList<BeaconContour>());
    }


    public void process(Mat rgbaImage) {
        beaconsDetected.clear();
        BeaconColor[] values = BeaconColor.values();
        for (BeaconColor beaconColor : values) {
            Scalar hsvColor = beaconColor.hsvColor();
            blobDetector.setHsvColor(hsvColor);
            blobDetector.process(rgbaImage);
            List<MatOfPoint> contours = blobDetector.getContours();
            List<BeaconContour> beaconContours = this.beaconContours.get(beaconColor);
            for (MatOfPoint contour : contours)
                beaconContours.add(new BeaconContour(contour));
        }
        /* compare every red contour with every (differently colored) contour */
        List<BeaconContour> redContours = beaconContours.get(BeaconColor.RED);
        for (int i = 0; i < values.length; i++)
            if (values[i] != BeaconColor.RED)
                for (BeaconContour beaconContour2 : beaconContours.get(values[i]))
                    for (BeaconContour beaconContour : redContours) {
                        int comp = compareContours(beaconContour.get4Tuple(), beaconContour2.get4Tuple());
                        if (comp != 0) {
                            Beacon beaconRed = findRedBeacons(comp, values[i], beaconContour.get4Tuple(), beaconContour2.get4Tuple());
                            if (beaconRed != null)
                                beaconsDetected.add(beaconRed);
                        }
                    }

        /* compare every blue contour with every (differently colored) contour */
        List<BeaconContour> blueContours = beaconContours.get(BeaconColor.BLUE);
        for (int i = 0; i < values.length; i++)
            if (values[i] != BeaconColor.BLUE)
                for (BeaconContour beaconContour2 : beaconContours.get(values[i]))
                    for (BeaconContour beaconContour : blueContours) {
                        int comp = compareContours(beaconContour.get4Tuple(), beaconContour2.get4Tuple());
                        if (comp != 0) {
                            Beacon beaconBlue = findBlueBeacons(comp, values[i], beaconContour.get4Tuple(), beaconContour2.get4Tuple());
                            if (beaconBlue != null)
                                beaconsDetected.add(beaconBlue);
                        }
                    }
    }


    private Beacon findRedBeacons(int comp, BeaconColor color, Point[] red4Tuple, Point[] other4Tuple) {
        // if comp < 0 then color is above red, triple2 is above triple1
        Beacon result = null;
        switch (color) {
            case BLUE:
                if (comp < 0) {
                    result = Beacon.BLUE_RED;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(red4Tuple[0]);
                } else {
                    result = Beacon.RED_BLUE;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(other4Tuple[0]);
                }
                break;
            case PURPLE:
                if (comp < 0) {
                    result = Beacon.PURPLE_RED;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(red4Tuple[0]);
                }
                break;
            case BLACK:
                if (comp < 0) {
                    result = Beacon.BLACK_RED;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(red4Tuple[0]);
                } else {
                    result = Beacon.RED_BLACK;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(other4Tuple[0]);
                }
                break;
            default:
                return null;
        }
        return result;
    }

    private Beacon findBlueBeacons(int comp, BeaconColor color, Point[] blue4Tuple, Point[] other4Tuple) {
         /* if comp < 0 then color is above blue,  triple2 is above triple1
            red has already been detected
          */
        Beacon result = null;
        switch (color) {
            case BLACK:
                if (comp < 0) {
                    result = Beacon.BLACK_BLUE;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(blue4Tuple[0]);
                } else {
                    result = Beacon.BLUE_BLACK;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(other4Tuple[0]);
                }
                break;
            case PURPLE:
                if (comp < 0) {
                    result = Beacon.PURPLE_BLUE;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(other4Tuple[0]);
                }
                break;
            default:
                return null;
        }
        return result;
    }

    /**
     * @return -1 if fourTuple2 is above fourTuple1, 1 if fourTuple2 is below fourTuple1,
     * 0 if contours do not belong to same beacon
     */
    private int compareContours(Point[] fourTuple1, Point[] fourTuple2) {
        /* check if their diameters are similar
        * if one diameter is less than 75% of the other, the contours do not belong to one beacon*/
        double diameter1 = Math.abs(fourTuple1[1].x - fourTuple1[2].x);
        double diameter2 = Math.abs(fourTuple2[1].x - fourTuple2[2].x);
        if (diameter1/diameter2 < 0.75 || diameter2/diameter1 < 0.75) return 0;

        /* check if the contours adjoin
         * 2 diffs because we don't know which one is on top, lazy version
        */
        double diff1 = Math.abs((fourTuple1[3].y-fourTuple2[3].y)/(fourTuple1[0].y - fourTuple1[3].y));
        double diff2 = Math.abs((fourTuple2[3].y-fourTuple1[3].y)/(fourTuple1[0].y - fourTuple1[3].y));
        /* if the difference between top of one contour and lowest of the other (divided by top-bottom to make it
         * independent of the distance) is smaller than 1/10 then the contours do not adjoin */
        if (diff1 < 1/10. && diff2 < 1/10.) return 0;

        /* check if they are in the same x area */
        double diameterX = diameter1/ 2;
        double x = Math.abs(fourTuple2[1].x - fourTuple2[2].x) / 2 + fourTuple2[1].x;
        if (x > (fourTuple1[1].x - diameterX) && x < (fourTuple1[2].x + diameterX))
            return Double.compare(fourTuple2[0].y, fourTuple1[0].y);
        else
            return 0;
    }

}
