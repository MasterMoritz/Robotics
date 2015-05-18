package com.ebstor.robot.beacons;

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

    private List<Beacon> beaconsDetected;
    private Pair<Beacon,Beacon> significantBeacons;
    private ColorBlobDetector blobDetector;
    private Map<BeaconColor,List<BeaconContour>> beaconContours;


    /**
     *
     * @return 2 neighboring beacons or null if there are not enough beacons seen
     */
    public Pair<Beacon, Beacon> getBeacons() {
        // enums are ordered by their ordinals, this works as long as they are ordered correctly in the declaration
        Collections.sort(beaconsDetected);
        try {
            significantBeacons = new Pair<>(beaconsDetected.get(0),beaconsDetected.get(1));
        } catch (IndexOutOfBoundsException e) {
            return null;
        }
        return significantBeacons;
    }

    public BeaconDetector(ColorBlobDetector blobDetector) {
        this.beaconsDetected = new LinkedList<>();
        this.blobDetector = blobDetector;
        this.beaconContours = new EnumMap<>(BeaconColor.class);
        for (BeaconColor color: BeaconColor.values())
            beaconContours.put(color,new LinkedList<BeaconContour>());
    }


    public void process(Mat rgbaImage) {
        beaconsDetected.clear();
        BeaconColor[] values = BeaconColor.values();
        for (BeaconColor beaconColor: values) {
            Scalar hsvColor = beaconColor.hsvColor();
            blobDetector.setHsvColor(hsvColor);
            blobDetector.process(rgbaImage);
            List<MatOfPoint> contours = blobDetector.getContours();
            List<BeaconContour> beaconContours = this.beaconContours.get(beaconColor);
            for (MatOfPoint contour: contours)
                beaconContours.add(new BeaconContour(contour));
        }
        /* compare every red contour with every (differently colored) contour */
        List<BeaconContour> redContours = beaconContours.get(BeaconColor.RED);
        for (int i = 0; i < values.length; i++) {
            if (values[i] != BeaconColor.RED) {
                for (BeaconContour beaconContour2 : beaconContours.get(values[i])) {
                    for (BeaconContour beaconContour: redContours) {
                        int comp = compareContours(beaconContour.getTriple(), beaconContour2.getTriple());
                        if (comp != 0) {
                            Beacon beaconRed = findRedBeacons(comp, values[i], beaconContour.getTriple(), beaconContour2.getTriple());
                            if (beaconRed != null) {
                                beaconsDetected.add(beaconRed);
                            }
                        }
                    }
                }
            }
        }
        /* compare every blue contour with every (differently colored) contour */
        List <BeaconContour> blueContours = beaconContours.get(BeaconColor.BLUE);
        for (int i = 0; i < values.length; i++) {
            if (values[i] != BeaconColor.BLUE) {
                for (BeaconContour beaconContour2 : beaconContours.get(values[i])) {
                    for (BeaconContour beaconContour: blueContours) {
                        int comp = compareContours(beaconContour.getTriple(), beaconContour2.getTriple());
                        if (comp != 0) {
                            Beacon beaconBlue = findBlueBeacons(comp, values[i], beaconContour.getTriple(), beaconContour2.getTriple());
                            if (beaconBlue != null) {
                                beaconsDetected.add(beaconBlue);
                            }
                        }
                    }
                }
            }
        }
    }


    private Beacon findRedBeacons(int comp, BeaconColor color, Point[] redTriple, Point[] triple2) {
        // if comp < 0 then color is above red, triple2 is above triple1
        Beacon result = null;
        switch(color) {
            case BLUE:
                if (comp < 0) {
                    result = Beacon.BLUE_RED;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(redTriple[0]);
                }
                else {
                   result = Beacon.RED_BLUE;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(triple2[0]);
                }
                break;
            case PURPLE:
                if (comp < 0 ) {
                    result = Beacon.PURPLE_RED;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(redTriple[0]);
                }
                break;
            case BLACK:
                if (comp < 0) {
                    result = Beacon.BLACK_RED;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(redTriple[0]);
                }
                else {
                    result = Beacon.RED_BLACK;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(triple2[0]);
                }
                break;
            default:
                return null;
        }
        return result;
    }

    private Beacon findBlueBeacons(int comp, BeaconColor color, Point[] blueTriple, Point[] triple2) {
         /* if comp < 0 then color is above blue,  triple2 is above triple1
            red has already been detected
          */
        Beacon result = null;
        switch (color) {
            case BLACK:
                if (comp < 0) {
                    result = Beacon.BLACK_BLUE;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(blueTriple[0]);
                }
                else {
                    result = Beacon.BLUE_BLACK;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(triple2[0]);
                }
                break;
            case PURPLE:
                if (comp < 0) {
                    result = Beacon.PURPLE_BLUE;
                    result.egocentricCoordinates = ColorBlobDetectionActivity.imageCoordToEgoCoord(triple2[0]);
                }
                break;
            default:
                return null;
        }
        return result;
    }

    /*private Beacon findPurpleBeacons(int comp, BeaconColor color) {
        // if comp < 0 then color is above purple
        // red and blue have already been detected
        switch (color) {
            case RED:
                if (comp > 0) return Beacon.PURPLE_BLUE;
                else return null;
            case BLUE:
                if (comp > 0) return Beacon.PURPLE_RED;
                else return null;
            default:
                return null;
        }
    }*/

    /*private Beacon findBlackBeacons(int comp, BeaconColor color) {
        // if comp < 0 then color is above black
        // all colors have been detected
        switch (color) {
            case RED:
                if (comp < 0) return Beacon.RED_BLACK;
                else return Beacon.BLACK_RED;
            case BLUE:
                if (comp < 0) return Beacon.BLUE_BLACK;
                else return Beacon.BLACK_BLUE;
            default:
                return null;
        }
    } */

    /**
     *
     * @return -1 if triple2 is above triple1, 1 if triple2 is below triple1, 0 if contours do not belong to same beacon
     */
    private int compareContours(Point[] triple1, Point[] triple2) {
        double areaX = Math.abs(triple1[1].x - triple1[2].x)/2;
        double x = Math.abs(triple2[1].x - triple2[2].x)/2 + triple2[1].x;
        if (x > (triple1[1].x - areaX) && x < (triple1[1].x + 2 * areaX))
            return (int)(Math.signum(triple1[0].y - triple2[0].y));
        else
            return 0;
    }

}
