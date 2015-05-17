package com.ebstor.robot.corefunctions;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import java.util.*;

/**
 * Created by johannes on 5/17/15.
 */
public class BeaconDetector {

    private List<Beacon> beaconsDetected;
    private ColorBlobDetector blobDetector;
    private Map<BeaconColor,List<BeaconContour>> beaconContours;

    public List<Beacon> getBeacons() {
        return beaconsDetected;
    }

    public BeaconDetector(ColorBlobDetector blobDetector) {
        this.beaconsDetected = new ArrayList<>();
        this.blobDetector = blobDetector;
        this.beaconContours = new EnumMap<>(BeaconColor.class);
        for (BeaconColor color: BeaconColor.values())
            beaconContours.put(color,new LinkedList<BeaconContour>());
    }


    public void process(Mat rgbaImage) {
        beaconsDetected.clear();
        BeaconColor[] values = BeaconColor.values();
        for (BeaconColor beaconColor: values) {
            Scalar hsvColor = beaconColor.hsvColor;
            blobDetector.setHsvColor(hsvColor);
            blobDetector.process(rgbaImage);
            List<MatOfPoint> contours = blobDetector.getContours();
            List<BeaconContour> beaconContours = this.beaconContours.get(beaconColor);
            for (MatOfPoint contour: contours)
                beaconContours.add(new BeaconContour(contour));
        }
        /* compare every contour with every (differently colored) contour */
        for (int i = 0; i < values.length-1; i++) {
            for (BeaconContour beaconContour: beaconContours.get(values[i])) {
                for (BeaconContour beaconContour2: beaconContours.get(values[i+1])) {
                    int comp = compareContours(beaconContour.getTriple(),beaconContour2.getTriple());
                    if (comp != 0)
                        switch(values[i]) {
                            case RED:
                                findRedBeacons(comp, values[i + 1]);
                                break;
                            case BLUE:
                                findBlueBeacons(comp, values[i + 1]);
                                break;
                        }
                }
            }
        }
    }

    /* there are separate methods for all different colors */

    private Beacon findRedBeacons(int comp, BeaconColor color) {
        /* if comp < 0 then color is above red */
        switch(color) {
            case BLUE:
                if (comp < 0) return Beacon.BLUE_RED;
                else return Beacon.RED_BLUE;
            case PURPLE:
                if (comp < 0 ) return Beacon.PURPLE_RED;
                else return null;
            case BLACK:
                if (comp < 0) return Beacon.BLACK_RED;
                else return Beacon.RED_BLACK;
            default:
                return null;
        }
    }

    private Beacon findBlueBeacons(int comp, BeaconColor color) {
         /* if comp < 0 then color is above blue
            red has already been detected
          */
        switch (color) {
            /*case RED:
                if (comp < 0 ) return Beacon.RED_BLUE;
                else return Beacon.BLUE_RED;*/
            case BLACK:
                if (comp < 0) return Beacon.BLACK_BLUE;
                else return Beacon.BLUE_BLACK;
            case PURPLE:
                if (comp < 0) return Beacon.PURPLE_BLUE;
                else return null;
            default:
                return null;
        }
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
        double areaX = Math.abs(triple1[1].x - triple1[2].x) + 10;
        double x = Math.abs(triple2[1].x - triple2[2].x)/2 + triple2[1].x;
        if (x > (triple1[1].x - areaX) && x < (triple1[1].x + 2 * areaX))
            return (int)(Math.signum(triple1[0].y - triple2[0].y));
        else
            return 0;
    }

}
