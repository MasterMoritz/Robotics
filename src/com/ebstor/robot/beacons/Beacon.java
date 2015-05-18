package com.ebstor.robot.beacons;

import org.opencv.core.Point;
import org.opencv.core.Scalar;

/**
 * Created by johannes on 5/11/15.
 */

public enum Beacon {

    BLUE_RED,
    RED_BLACK,
    PURPLE_RED,
    BLACK_RED,
    RED_BLUE,
    BLACK_BLUE,
    PURPLE_BLUE,
    BLUE_BLACK;

    public Point coordinates;
    public Point egocentricCoordinates;

    static {
        BLUE_RED.coordinates = new Point(0,125);
        RED_BLACK.coordinates = new Point(125,125);
        PURPLE_RED.coordinates = new Point(125,0);
        BLACK_RED.coordinates = new Point(125,-125);
        RED_BLUE.coordinates = new Point(0,-125);
        BLACK_BLUE.coordinates = new Point(-125,-125);
        PURPLE_BLUE.coordinates = new Point(-125,0);
        BLUE_BLACK.coordinates = new Point(-125,125);
    }

}
