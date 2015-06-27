package com.ebstor.robot.beacons;

import org.opencv.core.Point;

/**
 * Created by johannes on 5/11/15.
 */

public enum Beacon {

    BLUE_RED,
    RED_YELLOW, //upper right
    BLUE_GREEN,
    YELLOW_RED, //lower right
    RED_BLUE,
    YELLOW_BLUE, //lower left
    GREEN_BLUE,
    BLUE_YELLOW; //upper left

    public Point coordinates;
    public Point egocentricCoordinates;

    static {
        BLUE_RED.coordinates = new Point(0,125);
        RED_YELLOW.coordinates = new Point(125,125);
        BLUE_GREEN.coordinates = new Point(125,0);
        YELLOW_RED.coordinates = new Point(125,-125);
        RED_BLUE.coordinates = new Point(0,-125);
        YELLOW_BLUE.coordinates = new Point(-125,-125);
        GREEN_BLUE.coordinates = new Point(-125,0);
        BLUE_YELLOW.coordinates = new Point(-125,125);
    }


    @Override
    public String toString() {
         return super.toString() + " at x: " + egocentricCoordinates.x + " y: " + egocentricCoordinates.y;
    }
}
