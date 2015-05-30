package com.ebstor.robot.beacons;

import org.opencv.core.Point;

/**
 * Created by johannes on 5/11/15.
 */

public enum Beacon {

    BLUE_RED,
    RED_GREEN, //upper right
    PURPLE_RED,
    GREEN_RED, //lower right
    RED_BLUE,
    GREEN_BLUE, //lower left
    PURPLE_BLUE,
    BLUE_GREEN; //upper left

    public Point coordinates;
    public Point egocentricCoordinates;

    static {
        BLUE_RED.coordinates = new Point(0,125);
        RED_GREEN.coordinates = new Point(125,125);
        PURPLE_RED.coordinates = new Point(125,0);
        GREEN_RED.coordinates = new Point(125,-125);
        RED_BLUE.coordinates = new Point(0,-125);
        GREEN_BLUE.coordinates = new Point(-125,-125);
        PURPLE_BLUE.coordinates = new Point(-125,0);
        BLUE_GREEN.coordinates = new Point(-125,125);
    }



}
