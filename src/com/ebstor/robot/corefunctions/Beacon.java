package com.ebstor.robot.corefunctions;

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
    public Scalar[] hsvColor;

    static {
        BLUE_RED.coordinates = new Point(0,125);
        RED_BLACK.coordinates = new Point(125,125);
        PURPLE_RED.coordinates = new Point(125,0);
        BLACK_RED.coordinates = new Point(125,-125);
        RED_BLUE.coordinates = new Point(0,-125);
        BLACK_BLUE.coordinates = new Point(-125,-125);
        PURPLE_BLUE.coordinates = new Point(-125,0);
        BLUE_BLACK.coordinates = new Point(-125,125);

        Scalar red = new Scalar(2,255,140);
        Scalar blue = new Scalar(138,255,95);
        Scalar purple = new Scalar(215,144,95);
        Scalar black = new Scalar(45,255,0);

        BLUE_RED.hsvColor = new Scalar[]{blue, red};
        RED_BLACK.hsvColor = new Scalar[]{red, black};
        PURPLE_RED.hsvColor = new Scalar[]{purple, red};
        BLACK_RED.hsvColor = new Scalar[]{black, red};
        RED_BLUE.hsvColor = new Scalar[]{red, blue};
        BLACK_BLUE.hsvColor = new Scalar[]{black, blue};
        PURPLE_BLUE.hsvColor = new Scalar[]{purple, blue};
        BLUE_BLACK.hsvColor = new Scalar[]{blue, black};
    }

}
