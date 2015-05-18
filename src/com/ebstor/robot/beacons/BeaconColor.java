package com.ebstor.robot.beacons;

import org.opencv.core.Scalar;

/**
 * Created by johannes on 5/15/15.
 */
public enum BeaconColor {
    RED,
    BLUE,
    PURPLE,
    BLACK;

    public Scalar hsvColor;

    static {
        RED.hsvColor = new Scalar(2,255,140);
        BLUE.hsvColor = new Scalar(138,255,95);
        PURPLE.hsvColor = new Scalar(215,144,95);
        BLACK.hsvColor = new Scalar(45,255,0);
    }
}
