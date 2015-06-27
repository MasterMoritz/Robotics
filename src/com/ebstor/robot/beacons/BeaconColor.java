package com.ebstor.robot.beacons;

import org.opencv.core.Scalar;

import java.util.EnumMap;
import java.util.Map;

/**
 * Created by johannes on 5/15/15.
 */
public enum BeaconColor {
    RED,
    BLUE,
    GREEN,
    YELLOW;

    public static Map<BeaconColor,Scalar> hsvColors = new EnumMap<>(BeaconColor.class);

    static {
        /*hsvColors.put(RED, new Scalar(2,255,140));
        hsvColors.put(BLUE, new Scalar(138,255,95));
        hsvColors.put(GREEN, new Scalar(215,144,95));
        hsvColors.put(YELLOW, new Scalar(45,255,0));*/
    }

    public Scalar hsvColor() {
        return hsvColors.get(this);
    }

    public void setHsvColor(Scalar hsvColor) {
        hsvColors.put(this,hsvColor);
    }
}
