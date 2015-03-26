package com.ebstor.robot.robotcommunication;

/**
 * Created by johannes on 3/24/15.
 */
public class Location {

    private int x;
    private int y;
    private int theta;

    public Location() {
        x = 0; y = 0; theta = 0;
    }

    public Location(int x, int y) {
        this.x = x;
        this.y = y;
        theta = 0;
    }

    public void translate(int dist_cm) {
        x += (int)Math.cos(Math.toRadians(theta)) * dist_cm;
        y += (int)Math.sin(Math.toRadians(theta)) * dist_cm;
    }

    public void rotate(int degrees) {
        theta = (theta + degrees) %360;
    }


    public int getTheta() {
        return theta;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

}
