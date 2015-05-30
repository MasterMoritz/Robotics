package com.ebstor.robot.corefunctions;

/**
 * Created by johannes on 3/24/15.
 */
public class Location {

    private double x;
    private double y;
    private double theta;

    public Location() {
        x = 0; y = 0; theta = 0;
    }

    public Location(double x, double y) {
        this.x = x;
        this.y = y;
        theta = 0;
    }

    public Location(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
    
    public void translate(double dist_cm) {
        x += Math.cos(Math.toRadians(theta)) * dist_cm;
        y += Math.sin(Math.toRadians(theta)) * dist_cm;
    }

    public void rotate(double degrees) {
        theta = (theta + degrees) %360;
        if (theta < 0) theta = 360 + theta;
    }


    public double getTheta() {
        return theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public void setX(double x) {
    	this.x = x;
    }
    public void setY(double y) {
		this.y = y;
	}
    public void setTheta(double theta) {
        theta %= 360;
        if (theta < 0) theta = 360 + theta;
        this.theta = theta;
	}
    
   
    public Location(Location l) {
    	this.x = l.getX();
    	this.y = l.getY();
    	this.theta = l.getTheta();
    }
    @Override
    public String toString() {
    	return ("x = " + x + " | y = " + y + " | theta = " + theta);
    }
}
