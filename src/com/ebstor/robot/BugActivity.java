package com.ebstor.robot;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;
import android.widget.TextView;

import com.ebstor.robot.corefunctions.Location;
import com.ebstor.robot.corefunctions.SensorCondition;
import com.example.robot.R;

/**
 * Created by johannes on 3/24/15.
 */
public class BugActivity extends MainActivity {

    private EditText x_coordinate, y_coordinate;
    private TextView textLog;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_bug);
        x_coordinate = (EditText) findViewById(R.id.x_coordinate);
        y_coordinate = (EditText) findViewById(R.id.y_coordinate);
        textLog = (TextView) findViewById(R.id.bugTextLog);
        robot.com.setTextLog(textLog);
        robot.connect();
    }

    public void origin(View v) {
        robot.robotLocation = new Location(0, 0, 0);
    }

    public void startBug(View v) {
        int x = Integer.valueOf(x_coordinate.getText().toString());
        int y = Integer.valueOf(y_coordinate.getText().toString());
        Location goal = new Location(x,y);
        robot.setGoal(goal);
        robot.m_point = new Location(robot.robotLocation);
        double distanceToGoal;
        //robot.bug2();
       
        System.out.println("goal coordinates: " + goal);
        while (!robot.reachedGoal()){
        	distanceToGoal = robot.euclideanDistance(robot.robotLocation, robot.goal);
	        robot.turnToGoal();
	        if(robot.driveUntilObstacle(distanceToGoal)) {
		        //circle around obstacle counterclockwise until mline is hit
		        robot.followObstacle(-1);
	        }
        }
        distanceToGoal = robot.euclideanDistance(robot.robotLocation, robot.goal);
        robot.turnToGoal();
        robot.drive(distanceToGoal);
        System.out.println("robot reached goal \n RobotLocation: " + robot.robotLocation + "\n GoalLocation" + robot.goal);
        
        //turn to goal theta
        robot.turn(- (robot.robotLocation.getTheta() - robot.goal.getTheta()));
        System.out.println("final pose: " + robot.robotLocation);
    }

    public void testTurnToGoal(View v) {
        int x = Integer.valueOf(x_coordinate.getText().toString());
        int y = Integer.valueOf(y_coordinate.getText().toString());
        Location goal = new Location(x,y);
        robot.setGoal(goal);
        robot.turnToGoal();

    }
}