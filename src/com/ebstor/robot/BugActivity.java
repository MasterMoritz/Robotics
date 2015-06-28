package com.ebstor.robot;

import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.TextView;

import com.ebstor.robot.corefunctions.Location;
import com.ebstor.robot.corefunctions.Robot;
import com.ebstor.robot.R;

/**
 * Created by johannes on 3/24/15.
 */
public class BugActivity extends MainActivity {

    private EditText x_coordinate, y_coordinate;
    private TextView textLog;
    private static final String TAG = "BugActivity";
    
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_bug);
        x_coordinate = (EditText) findViewById(R.id.x_coordinate);
        y_coordinate = (EditText) findViewById(R.id.y_coordinate);
        textLog = (TextView) findViewById(R.id.bugTextLog);
        robot.connect();
    }

    public void origin(View v) {
        robot.robotLocation = new Location(0, 0, 0);
    }

    public void startBug(View v) {
    	//set goal location
        int x = Integer.valueOf(x_coordinate.getText().toString());
        int y = Integer.valueOf(y_coordinate.getText().toString());
        Location goal = new Location(x,y);
        robot.setGoal(goal);
        Log.i(TAG, "goal coordinates: " + goal);
        
        //initialize closest point to goal on mline
        robot.m_point = new Location(robot.robotLocation);
        
        //start m-line algorithm
        double distanceToGoal;
        while (!robot.reachedGoal()){
        	distanceToGoal = Robot.euclideanDistance(robot.robotLocation, robot.goal);
	        robot.turnToGoal();
	        if(robot.driveUntilObstacle(distanceToGoal)) {
		        //circle around obstacle counterclockwise until mline is hit
		        robot.followObstacle(-1);
	        }
        }
        
        //drive/turn to reach the specified pose (x, y, theta)
        distanceToGoal = Robot.euclideanDistance(robot.robotLocation, robot.goal);
        robot.turnToGoal();
        robot.drive(distanceToGoal);
        Log.v(TAG, "robot reached goal \n RobotLocation: " + robot.robotLocation + "\n GoalLocation" + robot.goal);
        robot.turn(- (robot.robotLocation.getTheta() - robot.goal.getTheta()));
        Log.v(TAG, "final pose: " + robot.robotLocation);
    }

    public void testTurnToGoal(View v) {
        int x = Integer.valueOf(x_coordinate.getText().toString());
        int y = Integer.valueOf(y_coordinate.getText().toString());
        Location goal = new Location(x,y);
        robot.setGoal(goal);
        robot.turnToGoal();

    }
}