package com.ebstor.robot;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;
import com.ebstor.robot.corefunctions.Location;
import com.example.robot.R;

/**
 * Created by johannes on 3/24/15.
 */
public class BugActivity extends MainActivity {

    private EditText x_coordinate, y_coordinate;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_bug);
        x_coordinate = (EditText) findViewById(R.id.x_coordinate);
        y_coordinate = (EditText) findViewById(R.id.y_coordinate);
        MainActivity.robot.connect();
    }

    public void origin(View v) {
        robot.robotLocation = new Location();
    }

    public void startBug(View v) {
        int x = Integer.valueOf(x_coordinate.getText().toString());
        int y = Integer.valueOf(y_coordinate.getText().toString());
        Location goal = new Location(x,y);
        robot.bug2(goal);
    }
}