package com.ebstor.robot;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;
import com.ebstor.robot.corefunctions.Robot;
import com.example.robot.R;

import static java.lang.Thread.sleep;

/**
 * Created by johannes on 3/23/15.
 */
public class CalibrateActivity extends MainActivity {
    /**
     * in ms
     */
    private final int CALIBRATION_DRIVE = 5000;
    /**
     * in ms
      */
    private final int CALIBRATION_TURN = 5000;
    private EditText realValue;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_calibrate);
        realValue = (EditText) findViewById(R.id.realvalue);
    }

    public void translationTestRun(View v) {
    	MainActivity.robot.connect();
    	if (MainActivity.robot != null) {
	        MainActivity.robot.drive();
	        try {
	            sleep(CALIBRATION_DRIVE);
	        } catch (Exception e) {}
	        MainActivity.robot.stop();
    	}
    }

    public void rotationTestRun(View v) {
        robot.turnLeft();
        try {
            sleep(CALIBRATION_TURN);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.stop();
    }

    public void calibrateTranslation(View v) {
        Integer actual = Integer.parseInt(realValue.getText().toString());
        Robot.CM_PER_MILLISECOND = actual/CALIBRATION_DRIVE;
    }

    public void calibrateRotation(View v) {
        Integer actual = Integer.parseInt(realValue.getText().toString());
        Robot.DEGREE_PER_MILLISECOND = actual/CALIBRATION_TURN;
    }
}