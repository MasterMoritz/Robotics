package com.ebstor.robot;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;
import com.ebstor.robot.corefunctions.Robot;
import com.ebstor.robot.R;

import static java.lang.Thread.sleep;

/**
 * Created by johannes on 3/23/15.
 */
public class CalibrateActivity extends MainActivity {
    /**
     * in ms
     */
    private final int CALIBRATION_DRIVE = 1092;
    /**
     * in ms
      */
    private final int CALIBRATION_TURN = 1092;
    private EditText realValue;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_calibrate);
        realValue = (EditText) findViewById(R.id.realvalue);
        robot.connect();
    }

    public void translationTestRun(View v) {
    	if (robot != null) {
	        robot.drive();
	        try {
	            sleep(CALIBRATION_DRIVE-50);
	        } catch (Exception e) {}
	        robot.stop();
    	}
    }

    public void rotationTestRun(View v) {
        robot.turnLeft();
        try {
            sleep(CALIBRATION_TURN-50);
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