package com.ebstor.robot;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;
import android.widget.EditText;
import com.example.robot.R;

/**
 * Created by johannes on 3/23/15.
 */
public class CalibrateActivity extends Activity {

    private final int CALIBRATION_DIST = 95;
    private final int CALIBRATION_DEG = 360;
    private EditText realValue;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_calibrate);
        realValue = (EditText) findViewById(R.id.realvalue);
    }

    public void translationTestRun(View v) {
       // TODO implement time using gyroskop
        MainActivity.robot.drive(CALIBRATION_DIST);
    }

    public void rotationTestRun(View v) {
       // TODO implement time using gyroskop
        MainActivity.robot.turn(90);
        MainActivity.robot.turn(CALIBRATION_DEG-90);
    }

    public void calibrateTranslation(View v) {
        Integer actual = Integer.parseInt(realValue.getText().toString());
        MainActivity.robot.setTRANSLATION_COEFFICIENT(CALIBRATION_DIST/actual);
    }

    public void calibrateRotation(View v) {
        Integer actual = Integer.parseInt(realValue.getText().toString());
        MainActivity.robot.setROTATION_COEFFICIENT(CALIBRATION_DEG/actual);
    }
}