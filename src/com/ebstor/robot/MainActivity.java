package com.ebstor.robot;

import android.content.Intent;
import android.widget.*;
import com.ebstor.robot.corefunctions.Robot;
import jp.ksksue.driver.serial.FTDriver;

import com.example.robot.R;

import android.app.Activity;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;

public class MainActivity extends Activity {

	private EditText distance;
    private EditText angle;
    public static Robot robot;

    @Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Button powah = (Button) findViewById(R.id.powah);
        robot = new Robot((TextView) findViewById(R.id.textLog),new FTDriver((UsbManager) getSystemService(USB_SERVICE)));
        powah.setOnTouchListener(new OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        robot.drive();
                        break;
                    case MotionEvent.ACTION_UP:
                        robot.stop();
                        break;
                }
                return true;
            }
        });

        distance = (EditText) findViewById(R.id.distance);
        angle = (EditText) findViewById(R.id.angle);
	}

    public void switchConnection(View v) {
        boolean on = ((Switch) v).isChecked();

        if (on) {
            disconnect();
        } else {
            connect();
        }
    }

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		// Handle action bar item clicks here. The action bar will
		// automatically handle clicks on the Home/Up button, so long
		// as you specify a parent activity in AndroidManifest.xml.
		int id = item.getItemId();
		if (id == R.id.action_bug) {
            Intent intent = new Intent(this, BugActivity.class);
            startActivity(intent);
            return true;
		}
		return super.onOptionsItemSelected(item);
	}

    public void startCalibrateActivity(MenuItem item) {
        Intent intent = new Intent(this,CalibrateActivity.class);
        startActivity(intent);
    }

    public void connect() {
        robot.connect();
    }

    public void disconnect() {
        robot.disconnect();
    }

    public void travelDistance(View v) {
        Integer dist = Integer.valueOf(distance.getText().toString());
        robot.drive(dist);

    }

    public void turnAngle(View v) {
        Integer ang = Integer.valueOf(angle.getText().toString());
        robot.turn(ang);
    }


    public void makeASquare(View v) {
        Integer dist = Integer.valueOf(distance.getText().toString());
        for (int i = 0; i < 4; i++) {
            robot.driveAndStopForObstacles(dist);
            robot.turn(90);
        }
    }




}
