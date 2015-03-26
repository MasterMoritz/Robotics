package com.ebstor.robot;

import android.content.Intent;
import com.ebstor.robot.robotcommunication.Robot;
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
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import static java.lang.Thread.sleep;

public class MainActivity extends Activity {


	private Button powah;
	private Button connect;
	private Button disconnect;
    private Button distanceButton;
    private Button angleButton;
    private Button squareButton;
	private EditText distance;
    private EditText angle;
    private TextView sensor;
    public static Robot robot;

    @Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
		powah = (Button) findViewById(R.id.powah);
		connect = (Button) findViewById(R.id.connect);
		disconnect = (Button) findViewById(R.id.disconnect);
        robot = new Robot((TextView) findViewById(R.id.textLog),new FTDriver((UsbManager) getSystemService(USB_SERVICE)));
        powah.setOnTouchListener(new OnTouchListener() {
			
			@Override
			public boolean onTouch(View v, MotionEvent event) {
					switch (event.getAction()) {
					case MotionEvent.ACTION_DOWN:
						robot.moveForward();
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
		distanceButton = (Button) findViewById(R.id.distancebutton);
        angleButton = (Button) findViewById(R.id.angleButton);
        squareButton = (Button) findViewById(R.id.makeASqaure);
        sensor = (TextView) findViewById(R.id.sensor);
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

    public void connect(View v) {
        robot.connect();
    }

    public void disconnect(View v) {
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
