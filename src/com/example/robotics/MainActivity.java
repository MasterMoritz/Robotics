package com.example.robotics;

import jp.ksksue.driver.serial.FTDriver;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.support.v7.app.ActionBarActivity;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

public class MainActivity extends ActionBarActivity {

	private FTDriver com;
	private TextView textLog;
	
	private Button btForward;
	private Button btBackward;
	private Button btConnect;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		// textLog = (TextView) findViewById(R.id.textLog);
		com = new FTDriver((UsbManager) getSystemService(USB_SERVICE));
		connect();
		com.write(new byte[] {'r', '\r', '\n'});
		
		btForward = (Button) findViewById(R.id.btForward);
		btBackward = (Button) findViewById(R.id.btBackward);
		btConnect = (Button) findViewById(R.id.btConnect);
		
		//----BUTTON MOVE FORWARD-----//
	    btForward.setOnClickListener(new View.OnClickListener() {
	    	@Override
	    	public void onClick(View v) {
	    		move((byte)'w', (byte)'1');
	    	}
	    });
	    
	    //----BUTTON MOVE BACKWARD-----//
	    btBackward.setOnClickListener(new View.OnClickListener() {
	    	@Override
	    	public void onClick(View v) {
	    		move((byte)'x', (byte)'1');
	    	}
	    });
	    
	    //----BUTTON CONNECT-----//
	    btConnect.setOnClickListener(new View.OnClickListener() {
	    	@Override
	    	public void onClick(View v) {
	    		if (com.isConnected()) {
	    			disconnect();
	    			btConnect.setText("connect");
	    		}
	    		else {
	    			connect();
	    			btConnect.setText("disconnect");
	    		}
	    	}
	    });
	}

	public void connect() {
		if (com.begin(FTDriver.BAUD9600)) {
			textLog.append("Connected\n");
		} else {
			textLog.append("Not connected\n");
		}
	}

	public void disconnect() {
		com.end();
		if (!com.isConnected()) {
			textLog.append("disconnected\n");
		}
	}

	public void comWrite(byte[] data) {
		if (com.isConnected()) {
			com.write(data);
		} else {
			textLog.append("not connected\n");
		}
	}

	public String comRead() {
		String s = "";
		int i = 0;
		int n = 0;
		while (i < 3 || n > 0) {
			byte[] buffer = new byte[256];
			n = com.read(buffer);
			s += new String(buffer, 0, n);
			i++;
		}
		return s;
	}

	public String comReadWrite(byte[] data) {
		com.write(data);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// ignore
		}
		return comRead();
	}
	


	//direction: w - forward, x - backward
	public void move(byte direction, byte distance_cm) {
		comReadWrite(
			new byte[] { direction, distance_cm, '\r', '\n' }
		);
	}


	//direction: a - left, d - right
	public void turn(byte direction, byte degree) {
		comReadWrite(
			new byte[] { direction, degree, '\r', '\n' }
		);
	}


}
