package com.ebstor.robot.communication;

import android.widget.TextView;
import jp.ksksue.driver.serial.FTDriver;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Thread.sleep;

/**
 * Created by johannes on 3/30/15.
 */
public class Communicator {

    private FTDriver driver;
    private TextView textLog ;
    private static final long WAIT_BUFFER = 50;

    public Communicator(FTDriver driver, TextView textLog) {
        this.driver = driver;
        this.textLog = textLog;
    }

    public void connect() {
        if (driver.begin(9600)) textLog.setText("connected! ");
        else textLog.setText("connection not okay");
    }

    public void disconnect() {
        driver.end();
        if (!driver.isConnected()) textLog.append("disconnected! ");
    }

    public boolean isConnected() {
    	return driver.isConnected();
    }
    
    public void write(byte[] data) {
        if (driver.isConnected()) {
            try {
                sleep(WAIT_BUFFER);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            driver.write(data);
        } else {
            textLog.append("not connected\n");
        }
    }

    public String read() {
        String s = "";
        int i = 0;
        int n = 0;
        while (i < 3 || n > 0) {
            byte[] buffer = new byte[256];
            n = driver.read(buffer);
            s += new String(buffer, 0, n);
            i++;
        }
        return s;
    }

    public String readWrite(byte[] data) {
        driver.write(data);
        try {
            sleep(100);
        } catch (InterruptedException e) {
            // ignore
        }
        return read();
    }


    public void setVelocity(byte left, byte right) {
        readWrite(
                new byte[]{'i', left, right, '\r', '\n'}
        );
    }
    public void setVelocity(int left, int right) {
        readWrite(
                new byte[]{'i', (byte)left, (byte)right, '\r', '\n'}
        );
    }

    public void stop() {
        write(new byte[]{'s', '\r', '\n'});
    }

    public int[] getSensors() {
        int[] sensors = new int[3];
        
        String[] parsed = readWrite(new byte[] { 'q', '\r', '\n' }).split("\\s+");
		ArrayList<String> hexas = new ArrayList<>();
		
		for (String s : parsed) {
			if (s.charAt(0) == '0') {
				hexas.add(s);
			}
		}
		
		sensors[0] = Integer.decode(hexas.get(2)); //left
		sensors[1] = Integer.decode(hexas.get(4)); //middle
		sensors[2] = Integer.decode(hexas.get(3)); //right
		
        return sensors;
    }

    /**
     *TODO implementation
     * @param sensor : 'l' = left sensor, 'm' = middle sensor, 'r' = right sensor
     * @return the value of the specified sensor
     */
    public int getSpecificSensor(char sensor){
    	return 0;
    }
    
    
    public static int hexaToDecimal(String s){
        String parsed = s.substring(2);
        return Integer.valueOf(parsed, 16);
    }

    public void setTextLog(TextView txlog) {
    	textLog = txlog;
    }
    
    public void setText(String text) {
    	this.textLog.setText(text + "\n");
    }
    
    public void append(String text) {
    	this.textLog.append(text + "\n");
    }
}
