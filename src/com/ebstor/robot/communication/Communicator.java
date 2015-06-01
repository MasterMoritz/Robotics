package com.ebstor.robot.communication;

import android.util.Log;
import android.widget.TextView;
import jp.ksksue.driver.serial.FTDriver;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import org.apache.http.auth.BasicUserPrincipal;

import com.ebstor.robot.corefunctions.Quadruple;

import static java.lang.Thread.sleep;

/**
 * Created by johannes on 3/30/15.
 */
public class Communicator {

    private static final String TAG = "Communicator";
    public static LinkedList<Quadruple> leash = new LinkedList<>();
    private FTDriver driver;
    private static final long WAIT_BUFFER = 50;

    public Communicator(FTDriver driver) {
        this.driver = driver;
    }

    public void connect() {
        if (driver.begin(9600)) Log.i(TAG,"connected! ");
        else Log.e(TAG,"connection not okay");
    }

    public void disconnect() {
        driver.end();
        if (!driver.isConnected()) Log.i(TAG,"disconnected! ");
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
            Log.e(TAG,"not connected");
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

    public void setBar(byte value) {
        write(new byte[] { 'o', value, '\r', '\n' });
    }

    public void setBarDown() {
        setBar((byte)0);
        for (int i = 0; i < 10; i++) lowerBar();
    }

    public void setBarUp() {
        setBar((byte)255);
    }

    public void setVelocity(byte left, byte right) {
        write(
                new byte[]{'i', left, right, '\r', '\n'}
        );
    }
    public void setVelocity(int left, int right) {
        write(
                new byte[]{'i', (byte)left, (byte)right, '\r', '\n'}
        );
    }

    public void lowerBar(){
    	write(
    			new byte[] {'-', '\r', '\n'}
    	);
    }
    
    public void raiseBar(){
    	write(
    			new byte[] {'+', '\r', '\n'}
    	);
    }
    
    public void stop() {
        write(new byte[]{'s', '\r', '\n'});
    }

    public int[] getSensors() {
    	try{sleep(WAIT_BUFFER);}catch(Exception e){};
        int[] sensors = new int[3];
        
        String[] parsed = readWrite(new byte[] { 'q', '\r', '\n' }).split("\\s+");
		ArrayList<String> hexas = new ArrayList<>();
		
		for (String s : parsed) {
			if (s.charAt(0) == '0') {
				hexas.add(s);
			}
		}
		
		if (hexas.size() < 5) {
			sensors = getSensors();
		}
		else {
			sensors[0] = Integer.decode(hexas.get(2)); //left
			sensors[1] = Integer.decode(hexas.get(4)); //middle
			sensors[2] = Integer.decode(hexas.get(3)); //right
		}
	
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
    public void leash() throws InterruptedException {
    	for (Quadruple q : leash) {
    		setVelocity(q.velocity1, q.velocity2);
            Thread.sleep(q.time);
            stop();
    	}
    }

}
