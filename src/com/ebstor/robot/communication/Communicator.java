package com.ebstor.robot.communication;

import android.widget.TextView;
import com.ebstor.robot.controller.OdometryUpdater;
import com.ebstor.robot.controller.RobotAction;
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
    private static OdometryUpdater odometry;
    /**
     * system time when the last command has been send
      */
    private static long commandTime;

    public Communicator(FTDriver driver, TextView textLog, OdometryUpdater odometryUpdater) {
        this.driver = driver;
        this.textLog = textLog;
        this.odometry = odometryUpdater;
        commandTime = System.currentTimeMillis();
    }

    private long timeSinceLastCommand() {
        return System.currentTimeMillis() - commandTime;
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
            long dt;
            try {
                if ((dt = timeSinceLastCommand()) <= WAIT_BUFFER)
                    sleep(WAIT_BUFFER-dt);

                switch(data[0]) {
                    case 'i':
                        if (data[1] < 0 && data[2] > 0)
                            odometry.setAction(RobotAction.TURN_LEFT);
                        else if (data[1] > 0 && data[2] < 0)
                            odometry.setAction(RobotAction.TURN_RIGHT);
                        else if (data[1] < 0 && data[2] < 0)
                            odometry.setAction(RobotAction.MOVE_BACKWARD);
                        else if (data[1] != 0 && data[2] != 0)
                            odometry.setAction(RobotAction.MOVE_FORWARD);
                        else // this should not happen
                            odometry.setAction(null);
                        break;
                    case 's':
                        odometry.setAction(null);
                        break;
                    case 'w':
                        odometry.setAction(RobotAction.MOVE_FORWARD);
                        break;
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            driver.write(data);
            commandTime = System.currentTimeMillis();
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
        write(
                new byte[]{'i', left, right, '\r', '\n'}
        );
    }
    public void setVelocity(int left, int right) {
        write(
                new byte[]{'i', (byte) left, (byte) right, '\r', '\n'}
        );
    }

    public void stop() {
        write(new byte[]{'s', '\r', '\n'});
    }

    public int[] getSensors() {
        try {
            sleep(WAIT_BUFFER);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
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
    	this.textLog.setText(text);
    }
}
