package com.ebstor.robot.corefunctions;

/**
 * Created by johannes on 3/29/15.
 */
public interface SensorCondition {
    /**
     * important: must check sensors of the robot so that it stops!
     * @return if the condition holds
     */
    boolean holds();
    void init();
}
