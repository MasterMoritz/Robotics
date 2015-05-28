package com.ebstor.robot;

/**
 * Created by johannes on 5/28/15.
 */
public enum State {
    /**
     * find 2 beacons and localize robot as soon as found
     */
    LOCALIZE,
    /**
     * try to find 2 beacons and localize robot if found
     */
    TRY_LOCALIZE,
    /**
     * look for a ball
     */
    SEARCH_BALL,
    /**
     * navigate to a target ball (includes recalculating path)
     */
    GOTO_BALL,
    /**
     * cage ball that is in front
     */
    CAGE_BALL,
    /**
     * move a caged ball to a target
     */
    BALL_TO_TARGET,
    /**
     * drop off a ball
     */
    DROP_BALL,
    /**
     * final state
     */
    FIN
}

