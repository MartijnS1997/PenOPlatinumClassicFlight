package internal.Testbed;

import internal.Helper.Vector;

/**
 * Created by Martijn on 20/02/2018.
 * an interface for passing the state of the drone into the physics engine
 */
public interface DroneState {

    /**
     * Getter for the position of the drone in the world axis system
     */
    Vector getPosition();

    /**
     * Getter for the velocity of the drone in the world axis system
     */
    Vector getVelocity();

    /**
     * Getter for the orientation of the drone (heading pitch and roll)
     */
    Vector getOrientation();

    /**
     * Getter for the rotation of the drone in the world axis system
     */
    Vector getRotation();

    /**
     * Getter for the previous tyre delta of the front tyre
     */
    float getPrevFrontTyreDelta();

    /**
     * Getter for the previous tyre delta of the rear left tyre
     */
    float getPrevRearLeftTyreDelta();

    /**
     * Getter for the previous tyre delta of the rear right tyre
     */
    float getPrevRearRightTyreDelta();


}
