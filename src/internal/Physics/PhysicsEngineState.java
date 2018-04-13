package internal.Physics;

import internal.Helper.Vector;

/**
 * Created by Martijn on 7/11/2017.
 * A class of physics engine states, used for configuring the drone for the next state
 */
public interface PhysicsEngineState {

    /**
     * Get the position output of the physics engine
     * @return the next position calculated by the engine
     */
    Vector getPosition();

    /**
     * Get the velocity output of the physics engine
     * @return the next velocity calculated by the engine
     */
    Vector getVelocity();

    /**
     * Get the orientation output of the physics engine
     * @return the next orientation calculated by the engine
     */
    Vector getOrientation();

    /**
     * Get the rotation output of the physics engine
     * @return the next rotation calculated by the engine
     */
    Vector getRotation();

    /**
     * Getter for the front tyre delta of the associated chassis
     * @return the front tyre delta
     */
    float getFrontTyreDelta();

    /**
     * getter for the rear left tyre delta of the associated chassis
     * @return the rear left tyre delta
     */
    float getRearLeftTyreDelta();

    /**
     * Getter for the rear right tyre delta of the associated chassis
     * @return the rear right tyre delta
     */
    float getRearRightTyreDelta();
}
