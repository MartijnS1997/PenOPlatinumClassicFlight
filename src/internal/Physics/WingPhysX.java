package internal.Physics;

import internal.Exceptions.AngleOfAttackException;
import internal.Helper.Vector;

import static java.lang.Math.PI;

/**
 * Implmented by Martijn (r0623847)
 */

//Todo optimize the calculations by storing and reusing computational heavy variables like absolute velocity
public abstract class WingPhysX {

    /**
     * Public constructor for a wing object
     * @param relativePosition the position relative to the center of gravity of the drone
     * @param mass the mass of the drone
     * @param maximumAngleOfAttack the maximum wing inclination of the drone
     * @param wingInclination the current wing inclination of the drone
     */
    public WingPhysX(Vector relativePosition, float liftSlope, float mass, float maximumAngleOfAttack, float wingInclination){
        if(!canHaveAsRelativePosition(relativePosition))
            throw new IllegalArgumentException(INVALID_POSITION);
        if(!canHaveAsMass(mass))
            throw new IllegalArgumentException(INVALID_MASS);
        if(!canHaveAsMaxAngleOfAttack(maximumAngleOfAttack))
            throw new IllegalArgumentException(INVALID_MAXAOA);
        if(!canHaveAsLiftSlope(liftSlope))
            throw new IllegalArgumentException(INVALID_LIFTSLOPE);

        this.relativePosition = relativePosition;
        this.mass = mass;
        this.maximumAngleOfAttack = maximumAngleOfAttack;
        this.setWingInclination(wingInclination);
        this.liftSlope = liftSlope;
    }

    /*
    Methods needed for physics
     */

    /**
     * Calculates the absolute velocity of the drone in the world axis
     * @return a vector containing the absolute velocity of the wing in the world axis
     * note: in next parts of the project there is need to account for airspeed caused by the wind
     */
    public Vector getAbsoluteVelocity(Vector orientation, Vector rotation, Vector velocity){


        //the relative position vector of the wing to the mass center of the drone
        Vector relativePos = this.getRelativePosition();
        //calculate the absolute position vector of the wing to the center of mass
        Vector radiusVector = PhysXEngine.droneOnWorld(relativePos, orientation);
        //calculate the rotational velocity component of the wing
        Vector rotationVelocity = rotation.crossProduct(radiusVector);

        //sum the velocity of the drone with the angular velocity caused by the rotation
        return velocity.vectorSum(rotationVelocity);

    }

    public abstract Vector getAxisVector();

    /**
     * Calculates the lift of the Airfoil expressed in world axis
     * @param orientation the orientation of the drone (heading, pitch, roll)
     * @param rotation the absolute rotation of the drone (rotation vector)
     * @param velocity the velocity of the center of mass of the drone in the world axis system
     * @return N*liftSlope*AOA*s^2
     */
    public Vector getLift(Vector orientation, Vector rotation, Vector velocity){

        Vector normal = PhysXEngine.droneOnWorld(this.getNormal(), orientation);
        Vector airspeed = this.getAbsoluteVelocity(orientation, rotation,  velocity);
        Vector axisVector = PhysXEngine.droneOnWorld(this.getAxisVector(), orientation);
        Vector projectedAirspeed = airspeed.orthogonalProjection(axisVector);

        float angleOfAttack = this.calcAngleOfAttack(orientation, rotation, velocity);
        float liftSlope = this.getLiftSlope();
        //System.out.println("angle of attack: " + angleOfAttack);


        // calculate s^2
        float airspeedSquared = projectedAirspeed.scalarProduct(projectedAirspeed);

        float scalarPart =  airspeedSquared*angleOfAttack*liftSlope;
        Vector lift = normal.scalarMult(-scalarPart);

        //if the lift on the wing is lower than the threshold, AOA is disabled
        if(lift.getSize() >= AOA_LIFT_THRESHOLD &&  Math.abs(angleOfAttack) >= this.getMaximumAngleOfAttack()) {
            System.out.println("Wing: " + this);
            System.out.println("AOA exception thrown, AOA: " + angleOfAttack);
            System.out.println("Velocity: " + velocity);
            System.out.println("Lift: " + lift);
            System.out.println("####################################################################################");
            //throw new AngleOfAttackException(this);
        }

        return lift;
    }

    /**
     * Returns true if the center of mass of the wing touches the ground
     * @param orientation the orientation of the drone
     * @param position the position of the drone
     * @return true if the wing touches the ground
     */
    public boolean checkCrash(Vector orientation, Vector position){
        Vector relPosDrone = this.getRelativePosition();
        Vector relPosWorld = PhysXEngine.droneOnWorld(relPosDrone, orientation);
        Vector absPos = position.vectorSum(relPosWorld);
        return absPos.getyValue() <= 0;
    }

    /*
    getters, setters and checkers for the drone
     */

    //TODO was originally just the normal in the wing axis sytem, now it is changed
    // to the drone axis system
    /**
     * Calculates the normal vector of the wing in the Drone's coordinate system
     */
    public abstract Vector getNormal();

    /**
     * returns the attackVector
     */
    public abstract Vector getAttackVector();


//    /**
//     * project the given vector on the world axis
//     * @param vector the vector to be projected onto the world
//     * @param orientation the orientation of the drone
//     */
//    public Vector projectOnWorld(Vector vector, Vector orientation){
//        Vector droneVector = this.projectOnDrone(vector);
//        return PhysXEngine.droneOnWorld(droneVector, orientation);
//    }

    /**
     * Getter for the angle of attack
     */
    public float getAngleOfAttack(){
        return this.angleOfAttack;
    }

    /**
     * Calculates the angle of attack and stores it in the designated variable
     * @param orientation the orientation of the drone (heading, pitch, roll)
     * @param rotation the rotation of the drone, given in the world axis system
     * @param velocity the velocity of the center of mass of the drone given in the world axis system
     * @post new angleOfAttack = -atan2(Airspeed*Normal, Airspeed*attackvector)
     */
    public float calcAngleOfAttack(Vector orientation, Vector rotation, Vector velocity){
        //need for the projected version of all the vectors because the airspeed is in the world axis
        Vector airspeed = this.getAbsoluteVelocity(orientation, rotation, velocity);
        Vector normal = PhysXEngine.droneOnWorld(this.getNormal(), orientation);
        Vector axisVector = PhysXEngine.droneOnWorld(this.getAxisVector(), orientation);
        Vector attackVector = PhysXEngine.droneOnWorld(this.getAttackVector(), orientation);


        Vector projectedAirspeed = airspeed.orthogonalProjection(axisVector);//.orthogonalProjection(normal);

        float numerator = projectedAirspeed.scalarProduct(normal);
        float denominator = projectedAirspeed.scalarProduct(attackVector);

        //set the angle of attack anyway, can be used for diagnostics
        float angleOfAttack = (float)Math.atan2(numerator, denominator);
/*        if(Math.abs(angleOfAttack) >= this.getMaximumAngleOfAttack()){
            System.out.println("Airspeed: " + projectedAirspeed);
            System.out.println("numerator: " + numerator);
        }*/
        return angleOfAttack;

    }


    /**
     * Basic getter for the wing inclination
     */
    public float getWingInclination() {
        return wingInclination;
    }


    //Todo add some constraint on the moving of the wings?
    /**
     * Basic setter, if the winginclination is valid, set the inclination to the given one
     * @param wingInclination the desired inclination of the wings
     */
    public void setWingInclination(float wingInclination) {
        if(!this.canHaveAsWingInclintion(wingInclination)){
            throw new IllegalArgumentException(INVALID_INCLINATION);
        }

        this.wingInclination = wingInclination;
    }

    //Todo note: Is this function still needed if we have just throw an error if the maximum AOA is reached?
    //or do we make a seperate internal Max inclination that we adhere to?
    /**
     * Returns true if and only if the inclination is between [0, getMaximumAngleOfAttack()]
     * @param inclination the inclination to be tested
     */
    public boolean canHaveAsWingInclintion(float inclination){

        return true;
    }

    /**
     * Getter for the maximum inclination of the drone
     * @return
     */
    public float getMaximumAngleOfAttack() {
        return maximumAngleOfAttack;
    }

    /**
     * Getter for the mass of the drone
     */
    public float getMass() {
        return mass;
    }

    /**
     * Returns true if and only if the mass is positive
     * @param mass the mass to be tested
     */
    public boolean canHaveAsMass(float mass){
        return  mass>=0;
    }

    /**
     * Getter for the relative position in the axis of the drone
     */
    public Vector getRelativePosition() {
        return relativePosition;
    }

    /**
     * returns true if and only if the wing is located on the x axis xor the negative z axis
     * @param position
     * @return
     */
    public static boolean canHaveAsRelativePosition(Vector position){
        float x_part = position.getxValue();
        float y_part = position.getyValue();
        float z_part = position.getzValue();

        //The wings may not be positioned on the y-axis
        if(y_part!=0.0f)
            return false;

        //The wings can only have a x or a z coordinate
        if(x_part!= 0 && z_part != 0)
            return false;
        //If the wings are positioned on the z-axis, they can only have
        //positive coordinates
        if(x_part == 0 && z_part < 0)
            return false;

        return true;
    }

    @Override
    public String toString() {
        return "WingPhysX{" +
                "wingInclination=" + wingInclination*RAD2DEGREE +
                ", relativePosition=" + relativePosition +
                '}';
    }

    /**
     * Checker for the angle of attack
     * @param maxAOA the maximum angle of attack
     * @return true if and only if the maxAOA is between [0, PI/2)
     */
    public boolean canHaveAsMaxAngleOfAttack(float maxAOA){
        return maxAOA>=0 && maxAOA <= PI/2.0f;
    }

    /**
     * Getter for the lift slope of the wing
     */
    public float getLiftSlope(){
        return this.liftSlope;
    }

    /**
     * Checker for the lift slope, returns true if and only if the lift slope is larger than 0
     */
    public boolean canHaveAsLiftSlope(float liftSlope){
        return liftSlope > 0;
    }

    /**
     * Variable that holds the inclination of the wing
     */
    private float wingInclination;

    /**
     * Variable that holds the maximum inclination of the wing (immutable)
     */
    private float maximumAngleOfAttack;

    /**
     * Variable that holds the mass of the wing (immutable)
     */
    private float mass;

    /**
     * Variable that holds the relative position to the center of mass of the drone (immutable)
     */
    private Vector relativePosition;


    /**
     * Variable that holds the lift slope of the wing (immutable)
     */
    private float liftSlope;

    /**
     * Variable that holds the angle of attack of the drone
     */
    private float angleOfAttack;

    /*
    Exception Strings
     */

    private final static String INVALID_INCLINATION = "The given inclination is invalid.";
    private final static String INVALID_POSITION = "The given position is invalid.";
    private final static String INVALID_MASS = "The given mass is invalid";
    private final static String INVALID_MAXAOA = "The given maximum inclination is invalid";
    private final static String INVALID_DRONE = "The wing is already attached to a drone";
    private final static String INVALID_LIFTSLOPE = "The lift slope is invalid";
    private final static String INVALID_AOA = "The angle of attack has exeded the maximum value";

    /*
    Constants
     */
    private static final float RAD2DEGREE = (float) (180/Math.PI);
    private static final float AOA_LIFT_THRESHOLD = 50.0f;
}
