package internal.Physics;

import internal.Testbed.DroneState;
import internal.Helper.Vector;

import static java.lang.Math.*;

/**
 * Created by Martijn on 13/02/2018.
 * A class of tyres, normal Tyres only exert a force in the y-component as an reaction to deltaRadius
 */
//TODO: checker for tyre position
public class TyrePhysX {

    /**
     * Constructor for a Tyre
     * @param tyrePosition the position of the tyre relative to the center of mass of the drone
     * @param tyreRadius the radius of the Tyre
     * @param tyreSlope the slope used in the upward force (tyre delta dependence)
     * @param dampSlope the slope used in the upward force (time derivative of the tyre delta)
     * @param maxBrake the maximum brake force that can be exerted on the tyre
     * @param maxFricCoeff the maximal friction coëfficient of the tyre used in lateral force compensation
     */
    public TyrePhysX(ChassisPhysX associatedChassisPhysics, Vector tyrePosition, float tyreRadius, float tyreSlope, float dampSlope, float maxBrake, float maxFricCoeff) {
        this.tyrePosition = tyrePosition;
        this.tyreRadius = tyreRadius;
        this.tyreSlope = tyreSlope;
        this.dampSlope = dampSlope;
        this.maxBrake = maxBrake;
        this.maxFricCoeff = maxFricCoeff;
        this.associatedChassisPhysics = associatedChassisPhysics;
    }

    /**
     * returns the net force exerted by the tyre: the force exerted by the deltaRadius and the brakes in world axis system
     * @param state  the state of the drone at the moment of invoking the function
     * @param brakeForce the force exerted by the brakes
     * @param prevTyreDelta the previous tyre compression
     * @param deltaTime the time that has passed between two simulation steps
     * @return the net forces exerted by the tyres in the world axis system
     */
    public Vector getNetForceTyre(DroneState  state, Vector nonChassisForces, float brakeForce, float deltaTime, float prevTyreDelta){

        Vector orientation = state.getOrientation();
        Vector rotation = state.getRotation();
        Vector position = state.getPosition();
        Vector velocity = state.getVelocity();
        float groundDist = this.getTyreDistanceToGround(orientation, position);
        //if the tyre doesn't touch the ground, the exerted force is zero
        if(groundDist >= this.getTyreRadius()){
            return new Vector();
        }
        //not so easy case:

        //first calculate the normal force:
        Vector verticalForce = this.getNormalForce(orientation, position, deltaTime, prevTyreDelta); //naming for consistency

        //then get the brake force
        Vector brakes = this.getBrakeForce(orientation, rotation, velocity, brakeForce, nonChassisForces, deltaTime);

        // the resulting force on the tyres
        return brakes.vectorSum(verticalForce);

    }

    /**
     * Calculates the netto moment on the drone
     * @param state the state of the drone at the moment of invoking the method
     * @param netForce the net force exerted on the drone
     * @return the net moment in the drone axis system
     */
    public Vector getNetMomentTyre(DroneState state, Vector netForce){
        Vector orientation = state.getOrientation();
        Vector position = state.getPosition();
        float groundDist = this.getTyreDistanceToGround(orientation, position);
        //first check if we touch the ground, if not return null vector
        if(groundDist > this.getTyreRadius()){
            //System.out.println("not touching ground");
            return new Vector();
        }
        //first calculate the force arm
        Vector relPosAxleDrone = this.getTyrePosition();
        float currentTyreRadius = this.getTyreRadius() - this.calcRadiusDelta(orientation, position);
        Vector relPosTyreBottomDrone = relPosAxleDrone.vectorSum(new Vector(0f, currentTyreRadius, 0f)); //given in the drone axis system

        //all the forces apply to the bottom of the tyre
        //1. transform the net forces to the drone axis system
        Vector netForceDrone = PhysXEngine.worldOnDrone(netForce, orientation);
        //System.out.println("Tyre position: " + relPosTyreBottomDrone);
        return relPosTyreBottomDrone.crossProduct(netForceDrone);
    }

    /**
     * Calculates the brake force exerted on the wheels of the drone in the world axis system
     * @param orientation the orientation of the drone
     * @param velocity the velocity of the drone
     * @param brakeForce the brake force exerted on the wheels (abs value)
     * @param nonChassisForces  the forces acting on the chassis that are not
     * @return  zero vector if the cannot be a brake force exerted (@see canExertBrakeForce)
     *          the brakeForce set in the opposite direction of the velocity in the drone axis system
     */
    private Vector getBrakeForce(Vector orientation, Vector rotation, Vector velocity, float brakeForce, Vector nonChassisForces, float deltaTime){
        //calculate the brake force in the world axis system
        //get the sign of the velocity component amongst the z-axis of the drone
        Vector absVelocity = this.absoluteVelocityWorld(orientation, rotation, velocity);
//        System.out.println("deltaTime: " + deltaTime);
//        System.out.println("absolute velocity : " + absVelocity);
        Vector velocityDrone = PhysXEngine.worldOnDrone(absVelocity, orientation);
//        System.out.println("Absolute velocity drone: " + absVelocity);
        //transform the net forces to the drone axis system
        Vector nonChassisForcesDrone = PhysXEngine.worldOnDrone(nonChassisForces, orientation);
//        System.out.println("Non chassis forces: " + nonChassisForces);
        float neededBrakeForce = this.calcNeededBrakeForce(velocityDrone, nonChassisForcesDrone, deltaTime/* *10*/);

        //check if we can exert the force
        float exerted = signum(neededBrakeForce) * min(abs(neededBrakeForce), brakeForce);
//        System.out.println("needed brake force: " + neededBrakeForce);
//        System.out.println("exerted brake force: " + exerted);
//        System.out.println("brake force: " + brakeForce);
        //now create a vector for the brake force
        Vector brakeForceDrone = new Vector(0,0,exerted);
        //transform it to the world axis system
        return PhysXEngine.droneOnWorld(brakeForceDrone, orientation);
        //check if we may brake
//        if(canExertBrakeForce(velocityDrone)){
//            return new Vector(); // if not, return zero vector
//        }

//        //get the sign
//        float zVelSign = signum(velocityDrone.getzValue());
//
//        //if the velocity is zero, we must counteract only the forces exerted on the chassis
//        if(velocityDrone.getzValue() == 0){
//            //brake force == z-component of non chassis forces (or max if not high enough)
//            float exertedBrakeForce = min(abs(nonChassiForcesDroneZ), brakeForce);
//            //set the signum and create the vector
//            Vector netBrakeForce = new Vector(0,0, -signum(nonChassiForcesDroneZ)*exertedBrakeForce);
//            return PhysXEngine.droneOnWorld(netBrakeForce, orientation);
//        }
//
//        //if we may brake, set the brake force opposite to the sign of the velocity in the drone axis system
//        Vector brakeForceDrone =  new Vector(0,0, - min(abs(brakeForce), this.getMaxBrake())*zVelSign);
//        System.out.println("brake force: " + brakeForceDrone);
//
//        //then transform the brake force to the world axis system
//        //all the y-components of the brake force need to be set to zero (can be a result of the transformation)
//        Vector tBrakeForceDrone= PhysXEngine.droneOnWorld(brakeForceDrone, orientation);
//        System.out.println("transformed brake force: " + tBrakeForceDrone);
//        return new Vector(tBrakeForceDrone.getxValue(), 0f, tBrakeForceDrone.getzValue());

    }

    /**
     * @param tyreVelocityDrone the absolute velocity of the tyre in the drone axis system
     * @param nonChassisForceDrone the non chassis forces exerted on the tyre (already scaled) transformed to the drone axis system
     * @param deltaTime the time difference between two steps
     * @return the needed brake force, the sign is already set right
     */
    private float calcNeededBrakeForce(Vector tyreVelocityDrone, Vector nonChassisForceDrone, float deltaTime){
        //calculate the exerted brake force based on the z-components of the variables
        //first we need the total mass of the drone
        float totalMass = this.getAssociatedChassisPhysics().getAssociatedPhysicsEngine().getTotalMass();
        //get the z-component of the velocity
        //check if the velocity is lower than 0.01, if so, ignore
        float tyreVelocityZ = abs(tyreVelocityDrone.getzValue()) >= 0.001 ? tyreVelocityDrone.getzValue() : 0;
        //take the z-component of the chassis force
        float nonChassisZ = nonChassisForceDrone.getzValue();
        //calculate the required brake force to stop the drone
        float velocityComponent = tyreVelocityZ*totalMass/deltaTime;
        return  abs(velocityComponent + nonChassisZ) >= 1 ? - (velocityComponent + nonChassisZ) : 0;
    }

    /**
     * Calculates the absolute velocity of the given tyre in the world axis system
     * @param velocity the velocity of the drone in the world axis system
     * @param rotation the rotation vector in the world axis system
     * @param orientation the orientation of the drone
     * @return the absolute velocity of the tyre, calculated in the world axis system
     */
    private Vector absoluteVelocityWorld(Vector orientation, Vector rotation, Vector velocity){
        Vector relTyrePos = this.getTyrePosition();
        Vector worldRelTyrePos = PhysXEngine.droneOnWorld(relTyrePos, orientation);
        //now multiply the rotation vector with the force arm
        Vector rotationVelocity = rotation.crossProduct(worldRelTyrePos);
        // add the world axis velocity and the rotation together
        return velocity.vectorSum(rotationVelocity);

    }

    /**
     * Checks if a brake force may be exerted, is only valid if the drone is not stationary
     * @param velocityDrone the velocity of the drone in drone axis system
     * @return true if and only if the velocity amongst the z-axis is nonzero
     * note: may add interval
     */
    private boolean canExertBrakeForce(Vector velocityDrone){
        return Math.abs(velocityDrone.getzValue()) < 0;
    }

    /**
     * Calculates the normal force exerted on the tyre, given in the world axis system
     * @return
     */
    protected Vector getNormalForce(Vector orientation, Vector position, float deltaTime, float prevTyreDelta){
        float tyreDelta = this.calcRadiusDelta(orientation, position);

        if(tyreDelta <= 0){
            return new Vector();
        }
        //calculate the force generated by the tyreslope
        float tyreSlope = this.getTyreSlope();
        float tyreSlopeForce = tyreSlope*tyreDelta;

        //dampSlope force calculation
        float dampSlope = this.getDampSlope();
        float deltaTyreDelta = tyreDelta - prevTyreDelta;
        float deltaDiff = deltaTyreDelta/deltaTime;
        float dampSlopeForce = dampSlope*deltaDiff;
//        System.out.println("Damp slope force: "  + dampSlopeForce);
//        System.out.println("current TyreDelta: " + tyreDelta);
//        System.out.println("prev tyre delta: " + prevTyreDelta);
//        System.out.println("tyreSlope force: " + tyreSlopeForce);

        return new Vector(0f, tyreSlopeForce + dampSlopeForce, 0f);
    }



    /**
     * Calculates the absolute position of the tyre
     * @param orientation the orientation of the drone
     * @param position the position of the drone in the world axis system
     * @return the absolute postion of the tyre
     */
    protected Vector getAbsolutePosition(Vector orientation, Vector position){
        //first get the position of the wheel
        Vector relTyrePosDrone = this.getTyrePosition();
        //then transform it to the world
        Vector relTyrePosWorld = PhysXEngine.droneOnWorld(relTyrePosDrone, orientation);
        //then get the absolute position
        return relTyrePosWorld.vectorSum(position);
    }

    /**
     * Calculates the radius delta, may return a negative delta, means that the tyre is not on the ground
     * @param orientation the orientation of the drone
     * @param position the position of the drone
     * @return the radius delta
     */
    protected float calcRadiusDelta(Vector orientation, Vector position){
        float groundDist = this.getTyreDistanceToGround(orientation, position);
        float tyreRad = this.getTyreRadius();

        if(tyreRad - groundDist <= 0){
            return 0f;
        }else{
            return tyreRad - groundDist;
        }
    }

    /**
     * Calculates the distance from the center of the tyre to the ground seen parallel from the orientation
     * of the tyre
     * @param orientation the orientation of the drone
     * @param position position of the drone
     * @return the distance to the ground
     */
    protected float getTyreDistanceToGround(Vector orientation, Vector position){

        //calculate the position of the center of the tyre in the world axis system
        Vector tyrePos = this.getTyrePosition();
        //transform to world axis
        tyrePos = PhysXEngine.droneOnWorld(tyrePos, orientation);
        //calculate the abs pos:
        Vector worldTyrePos = tyrePos.vectorSum(position);
        //get the height of the center coord of the tyre
        float centerHeight = worldTyrePos.getyValue();
        //System.out.println("Drone Height: " + position.getyValue());
        //System.out.println("Axle height: " + centerHeight);
        //get the roll of the plane
        float roll = orientation.getzValue();

        //System.out.println("Distance to ground: " + centerHeight/cos(roll));
        //calculate the distance from the center of the tyre to the ground parallel to the tyre orientation
        return (float) (centerHeight/cos(roll));
    }

    /**
     * returns true if and only if the centre of the tyre goes below the y = 0 mark (the axle touches the ground, not so good)
     * @param orientation the orientation of the drone
     * @param position the position of the drone
     * @return true if the axle touches the ground
     */
    public boolean checkCrash(Vector orientation, Vector position){
        Vector relTyrePosDrone = this.getTyrePosition();
        Vector relTyrePosWorld = PhysXEngine.droneOnWorld(relTyrePosDrone, orientation);
        Vector absPos = position.vectorSum(relTyrePosWorld);
        return absPos.getyValue() <= 0;
    }

    /**
     * Getter for the tyre position relative to the drone mass center
     * @return tyre position (drone axis system)
     */
    protected Vector getTyrePosition() {
        return tyrePosition;
    }

    /**
     * Getter for the tyre radius
     * @return the radius of said tyre
     */
    protected float getTyreRadius() {
        return tyreRadius;
    }

    /**
     * Getter for the tyre slope of the tyre (relation to the tyre Delta)
     * @return the tyre slope
     */
    protected float getTyreSlope() {
        return tyreSlope;
    }

    /**
     * Getter for the damp slope of the tyre (relation to the time derivative of the tyre delta)
     * @return
     */
    protected float getDampSlope() {
        return dampSlope;
    }

    /**
     * Getter for the maximal brake force that can be exerted onto the brakes of the tyre
     * @return the maximal brake force
     */
    protected float getMaxBrake() {
        return maxBrake;
    }

    /**
     * Getter for the maximal friction coefficient used to compensate for lateral drift during taxiing and landing
     * @return the maximal friction coefficient
     */
    protected float getMaxFricCoeff() {
        return maxFricCoeff;
    }

    public ChassisPhysX getAssociatedChassisPhysics() {
        return associatedChassisPhysics;
    }

    /**
     * Variable that stores the position of the tyre in the drone axis system
     */
    private Vector tyrePosition;

    /**
     * Variable that stores the radius of the tyre
     */
    private float tyreRadius;

    /**
     * Variable that stores the tyre slope, used to calculate the normal force, linear with tyre delta
     */
    private float tyreSlope;

    /**
     * Variable that stores the damp slope, used to calculate the normal force, linear with time derivative of tyre delta
     */
    private float dampSlope;

    /**
     * Variable that stores the maximum brake force for the tyre
     */
    private float maxBrake;

    /**
     * Variable that stores the maximal friction coefficient of the tyre for compensating lateral drift
     */
    private float maxFricCoeff;

    private ChassisPhysX associatedChassisPhysics;


}
