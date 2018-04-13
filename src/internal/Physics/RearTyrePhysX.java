package internal.Physics;

import internal.Testbed.DroneState;
import internal.Helper.Vector;

/**
 * Created by Martijn on 20/02/2018.
 */
public class RearTyrePhysX extends TyrePhysX {
    public RearTyrePhysX(ChassisPhysX chassisPhysX, Vector tyrePosition, float tyreRadius, float tyreSlope, float dampSlope, float maxBrake, float maxFricCoeff){
        super(chassisPhysX, tyrePosition, tyreRadius, tyreSlope, dampSlope, maxBrake, maxFricCoeff);
    }

    @Override
    public Vector getNetForceTyre(DroneState state, Vector nonChassisForces, float brakeForce, float deltaTime, float prevTyreDelta) {
        Vector base =  super.getNetForceTyre(state, nonChassisForces, brakeForce, deltaTime, prevTyreDelta);
        Vector lateralForce = this.getLateralForce(state, deltaTime, prevTyreDelta);
        return base.vectorSum(lateralForce);
    }

    /**
     * Calculates the lateral force exerted on the tyre
     * @param state the state of the drone at instance of invocation
     * @param deltaTime the time passed
     * @return the lateral force exerted on the tyre
     */
    public Vector getLateralForce(DroneState state, float deltaTime, float prevTyreDelta) {

        Vector orientation = state.getOrientation();
        Vector position = state.getPosition();
        //first get the x component in the drone axis system of the velocity
        //1. get the velocity
        Vector velocityWorld = this.getAbsoluteVelocity(state);
        //2. transform the world coordinates to drone coordinates
        Vector velocityDrone = PhysXEngine.worldOnDrone(velocityWorld, orientation);
        //3. calculate the normal force of the wheel
        Vector normalForce = this.getNormalForce(orientation, position, deltaTime, prevTyreDelta);

        float xComponent = velocityDrone.getxValue();
        float fcMAx = this.getMaxFricCoeff();
        float normalForceScalar = normalForce.getSize();

        float lateralForceScalar = xComponent * -fcMAx * normalForceScalar;
        Vector forceDirection = ProjectXAxisOnWorldXZPLane(orientation);

        return forceDirection.scalarMult(lateralForceScalar);

    }

    /**
     * Returns the x-axis of the drone transformed to the world axis system, projected onto the
     * xz-plane of the world and normalized
     * @param orientation the orientation of the drone
     */
    private Vector ProjectXAxisOnWorldXZPLane(Vector orientation) {
        //now project the force
        //1. define the x-axis in the drone axis system
        Vector xAxisDrone = new Vector(1f, 0f, 0f);

        //2. define the yAxis of the world
        Vector yAxisWorld = new Vector(0f, 1f, 0f);

        //3. transform the x-axis to world coordinates
        Vector xAxisDroneTrans = PhysXEngine.droneOnWorld(xAxisDrone, orientation);

        //4. project the transformed x-axis onto the xz-plane in the world and normalize
        return (xAxisDroneTrans.orthogonalProjection(yAxisWorld)).normalizeVector();
    }

    /**
     * Calculates the absolute velocity of the tyre
     * @param state the state of the drone at moment of invocation of the method
     * @return the absolute velocity of the drone
     */
    public Vector getAbsoluteVelocity(DroneState state){
        Vector orientation = state.getOrientation();
        Vector rotation = state.getRotation();
        Vector position = state.getPosition();
        Vector velocity = state.getVelocity();

        Vector relPosAxleDrone = this.getTyrePosition();
        float currentTyreRadius = this.getTyreRadius() - this.calcRadiusDelta(orientation, position);
        Vector relPosTyreBottomDrone = relPosAxleDrone.vectorSum(new Vector(0f, currentTyreRadius, 0f));
        Vector relPosWorld = PhysXEngine.droneOnWorld(relPosTyreBottomDrone, orientation);
        Vector rotationVelocity = rotation.crossProduct(relPosWorld);

        return velocity.vectorSum(rotationVelocity);
    }
}
