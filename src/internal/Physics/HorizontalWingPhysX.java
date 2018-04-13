package internal.Physics;

import internal.Helper.Vector;

/**
 * Created by Martijn & Jasper on 9/10/2017.
 */

public class HorizontalWingPhysX extends WingPhysX {

    public HorizontalWingPhysX(Vector relativePosition, float liftSlope, float mass, float maximumAngleOfAttack, float wingInclination){
        super(relativePosition, liftSlope,  mass, maximumAngleOfAttack, wingInclination);
    }
/*
    @Override
    public Vector getNormalInWorld() {
        Vector normal = this.getNormal();
        Drone drone = this.getDrone();
        Vector normalOnDrone = this.projectOnDrone(normal);
        return drone.droneOnWorld(normalOnDrone);
    }*/


    /**
     * (0, sin(leftWingInclination), -cos(leftWingInclination))
     * @return
     */
    @Override
    public Vector getAttackVector() {
        float inclination = this.getWingInclination();

        float x_part = 0.0f;
        float y_part = (float)Math.sin(inclination);
        float z_part = - (float)Math.cos(inclination);

        return new Vector(x_part, y_part, z_part);
    }

//    /**
//     * See pitch transformation matrix
//     * @param vector
//     * @return
//     */
//    @Override
//    public Vector projectOnDrone(Vector vector){
//
//        float inclination = this.getWingInclination();
//
//        float x_part = vector.getxValue();
//        float y_part = (float) Math.cos(inclination) * vector.getyValue()
//                - (float)Math.sin(inclination)*vector.getzValue();
//        float z_part = (float)Math.sin(inclination)*vector.getyValue()
//                +(float)Math.cos(inclination)*vector.getzValue();
//
//        return new Vector(x_part, y_part, z_part);
//
//    }

    @Override
    public Vector getAxisVector() {
        return this.axisVector;
    }

    public Vector getNormal(){
        float inclination = this.getWingInclination();
        return new Vector(0.f, (float)Math.cos(inclination), (float)Math.sin(inclination));
    }

    public final Vector normal = new Vector(0,1,0);
    public final Vector axisVector = new Vector(1,0,0);

}
