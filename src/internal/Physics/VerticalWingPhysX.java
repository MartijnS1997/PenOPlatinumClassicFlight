package internal.Physics;

import internal.Helper.Vector;

/**
 * Created by Martijn & Jasper on 9/10/2017.
 */
public class VerticalWingPhysX extends WingPhysX {

    public VerticalWingPhysX(Vector relativePosition, float liftSlope, float mass, float maximumAngleOfAttack, float wingInclination){
        super(relativePosition, liftSlope, mass, maximumAngleOfAttack, wingInclination);

    }

    @Override
    public Vector getNormal() {
        float inclination = this.getWingInclination();
        return new Vector((float)-Math.cos(inclination), 0.f, (float)Math.sin(inclination));
    }

    /**
     * (-sin(verStabInclination), 0, -cos(verStabInclination))
     * @return
     */
    @Override
    public Vector getAttackVector() {
        float inclination = this.getWingInclination();

        float x_part = -(float)Math.sin(inclination);
        float y_part = 0.0f;
        float z_part = -(float)Math.cos(inclination);

        return new Vector(x_part, y_part, z_part);
    }

    @Override
    public Vector getAxisVector() {
        return this.axisVector;
    }

//    @Override
//    public Vector projectOnDrone(Vector vector){
//
//        float inclination = this.getWingInclination();
//
//        float x_part = (float) -Math.cos(inclination) * vector.getxValue()
//                + (float)Math.sin(inclination)*vector.getzValue();
//        float y_part = vector.getyValue();
//        float z_part = +(float)Math.sin(inclination)*vector.getxValue()
//                + (float)Math.cos(inclination)*vector.getzValue();
//
//        return new Vector(x_part, y_part, z_part);
//    }
    //TODO original value of normal was 1, restore if shit breaks
    public final Vector normal = new Vector(-1,0,0);
    public final Vector axisVector = new Vector(0, 1, 0);

}
