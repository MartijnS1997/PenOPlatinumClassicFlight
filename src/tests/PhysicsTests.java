//package tests;
//
//import internal.*;
//import org.junit.Before;
//import org.junit.Test;
//
//import static org.junit.Assert.assertEquals;
//import static org.junit.Assert.assertNotEquals;
//
///**
// * Created by Martijn on 17/10/2017.
// */
//public class PhysicsTests {
//
//    public static HorizontalWingPhysX rightMain, leftMain, horizontalStab;
//    public static VerticalWingPhysX verticalStab;
//    public static Drone drone;
//
//    public static float EPSILON = 10E-5f;
//
//    @Before
//    public void setupMutableFixture(){
//        float wingpos =1.0f;
//        float wingmass = 1.0f;
//        float engineMass = 1.0f;
//        float liftSlope = 1.0f;
//        float maxInclination = (float) Math.PI/2 - 0.0001f;
//        float givenInclination = (float) Math.PI/6;
//        float maxThrust = 1.0f;
//        Vector startPos = new Vector();
//        Vector startVelocity = new Vector(0,0,-10.0f);
//        Vector startOrientation = new Vector(0,0,0);
//        Vector startRotation = new Vector(0,0,0);
//
//        rightMain = new HorizontalWingPhysX(new Vector(wingpos,0,0 ), liftSlope, wingmass, maxInclination, givenInclination);
//        leftMain = new HorizontalWingPhysX(new Vector(-wingpos, 0,0), liftSlope, wingmass, maxInclination, givenInclination);
//        horizontalStab = new HorizontalWingPhysX(new Vector(0, 0, wingpos), liftSlope, wingmass, maxInclination, 0.0f);
//        verticalStab = new VerticalWingPhysX(new Vector(0,0,wingpos), liftSlope, wingmass, maxInclination, 0.0f);
//
//        drone = new Drone(engineMass, maxThrust, startPos, startVelocity, startOrientation, startRotation,
//                rightMain, leftMain, horizontalStab, verticalStab);
//
//
//    }
//
//    /**
//     * Test for the transformation matrix, world on drone
//     */
//    @Test
//    public void testTransformationMatrixDOW(){
//        drone.setOrientation((float) (Math.PI/4), 0.0f, 0.0f);
//        Vector zAxisDrone = new Vector(0.0f, 0.0f, 1.0f);
//        Vector transformed = drone.droneOnWorld(zAxisDrone);
//        Vector control = new Vector((float)Math.sqrt(2)/2.0f, 0.0f,(float) Math.sqrt(2)/2.0f);
//        assertEquals(transformed, control);
//    }
//
//    @Test
//    public void testEnginePos(){
//        Vector enginePos = drone.getEnginePos();
//        assertEquals(enginePos.getzValue(), -2.0f, EPSILON);
//    }
//
//    /**
//     * Test if the force exerted on the drone has no x component
//     * and the z and y components are nonzero
//     */
//    @Test
//    public void testForce(){
//        Vector forceVector = drone.getTotalExternalForcesWorld();
//        assertEquals(forceVector.getxValue(), 0.0f, EPSILON);
//        assertNotEquals(forceVector.getyValue(), 0.0f, EPSILON);
//        assertNotEquals(forceVector.getzValue(), 0.0f, EPSILON);
//
//    }
//
//    @Test
//    public void testMoment(){
//
//        Vector moment = drone.getTotalMomentDrone();
//        assertEquals(moment.getxValue(), 0.0f, EPSILON);
//        assertEquals(moment.getyValue(), 0.0f, EPSILON);
//        assertEquals(moment.getzValue(), 0.0f, EPSILON);
//    }
//
//    @Test
//    public void testHPRRotationStandardOrientation(){
//        Vector rotationVecor = new Vector(1.0f, 0.0f, 1.0f);
//        Vector rotationHPR = drone.getRotationHPR(rotationVecor);
//        assertEquals(rotationHPR.getxValue(), 0.0f, EPSILON);
//        assertEquals(rotationHPR.getyValue(), 1.0f, EPSILON);
//        assertEquals(rotationHPR.getzValue(), 1.0f, EPSILON);
//    }
//
//    @Test
//    public void testHRPRotationCustomOrientation(){
//        float heading = (float)Math.PI/4.0f;
//        float pitch = (float)Math.PI/4.0f;
//        drone.setOrientation(new Vector(heading, pitch, 0.0f));
//        Vector rotationVector = new Vector(1.0f, 1.0f, 1.0f);
//        Vector HPRrotation = drone.getRotationHPR(rotationVector);
//        assertEquals(HPRrotation.getxValue(), 1.0f+ Math.sqrt(2), EPSILON);
//        assertEquals(HPRrotation.getyValue(), 0.0f, EPSILON);
//        assertEquals(HPRrotation.getzValue(), 2.0f, EPSILON);
//
//    }
//
//    @Test
//    public void testTensor(){
//        SquareMatrix inertia = drone.getInertiaTensor();
//        SquareMatrix calcInertia = new SquareMatrix(new float[]{6.0f, 0.0f, 0.0f, 0.0f, 8.0f, 0.0f, 0.0f, 0.0f, 2.0f});
//        assertEquals(inertia, calcInertia);
//    }
//
//    @Test
//    public void testAngularAccelerationStandard(){
//        Vector angularAcceleration = drone.calcAngularAcceleration();
//        assertEquals(angularAcceleration, new Vector());
//    }
//
////    @Test
////    public void getNextStateTestAt0(){
////        drone.nextState();
////        Vector nextOrientation = drone.getOrientation();
////        Vector nextRotation = drone.getRotationVector();
////
////        assertEquals(new Vector(), nextOrientation);
////        assertEquals(new Vector(), nextRotation);
////    }
//
//    @Test
//    public void testAngularAccelerationRoll(){
//        drone.getLeftWing().setWingInclination((float) (-Math.PI/6.0f));
//        drone.getRightWing().setWingInclination((float) (Math.PI/6.0f));
//
//        Vector angularAcceleration = drone.calcAngularAcceleration();
//        assertEquals(0.0f, angularAcceleration.getxValue(), EPSILON);
//        assertEquals(0.0f, angularAcceleration.getyValue(), EPSILON);
//        assertNotEquals(0.0f, angularAcceleration.getzValue());
//
//    }
//
//    @Test
//    public void testAngularAccelerationPitch(){
//        drone.getHorizontalStab().setWingInclination((float) (Math.PI/6.0f));
//        Vector angularAcceleration = drone.calcAngularAcceleration();
//
//        assertNotEquals(0.0f, angularAcceleration.getxValue(), EPSILON);
//        assertEquals(0.0f, angularAcceleration.getyValue(), EPSILON);
//        assertEquals(0.0f, angularAcceleration.getzValue(), EPSILON);
//    }
//
//    @Test
//    public void testAngularAccelerationHeading(){
//        drone.getVerticalStab().setWingInclination((float) (Math.PI/6.0f));
//        Vector angularAcceleration = drone.calcAngularAcceleration();
//
//        assertEquals(0.0f, angularAcceleration.getxValue(), EPSILON);
//        assertNotEquals(0.0f, angularAcceleration.getyValue(), EPSILON);
//        assertEquals(0.0f, angularAcceleration.getzValue(), EPSILON);
//
//    }
//}
