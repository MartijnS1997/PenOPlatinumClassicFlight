//package tests;
//import internal.*;
//import org.junit.Test;
//import static org.junit.Assert.assertNotEquals;
//
//import internal.WingPhysX;
//
//import static org.junit.Assert.assertEquals;
//
//import org.junit.Before;
//
///**
// *
// * @author Jonathan
// *
// */
//public class droneTest {
//
//	public float PI = (float) Math.PI;
//
//	public static Drone drone1,drone2;
//	public static WingPhysX wing1,wing2,wing3,wing4;
//	public Vector position = new Vector(1,0,2);
//	public Vector velocity = new Vector(1,1,0);
//	public Vector orientation = new Vector(PI,PI/2,0);
//	public Vector rotation = new Vector(1,1,1);
//
//	public Vector relPosWing1 = new Vector(1,0,0);
//	public Vector relPosWing2 = new Vector(-1,0,0);
//	public Vector relPosWing3 = new Vector(0,0,2);
//	public Vector relPosWing4 = new Vector(0,0,1);
//
//	public AutoPilot AP = new AutoPilot();
//
//	public float delta = 0.000001f;
//
//
//
//	HorizontalWingPhysX wingTest1 = new HorizontalWingPhysX(relPosWing1,20,20,1,0.5f);
//
//	@Before
//	public void wingSetup(){
//		//liftslope/mass/maxaoa/incl
//		wing1 = new HorizontalWingPhysX(relPosWing1,PI/3,10,1.5f,PI);
//		wing2 = new HorizontalWingPhysX(relPosWing2,PI/3,10,1.5f,PI);
//		wing3 = new HorizontalWingPhysX(relPosWing3,PI/3,5,1.5f,PI);
//		wing4 = new VerticalWingPhysX(relPosWing4,PI/3,5,1.5f,PI);
//	}
//
//
//	@Before
//	public void setupDrones(){
//		//enginemass/maxthrust
//		drone1 = new Drone(50, 10, position, velocity, orientation,
//				rotation, wing2, wing1, wing3, wing4);
//	}
//
//	@Test
//	public final void droneOnWorldTest(){
//		Vector v = drone1.droneOnWorld(orientation);
//		assertEquals(v.getxValue(), -PI, delta);
//		assertEquals(v.getyValue(), 0, delta);
//		assertEquals(v.getzValue(), -PI/2, delta);
//	}
//
//
////	@Test
////	public final void getEulerRotationTest(){
////		Vector v = drone1.getEulerRotations(orientation);
////		assertEquals(v.getyValue(), -PI, delta);
////		assertEquals(v.getxValue(), 0, delta);
////		assertEquals(v.getzValue(), PI/2, delta);
////	}
//
//
//
//	@Test
//	public final void getTotalMassTest(){
//		float m = drone1.getTotalMass();
//		float mass = 50f+10f+10f+5f+5f;
//		float mass2 = 50f+10f+10f+5f+5f+0.0001f;
//		assertEquals(mass, m, delta);
//		assertNotEquals(mass2, m, delta);
//	}
//
//
//	@Test
//	public final void setInertiaTensorTest(){
//		float[] m = {29.5f,0,0,
//		             0,49.5f,0,
//		             0,0,10+10};
//		SquareMatrix inert = drone1.getInertiaTensor();
//		assertEquals(new SquareMatrix(m), inert);
//	}
//
//	@Test
//	public final void setLeftWingTest(){
//		drone1.setLeftWing(wingTest1);
//		assertEquals(drone1.getLeftWing(), wingTest1);
//		assertEquals(drone1.getLeftWing().getMass(),20,delta);
//	}
//
//	@Test
//	public final void setRightWingTest(){
//		drone1.setRightWing(wingTest1);
//		assertEquals(drone1.getRightWing(), wingTest1);
//		assertEquals(drone1.getRightWing().getLiftSlope(),20,delta);
//	}
//
//	@Test
//	public final void setOrientationTest(){
//		float h = 5;
//		float p = 3;
//		float r = 4;
//		drone1.setOrientation(new Vector(h,p,r));
//		assertEquals(drone1.getHeading(), h-2*PI, delta);
//		assertEquals(drone1.getPitch(), p, delta);
//		assertEquals(drone1.getRoll(), r-2*PI, delta);
//
//	}
//
//	@Test
//	public final void setThrustTest(){
//		drone1.setThrust(10);
//		assertEquals(drone1.getThrust(), 10,delta);
//	}
//
//	@Test (expected = IllegalArgumentException.class)
//	public final void setThrustTest2(){
//		drone1.setThrust(31);
//	}
//
//	@Test
//	public final void getGravityTest(){
//		Vector g = drone1.getGravity();
//		Vector v = new Vector(0f,(float)(-drone1.getTotalMass()*9.8106),0f);
//		assertEquals(g.getyValue(), v.getyValue(), 0.001);
//		assertEquals(g.getxValue(), v.getxValue(), 0.001);
//		assertEquals(g.getzValue(), v.getzValue(), 0.001);
//	}
//
////	@Test (expected = IllegalArgumentException.class)
////	public final void canHaveAsDroneMassTest(){
////		drone2 = new Drone(0, 10, position, velocity, orientation,
////				rotation, wing2, wing1, wing3, wing4, AP);
////		drone2.canHaveAsDroneMass(drone2.getDroneMass());
////	}
//
//	@Test (expected = IllegalArgumentException.class)
//	public final void setVelocityTest(){
//		float a = 1000000000;
//		Vector v = new Vector(a,a,a);
//		drone1.setVelocity(v);
//	}
//
//	@Test
//	public final void setVelocityTest2(){
//		float a = 100;
//		Vector v = new Vector(a,a,a);
//		drone1.setVelocity(v);
//		assertEquals(drone1.getVelocity(), v);
//	}
//
//	@Test
//	public final void getHeadingRotationVectTest(){
//		Vector i = new Vector(PI/3,0,0);
//		Vector v = drone1.getHeadingRotationVector(i);
//
//		float x = (float) (0 +0+ (PI/3*Math.sin(drone1.getHeading())*Math.tan(drone1.getPitch())));
////		System.out.println(drone1.getHeading());
////		System.out.println(drone1.getPitch());
////		System.out.println(i.getyValue());
//		assertEquals(v.getxValue(), x, delta);
//	}
//
//	@Test
//	public final void getPitchRotationVectTest(){
//		Vector i = new Vector(PI/3,0,0);
//		Vector v = drone1.getPitchRotationVector(i);
//		float x = (float) Math.cos(-PI)*PI/3;
//		assertEquals(x, v.getyValue(),delta);
//	}
//
//	@Test
//	public final void getRollRotationVectTest(){
//		Vector i = new Vector(PI/3,0,0);
//		Vector v = drone1.getRollRotationVector(i);
//		float x = (float) (0+Math.sin(-PI)*PI/3/1);
//		assertEquals(x, v.getyValue(),delta);
//	}
//
//
//	@Test
//	public final void getRotationHPRTest(){
//		Vector r = new Vector(PI/3,0,0);
//		Vector v = drone1.getRotationHPR(r);
//		Vector x = new Vector(drone1.getHeadingRotationVector(r).getxValue(),drone1.getPitchRotationVector(r).getyValue(),
//				drone1.getRollRotationVector(r).getzValue());
//		assertEquals(x, v);
//	}
//
//
//
//	@Test
//	public final void getAOATests(){
//
//		Vector v = wing1.getAbsoluteVelocity();
//		Vector r = drone1.getRotationVector();
//		Vector a = drone1.droneOnWorld(wing1.getRelativePosition());
//		Vector t = a.crossProduct(r).vectorSum(v);
//		Vector nor = wing1.projectOnWorld(wing1.getNormal());
//
//		//System.out.println("aoaNom:"+v.scalarProduct(nor));
//		//System.out.println("aoaDenom:"+v.scalarProduct(wing1.getAttackVector()));
//
//		//ABSVELOCITY
//		assertEquals(t.getxValue(), 1, delta);
//		assertEquals(t.getyValue(), 3, delta);
//		assertEquals(t.getzValue(), -2, delta);
//		//System.out.println(t.getyValue());
//		//fout 1.5*10^-7
//		//System.out.println(drone1.droneOnWorld(wing1.getRelativePosition()));
//
//		//PROJONWORLD
//		Vector n = new Vector(0,1,0);
//		Vector n2 = wing1.projectOnWorld(n);
//		assertEquals(wing1.projectOnWorld(n).getxValue(),0,delta);
//		assertEquals(wing1.projectOnWorld(n).getyValue(),0,delta);
//		assertEquals(wing1.projectOnWorld(n).getzValue(),1,delta);
//
//		//ATKVECTOR
//		assertEquals(wing1.getAttackVector().getxValue(),0,delta);
//		assertEquals(wing1.getAttackVector().getyValue(),0,delta);
//		assertEquals(wing1.getAttackVector().getzValue(),1,delta);
//		//fout E-7
//		//System.out.println(wing1.getAttackVector());
//
//		assertEquals(t.scalarProduct(n2), -2, delta);
//		assertEquals(t.scalarProduct(wing1.getAttackVector()), -2, delta);
//		//fout E-7
//		//System.out.println(t.scalarProduct(n2));
//		//System.out.println(t.scalarProduct(wing1.getAttackVector()));
//		//System.out.println(Math.atan2(t.scalarProduct(n2), t.scalarProduct(wing1.getAttackVector())));
//		wing1.setWingInclination(PI/4);
//		//System.out.println(wing1.getAngleOfAttack());
//		//System.out.println((float)Math.tan(PI));
//	}
//
//
//
//	@Test
//	public final void getNextVelocityTest(){
//		float d = 0.01f;
//		Vector a = new Vector(1,0,0);
//		for (int i=0; i <= 10000; i++){
//			Vector a2 = drone1.getVelocity();
//			drone1.setVelocity(new Vector(a2.getxValue()+a.getxValue()*d,a2.getyValue()+a.getyValue()*d,
//					a2.getzValue()+a.getzValue()*d));
//		}
//		Vector v1 = drone1.getVelocity();
//		drone1.setVelocity(velocity);
//		for (int i=0; i <= 10000; i++){
//			drone1.setVelocity(drone1.getNextVelocity(d, a));
//		}
//		Vector v2 = drone1.getVelocity();
//		assertEquals(v1, v2);
//	}
//
//
//
//	@Test
//	public final void getNextPosTest(){
//		float d = 2f;
//		Vector a = new Vector(1,1,1);
//		Vector r = drone1.getNextPosition(d,a);
//		Vector rr = new Vector(1+d*1+1*d,0+d*1+1*d,2+d*0+1*d);
//		assertEquals(rr, r);
//	}
//
//
//
//
//
////	@Test
////	public final void extForceTest(){
////
////	}
//
//
//
//	@Test
//	public final void liftTest(){
//		wing3.calcAngleOfAttack();
//		Vector n = wing3.getNormal();
//		Vector a = wing3.getAbsoluteVelocity();
//		float aoa = wing3.getAngleOfAttack();
//
//		float a2 = a.scalarProduct(a);
//		float prod = PI/3*a2*aoa;
//		Vector l = n.scalarMult(-prod);
//		//System.out.println("calc:"+wing3.getLift());
//	//	System.out.println("res:"+l);
//
//
//	}
//}
//
//
//
//
