package tests;
import gui.Window;

import internal.Autopilot.AutoPilot;
import internal.Autopilot.AutoPilotCamera;
import internal.Autopilot.AutoPilotInputs;
import internal.Helper.Pixel;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;
import internal.Testbed.Block;
import internal.Testbed.Drone;
import internal.Testbed.World;
import org.junit.Test;

import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotInputs;

import static java.lang.Math.PI;
import static org.junit.Assert.assertNotEquals;

import java.io.IOException;
import java.util.ArrayList;
import java.util.concurrent.ExecutionException;

import internal.Physics.WingPhysX;

import static org.junit.Assert.assertEquals;

/**
 * 
 * @author Anthony Rath�
 * 
 */
// TODO autopilot debuggen
public class autoPilotTests {

	public World world;
	public Window window;
	public Block redBlock;
	public AutoPilot AP;
	public AutoPilotCamera APCamera;
	public AutopilotConfig APConfig;
	public AutopilotInputs APInputs;
	public static Drone drone;
	public static WingPhysX wing1,wing2,wing3,wing4;
	
	public Vector position = new Vector(100,0,2);
	public Vector velocity = new Vector(1,1,0);
	public Vector orientation = new Vector((float) PI,(float) PI/2,0);
	public Vector rotation = new Vector(1,1,1);
	
	public Vector relPosWing1 = new Vector(1,0,0);
	public Vector relPosWing2 = new Vector(-1,0,0);
	public Vector relPosWing3 = new Vector(0,0f,1f);
	public Vector relPosWing4 = new Vector(0,0,2);
	
	private final static float RED_H_VALUE = 0.0f;
    private final static float RED_S_VALUE = 1.0f;
    private final static float Z_AXIS_V_VALUE = 0.7f;
    private final static float EPSILON  = 1E-5f;
    private final static float TIME_STEP = 0.1f;
	private final static float FRAMERATE = 20.0f;
	private final static int STEPS_PER_ITERATION = Math.round((1/ FRAMERATE)*TIME_STEP);
	
	public float delta = 0.000001f;
	
	/**
	 * Creates a byte array that simulates an image (101x101 with the red cube at a certain location.
	 * @param x
	 * @param y
	 * @return
	 */
	static byte[] createImage(int x, int y) {
		ArrayList<Byte> image = new ArrayList<Byte>();
		for(int row = -50; row < 50; row++) {
			for(int col = -50; col < 50; col++) {
				if (col == x && row == y) {
					// Add red RGB bytes
					image.add((byte) (255/188));
					image.add((byte) (0));
					image.add((byte) (0));
				}else {
					// Add non-red RGB bytes
					image.add((byte) (0));
					image.add((byte) (0));
					image.add((byte) (0));
				}
			}
		}
		byte[] imageArray = new byte[image.size()];
		for(int i = 0; i < image.size(); i++) {
			imageArray[i] = image.get(i);
		}
		return imageArray;
	}
	
//	@Before
//	public void setup() throws IOException{
//		WorldBuilder worldBuilder = new WorldBuilder();
//		// drone builder covers all the stuff involving building the drone, adjust parameters there
//		world = worldBuilder.createWorld();
//
//		// initialize a window
//		window = new Window(world);
//
//		//first render the image
//		window.renderFrame();
//		//pass the outputs to the drone
//		byte[] camera = window.getCameraView();
//		worldBuilder.DRONE.setAPImage(camera);
//		try {
//			world.advanceWorldState(TIME_STEP, STEPS_PER_ITERATION);
//		} catch (SimulationEndedException e) {
//			//ignore
//		} catch (IOException e) {
//			//ignore
//		}
//
//		world.getDrone().setThrust(20f);
//
//	}
	
	
	@Test
	@Deprecated
	public final void autoPilotOutputsTest() throws IOException{
		Pixel redPixel = new Pixel((byte) (255 + 128), (byte) 128, (byte) 128);
		Vector HSV = new Vector(redPixel.convertToHSV());
        float hValue = HSV.getxValue();
        float sValue = HSV.getyValue();
        float vValue = HSV.getzValue();
        
        System.out.println(hValue);
        System.out.println(sValue);
        System.out.println(vValue);
        
        System.out.println(redPixel.getGreenInt());
        
		System.out.println(Pixel.isEqualFloat(hValue, RED_H_VALUE, EPSILON) && Pixel.isEqualFloat(sValue, RED_S_VALUE, EPSILON) && Pixel.isEqualFloat(vValue, Z_AXIS_V_VALUE, EPSILON));
		APInputs = new AutoPilotInputs(autoPilotTests.createImage(1, 6), 0f, 0f, 0f, 0f, 0f, 0f, 0f);
		//assertEquals(AP.simulationStarted(APConfig, APInputs).getHorStabInclination(), (float) PI/3, delta);
		
	}
	
	@Test
	public final void autoPilotDroneInstructionTest() throws IOException, ExecutionException, InterruptedException {
		for (int i = 0; i < 1000; i++) {
			System.out.println(world.getDrone().getPosition());
			world.advanceWorldState(TIME_STEP, STEPS_PER_ITERATION);
		}
		
		
		
	}

	@Test
	public final void transformationsTest(){
		Vector testVector = new Vector(1,2,3);
		Vector orientationVector =new Vector((float)(20* PI/180), (float)(-5*PI/180), (float)(14*PI/180));
		Vector transformed = PhysXEngine.getRotationHPR(testVector, orientationVector);
		Vector result = PhysXEngine.HPRtoRotation(transformed, orientationVector);
		System.out.println("Old: " + testVector + "; new: " + result );
	}
	
	
	
	
	
}


