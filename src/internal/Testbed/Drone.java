
package internal.Testbed;

import java.io.IOException;
import java.lang.Math;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.Callable;

import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotOutputs;
import gui.Cube;
import gui.GraphicsObject;
import gui.Wheel;
import gui.Polygon;
import internal.Physics.PhysXEngine;
import internal.Physics.PhysicsEngineState;
import internal.Helper.Vector;
import math.Vector3f;

/**
 * @author Anthony Rathé & MartijnSauwens & Bart
 * Immutable variables: maxThrust, engineMass, enginePosition, leftWing, rightWing,
 * 						horizontalStab, verticalStab, inertiaTensor
 * 	note: Orientation = (heading, pitch, roll) (in that order)
 * 	the orientation has always values in the range [-PI, PI]
 */
public class Drone implements WorldObject, Callable<Void> {


	/*
	########################## Methods used for initialisation ##########################
	 */

	/**
	 * Simple constructor for generating drones that do not touch the ground (tyre delta's are initialized at zero)
	 * @param position the position of the drone in the simulation world
	 * @param velocity the velocity of the drone in the simulation world
	 * @param orientation the orientation of the drone in the simulation world
	 * @param rotationVector the rotation of the drone
	 * @param config the configuration for the physical properties of the drone
	 */
	public Drone(Vector position, Vector velocity, Vector orientation, Vector rotationVector, AutopilotConfig config){
		this(position, velocity, orientation, rotationVector, 0f, 0f, 0f, config);
		//check if the drone is actually flying
		if(this.getPhysXEngine().chassisTouchesGround(orientation, position)){
			//Todo uncomment if the simulator is ready for ground
			//throw new IllegalStateException("Drone touches ground, cannot initialize");
		}
	}

	/**
	 * More compact constructor used for building drones in the drone builder class
	 * @param state the state of the drone at the start of the simulation
	 * @param config the configuration of the drone
	 */
	public Drone(DroneState state, AutopilotConfig config){

		this(state.getPosition(), state.getVelocity(), state.getOrientation(), state.getRotation()
				, state.getPrevFrontTyreDelta(), state.getPrevRearLeftTyreDelta(), state.getPrevRearRightTyreDelta(), config);

	}

	/**
	 * Constructor for a drone class object
	 *
	 * @param position       the position of the drone in space
	 * @param velocity       the velocity of the drone
	 * @param orientation    the orientation of the drone (roll, pitch heading)
	 * @param rotationVector the rotational vector of the drone
	 * @param prevFrontTyreDelta the previous compression of the front tyre
	 * @param prevRearLeftTyreDelta the previous compression of the rear left tyre
	 * @param prevRearRightTyreDelta the previous compression of the rear right tyre
	 * @param configuration  the configuration needed for the physics engine
	 */
	public Drone(Vector position, Vector velocity, Vector orientation,
                 Vector rotationVector, float prevFrontTyreDelta, float prevRearLeftTyreDelta, float prevRearRightTyreDelta,
				 AutopilotConfig configuration) {

		PhysXEngine physXEngine = new PhysXEngine(configuration);
		this.setPhysXEngine(physXEngine);

		// set variable variables of the drone
		this.setPosition(position);
		this.setVelocity(velocity);
		this.setOrientation(orientation);
		this.setRotationVector(rotationVector);

		//more instance variables used for the chassis of the drone
		this.setPrevFrontTyreDelta(prevFrontTyreDelta);
		this.setPrevRearLeftTyreDelta(prevRearLeftTyreDelta);
		this.setPrevRearRightTyreDelta(prevRearRightTyreDelta);

		//PhysXEngine.PhysXOptimisations optimisations = this.getPhysXEngine().createPhysXOptimisations();
		//this.setVelocity(optimisations.balanceDrone(this.getOrientation(), (float) (7*PI/180), 0.0f)[1]);

		//Todo set the tyreDelta to appropriate initial value

		// the cube associated with the drone
		try {
			
			Cube wings = new Cube(position.convertToVector3f(), new Vector3f(240f, 1f, 1f), false);
			wings.setSize(new Vector3f(5f, 0.1f, 0.7f).scale(scalingFactor));
			this.setAssociatedGraphicsObject(wings);
			
			Vector3f position2 = new Vector3f(0f, 0f, 1.75f).scale(scalingFactor);
			Cube middleFrame = new Cube(position2, new Vector3f(240f, 1f, 1f), wings, false);
			middleFrame.setSize(new Vector3f(0.6f, 0.6f, 2.5f).scale(scalingFactor));
			this.setAssociatedGraphicsObject(middleFrame);
			
			Vector3f position3 = new Vector3f(0f, 0.6f, 4.5f).scale(scalingFactor);
			Cube verticalStabilizer = new Cube(position3, new Vector3f(240f, 1f, 1f), wings, false);
			verticalStabilizer.setSize(new Vector3f(0.2f, 1.2f, 0.5f).scale(scalingFactor));
			this.setAssociatedGraphicsObject(verticalStabilizer);
			
			Vector3f position4 = new Vector3f(0f, 1f, 4.5f).scale(scalingFactor);
			Cube horizontalStabilizer = new Cube(position4, new Vector3f(240f, 1f, 1f), wings, false);
			horizontalStabilizer.setSize(new Vector3f(1.7f, 0.1f, 0.5f).scale(scalingFactor));
			this.setAssociatedGraphicsObject(horizontalStabilizer);
			
			Vector3f position5 = new Vector3f(0f, 0f, 3.6f).scale(scalingFactor);
			Cube longFrame = new Cube(position5, new Vector3f(240f, 1f, 1f), wings, false);
			longFrame.setSize(new Vector3f(0.3f, 0.3f, 1.5f).scale(scalingFactor));
			this.setAssociatedGraphicsObject(longFrame);
			
			Vector3f position6 = new Vector3f(0f, 0.4f, -0.5f).scale(scalingFactor);
			Cube shortFrame = new Cube(position6, new Vector3f(240f, 1f, 1f), wings, false);
			shortFrame.setSize(new Vector3f(1f, 1f, 2f).scale(scalingFactor));
			this.setAssociatedGraphicsObject(shortFrame);
			
			Vector3f position7 = new Vector3f(0f, 0f, -1.5f).scale(scalingFactor);
			Cube front = new Cube(position7, new Vector3f(240f, 1f, 1f), wings, false);
			front.setSize(new Vector3f(0.8f, 0.5f, 1f).scale(scalingFactor));
			this.setAssociatedGraphicsObject(front);
			
			
			Vector3f positioncp1 = new Vector3f(0f, 0.8f, -0.5f).scale(scalingFactor);
			Cube cockpit1 = new Cube(positioncp1, new Vector3f(30f, 1f, 1f), wings, false);
			cockpit1.setSize(new Vector3f(0.7f, 0.8f, 1f).scale(scalingFactor));
			this.setAssociatedGraphicsObject(cockpit1);
			
//			Vector3f positioncp2 = new Vector3f(0f, 0.8f, -0.5f).scale(scalingFactor);
//			Wheel cockpitcp2 = new Wheel(positioncp1, new Vector3f(30f, 1f, 1f), wings, 25);
//			cockpit1.setSize(new Vector3f(0.7f, 0.8f, 1f).scale(scalingFactor));
//			this.setAssociatedGraphicsObject(cockpit1);
			
			Vector3f rearwheel1position = new Vector3f(0.5f, -0.5f, 0f).scale(scalingFactor);
			Wheel rearwheel1 = new Wheel(rearwheel1position, new Vector3f(0f, 1f, 1f), wings, 25);
			rearwheel1.setSize(new Vector3f(0.1f, 0.2f, 0.2f).scale(scalingFactor));
			this.setAssociatedGraphicsObject(rearwheel1);
			
			Vector3f rearwheel2position = new Vector3f(-0.5f, -0.5f, 0f).scale(scalingFactor);
			Wheel rearwheel2 = new Wheel(rearwheel2position, new Vector3f(0f, 1f, 1f), wings, 25);
			rearwheel2.setSize(new Vector3f(0.1f, 0.2f, 0.2f).scale(scalingFactor));
			this.setAssociatedGraphicsObject(rearwheel2);
			
			Vector3f frontwheelposition = new Vector3f(0f, -0.5f, -1.5f).scale(scalingFactor);
			Wheel frontwheel = new Wheel(frontwheelposition, new Vector3f(0f, 1f, 1f), wings, 25);
			frontwheel.setSize(new Vector3f(0.1f, 0.2f, 0.2f).scale(scalingFactor));
			this.setAssociatedGraphicsObject(frontwheel);
			
		}catch(NullPointerException e){
			this.setAssociatedGraphicsObject(new Cube(position.convertToVector3f(), new Vector3f(240f, 100f, 100f), false));
		}

		this.setAutopilotConfig(configuration);

	}

	/*
	########################## Next state methods ##########################
	 */

	/**
	 * Executes one iteration of the to next state method (needed for concurrent calculation in the world)
	 * @return nothing
	 * @throws Exception may throw AOA exception
	 *
	 * Todo: Now every time a next step is calculated we need to call this callable, maybe more efficient if
	 * we only call once for all steps (but how to check? do some sub iterations that are non critical? eg 5 of the 20)
	 */
	@Override
	public Void call() throws Exception {
		float deltaTime = this.getDeltaTime();
		toNextState(deltaTime);
		return null;
	}

	/**
	 * advances the drone for a given time step, it changes the position, velocity, orientation and rotation
	 * variables
	 *
	 * @param deltaTime the time step
	 * @throws IOException
	 * @author Martijn Sauwens & Bart Jacobs
	 */
	@Override
	public void toNextState(float deltaTime) throws IOException {
		if (!WorldObject.isValidTimeStep(deltaTime))
			throw new IllegalArgumentException(INVALID_TIMESTEP);
		float INSIGNIFICANCE = 0.0001f;

		//engage autopilot
		AutopilotOutputs autopilotOutputs = this.getAutopilotOutputs();

		DroneState state = this.getDroneState();
		//calculate the next state of the physics engine
		PhysicsEngineState nextState = this.getPhysXEngine().getNextStatePhysXEngine(deltaTime,state, autopilotOutputs, INSIGNIFICANCE);


		Vector differencePos = (nextState.getPosition()).vectorDifference(this.getPosition());

		// move the cube representing the drone

		try{
			for (GraphicsObject polygon: this.getAssociatedGraphicsObjects())
				((Polygon) polygon).update(differencePos.convertToVector3f(), getOrientation().convertToVector3f());
		}catch(NullPointerException e){

			//let it go
		}

//		System.out.println("prev left tyre delta: " + state.getPrevRearLeftTyreDelta());
//		System.out.println("Rear left tyre delta: " + nextState.getRearLeftTyreDelta());
//		System.out.println("----------------");

		this.setPosition(nextState.getPosition());
		this.setVelocity(nextState.getVelocity());
		this.setOrientation(nextState.getOrientation());
		this.setRotationVector(nextState.getRotation());
		this.setPrevFrontTyreDelta(nextState.getFrontTyreDelta());
		this.setPrevRearLeftTyreDelta(nextState.getRearLeftTyreDelta());
		this.setPrevRearRightTyreDelta(nextState.getRearRightTyreDelta());

	}

	//TODO implement, pass the required arguments to the physXEngine he knows how to handle the request
	public boolean checkCrash(){
		Vector orientation = this.getOrientation();
		Vector position = this.getPosition();
		return this.getPhysXEngine().checkCrash(orientation, position);
	}


	/*
	 ############################# Gui configuration methods #############################
	 */

	public Set<GraphicsObject> getAssociatedGraphicsObjects() {
		return droneCubes;
	}


	private void setAssociatedGraphicsObject(GraphicsObject droneCube) {
		this.droneCubes.add(droneCube);
	}


	/*
	 ############################# Drone configuration methods #############################
	  */

	public DroneState getDroneState(){
		return new DroneState() {
			@Override
			public Vector getPosition() {
				return Drone.this.getPosition();
			}

			@Override
			public Vector getVelocity() {
				return Drone.this.getVelocity();
			}

			@Override
			public Vector getOrientation() {
				return Drone.this.getOrientation();
			}

			@Override
			public Vector getRotation() {
				return Drone.this.getRotationVector();
			}

			@Override
			public float getPrevFrontTyreDelta() {
				return Drone.this.getPrevFrontTyreDelta();
			}

			@Override
			public float getPrevRearLeftTyreDelta() {
				return Drone.this.getPrevRearLeftTyreDelta();
			}

			@Override
			public float getPrevRearRightTyreDelta() {
				return Drone.this.getPrevRearRightTyreDelta();
			}
		};
	}

	/**
	 * Getter for the orientation of the drone
	 *
	 * @return a vector of the following format: (heading, pitch, roll)
	 */
	public Vector getOrientation() {
		return Orientation;
	}

	/**
	 * Setter for the orientation of the drone
	 *
	 * @param orientation vector containing the orientation of the drone
	 *                    structured (heading, pitch, roll)
	 */
	public void setOrientation(Vector orientation) {
		Vector tempOrientation = normalizeOrientation(orientation);
		Orientation = tempOrientation;
	}

	/**
	 * Setter for the orientation of the drone
	 *
	 * @param heading the heading value of the rotation
	 * @param pitch   the pitch value of the rotation
	 * @param roll    the roll value of the rotation
	 * @author Martijn Sauwens
	 */
	public void setOrientation(float heading, float pitch, float roll) {
		this.Orientation = normalizeOrientation(new Vector(heading, pitch, roll));
	}

	/**
	 * Normalizes the given orientation, with normalization meaning setting
	 * the range of the orientation elements to the range [-PI, PI]
	 *
	 * @param orientation the vector containing the orientation of the drone (heading, pitch, roll)
	 * @return a new vector with all elements rescaled to the range [-PI, PI]
	 * @author Martijn Sauwens
	 */
	public Vector normalizeOrientation(Vector orientation) {
		float[] vectorArray = new float[Vector.VECTOR_SIZE];
		for (int index = 0; index != Vector.VECTOR_SIZE; index++) {
			float tempValue = orientation.getElementAt(index);
			//first set the value between the range [0, 2PI]
			tempValue = (float) (tempValue % (2 * Math.PI));
			//then if the value is larger than PI subtract PI*2
			//so the range becomes [-PI, PI]
			if (tempValue > Math.PI) {
				tempValue = (float) (tempValue - Math.PI * 2);
			}
			vectorArray[index] = tempValue;
		}

		return new Vector(vectorArray);
	}

	/**
	 * @return the heading (rotation around the x-axis) of the drone
	 */
	public float getHeading() {
		return this.getOrientation().getxValue();
	}

	/**
	 * @return the pitch (rotation around the y axis) of the drone
	 */
	public float getPitch() {
		return this.getOrientation().getyValue();
	}

	/**
	 * @return the roll (rotation around the z-axis) of the drone
	 */
	public float getRoll() {
		return this.getOrientation().getzValue();
	}


	/**
	 * Getter for the position variable
	 *
	 * @return a vector containing the position
	 */
	@Override
	public Vector getPosition() {
		return position;
	}

	/**
	 * Setter for the position of the drone
	 *
	 * @param position the position of the drone
	 */
	public void setPosition(Vector position) {
		this.position = position;
	}

	/**
	 * Getter for the velocity of the drone
	 *
	 * @author Martijn Sauwens
	 */
	public Vector getVelocity() {
		return velocity;
	}


	/**
	 * Setter for the velocity of the drone
	 *
	 * @param velocity the desired velocity of the drone
	 * @throws IllegalArgumentException if the speed is invalid
	 * @author Martijn Sauwens
	 */
	public void setVelocity(Vector velocity) throws IllegalArgumentException {
		if (this.isValidVelocity(velocity)) {
			this.velocity = velocity;
		} else {
			throw new IllegalArgumentException(VELOCITY_ERROR);
		}
	}

	public boolean isValidVelocity(Vector velocity) {
		return velocity.getSize() <= LIGHTSPEED;
	}


	/**
	 * Checks if the given max thrust can be assigned as maximum thrust of the drone
	 *
	 * @param maxThrust the desired maximum thrust of the drone
	 * @return true if and only if the maxThrust > 0
	 */
	public boolean canHaveAsMaxThrust(float maxThrust) {
		return maxThrust > 0;
	}


	/**
	 * Getter for the rotation vector
	 *
	 * @return the rotation vector of the drone
	 * @author Martijn Sauwens
	 */
	public Vector getRotationVector() {
		return rotationVector;
	}

	/**
	 * setter for the rotation vector
	 *
	 * @param rotationVector the desired rotation vector
	 */
	public void setRotationVector(Vector rotationVector) {
		this.rotationVector = rotationVector;
	}


	//Todo add comment

	public PhysXEngine getPhysXEngine() {
		return physXEngine;
	}

	public void setPhysXEngine(PhysXEngine physXEngine) {
		this.physXEngine = physXEngine;
	}

	public AutopilotOutputs getAutopilotOutputs() {
		return autopilotOutputs;
	}

	public void setAutopilotOutputs(AutopilotOutputs autopilotOutputs) {
		this.autopilotOutputs = autopilotOutputs;
	}

	public AutopilotConfig getAutopilotConfig() {
		return autopilotConfig;
	}

	public void setAutopilotConfig(AutopilotConfig autopilotConfig) {
		this.autopilotConfig = autopilotConfig;
	}

	public void addFlightRecorder(FlightRecorder flightRecorder){
		this.getPhysXEngine().setFlightRecorder(flightRecorder);
	}

	public float getPrevFrontTyreDelta() {
		return prevFrontTyreDelta;
	}

	public void setPrevFrontTyreDelta(float prevFrontTyreDelta) {
		this.prevFrontTyreDelta = prevFrontTyreDelta;
	}

	public float getPrevRearLeftTyreDelta() {
		return prevRearLeftTyreDelta;
	}

	public void setPrevRearLeftTyreDelta(float prevRearLeftTyreDelta) {
		this.prevRearLeftTyreDelta = prevRearLeftTyreDelta;
	}

	public float getPrevRearRightTyreDelta() {
		return prevRearRightTyreDelta;
	}

	public void setPrevRearRightTyreDelta(float prevRearRightTyreDelta) {
		this.prevRearRightTyreDelta = prevRearRightTyreDelta;
	}

	/**
	 * Getter for the time step for a next state calculation (needed for callable)
	 * @return the time needed for an execution
	 */
	public float getDeltaTime() {
		return deltaTime;
	}

	/**
	 * Setter for the time interval needed for execution
	 * @param deltaTime the desired time
	 */
	public void setDeltaTime(float deltaTime) {
		if(!isValidDeltaTime(deltaTime)){
			throw new IllegalArgumentException(INVALID_TIME);
		}
		this.deltaTime = deltaTime;
	}

	private static boolean isValidDeltaTime(float deltaTime) {
		return deltaTime > 0.f;
	}

	/**
	 * Variable containing the autopilot outputs
	 */
	AutopilotOutputs autopilotOutputs;

	/**
	 * A variable containing the position of the drone
	 */
	private Vector position;

	/**
	 * A variable containing the velocity of the drone
	 */
	private Vector velocity;

	/**
	 * A variable containing the orientation of the drone, (heading, pitch, roll)
	 */
	private Vector Orientation;

	/**
	 * A variable containing the rotation vector of the drone (given in the world axis)
	 */
	private Vector rotationVector;

	/**
	 * A variable storing the compression of the front tyre
	 */
	private float prevFrontTyreDelta;

	/**
	 * A variable storing the compression of the rear left tyre
	 */
	private float prevRearLeftTyreDelta;

	/**
	 * A variable storing the compression of the rear right tyre
	 */
	private float prevRearRightTyreDelta;


	/**
	 * A variable containing the physics engine of the drone
	 */
	private PhysXEngine physXEngine;

	/**
	 * A variable containing the configuration of the drone
	 */
	private AutopilotConfig autopilotConfig;

	/**
	 * A variable containing the time step for next state
	 */
	private float deltaTime;

	/*
	 * Constants
	 */
	/**
	 * Constant: light speed, used in velocity check, maybe redundant
	 */
	private static float LIGHTSPEED = 300000000;

	/**
	 * Variable that stores the cubes representing the drone
	 */
	private Set<GraphicsObject> droneCubes = new HashSet<>();


	/**
	 * variable used for the size of the drone.
	 */
	private static final float scalingFactor = 1f;
	    
	/*
	 * Error Messages:
	 */

	private final static String THRUST_OUT_OF_RANGE = "The thrust is out of range: [0, this.maxThrust]";
	private final static String INCLINATION_OUT_OF_RANGE = "The inclination is out of range: [0, 2.PI[";
	public final static String VELOCITY_ERROR = "The velocity exceeds the upper limit";
	private final static String WING_EXCEPTION = "the wings are null references or the drone has already wings" +
			"attached to it";
	private final static String UNINITIALIZED_ENGINEMASS = "The mass of the engine is uninitialized";
	private final static String UNINITIALIZED_POINTMASS = "one or more of the point masses of the drone " +
			"have not been initialized yet";
	private final static String ILLEGAL_CONFIG = "The given configuration contains illegal values and or arguments";
	private final static String INVALID_TIMESTEP = "The provided time needs to be strictly positive";
	private final static String AUTOPILOT_CONFIG = "the autopilot has already been initialized";
	public static final String INVALID_TIME = "Invalid Time";
}
	/*
	code graveyard:
	 */
//
//
//	//------- Drone Controlling Methods -------
//
//	private static final float STANDARD_INCLINATION = (float)Math.PI/20;
//	private static final float STABLE_INCLINATION = (float)Math.PI/20;
//
//	/**
//	 * @author anthonyrathe
//	 */
//	public void clockRollStart(){
//		this.getLeftWing().setWingInclination(-STANDARD_INCLINATION);
//		this.getRightWing().setWingInclination(STANDARD_INCLINATION);
//	}
//
//	/**
//	 * @author anthonyrathe
//	 */
//	public void counterClockRollStart(){
//		this.getLeftWing().setWingInclination(STANDARD_INCLINATION);
//		this.getRightWing().setWingInclination(-STANDARD_INCLINATION);
//	}
//
//	/**
//	 * @author anthonyrathe
//	 */
//	public void stopRoll(){
//		this.getLeftWing().setWingInclination(STABLE_INCLINATION);
//		this.getRightWing().setWingInclination(STABLE_INCLINATION);
//	}
//
//	/**
//	 * @author anthonyrathe
//	 */
//	public void startTurnLeft(){
//		this.getVerticalStab().setWingInclination(-STANDARD_INCLINATION);
//	}
//
//	/**
//	 * @author anthonyrathe
//	 */
//	public void startTurnRight(){
//		this.getVerticalStab().setWingInclination(STANDARD_INCLINATION);
//	}
//
//	/**
//	 * @author anthonyrathe
//	 */
//	public void stopTurn(){
//		this.getVerticalStab().setWingInclination(0f);
//	}
//
//	/**
//	 * @author anthonyrathe
//	 */
//	public void startAscend(){
//		this.getHorizontalStab().setWingInclination(STANDARD_INCLINATION);
//	}
//
//	/**
//	 * @author anthonyrathe
//	 */
//	public void startDescend(){
//		this.getHorizontalStab().setWingInclination(-STANDARD_INCLINATION);
//
//	}
//
//	/**
//	 * @author anthonyrathe
//	 */
//	public void stopAscendDescend(){
//		this.getHorizontalStab().setWingInclination(0f);
//	}
//
//
//	/**
//	 * Changes the state of the drone to the next state, as calculated earlier by the autopilot.
//	 * State will remain the same if there is no queue-time.
//	 * @author anthonyrathe
//	 */
//	public void nextState() {
//		if (this.nextStateAvailable()) {
//			this.setThrust(this.getNextThrust());
//			this.getLeftWing().setWingInclination(this.getNextLeftWingInclination());
//			this.getRightWing().setWingInclination(this.getNextRightWingInclination());
//			this.getHorizontalStab().setWingInclination(this.getNextHorStabInclination());
//			this.getVerticalStab().setWingInclination(this.getNextVerStabInclination());
//		}
//	}
//
//	/**
//	 * A variable containing the amount of time (in seconds) the drone must wait before being able to call the autopilot
//	 */
//	private float queueTime;
//
//	/**
//	 * A getter that returns the current queueTime
//	 */
//	private float getQueueTime() {
//		return this.queueTime;
//	}
//
//	/**
//	 * A setter that sets the new queueTime
//	 *
//	 * @param newQueueTime the new amount of queue time (in seconds)
//	 * @author anthonyrathe
//	 */
//	private void setQueueTime(float newQueueTime) {
//		this.queueTime = newQueueTime;
//	}
//
//	/**
//	 * Method for telling whether or not a next state is available
//	 */
//	private boolean nextStateAvailable() {
//		return this.getQueueTime() > 0;
//	}
//
//	/**
//	 * A variable containing the new thrust of the drone, after queueTime has elapsed, as calculated by the autopilot
//	 */
//	private float nextThrust;
//
//	/**
//	 * A getter that returns the next amount of thrust
//	 */
//	private float getNextThrust() {
//		return this.nextThrust;
//	}
//
//	private void setNextThrust(float thrust){
//		this.nextThrust = thrust;
//	}
//
//	/**
//	 * A variable containing the current left wing inclination of the drone
//	 */
//	private float leftWingInclination;
//
//	/**
//	 * A variable containing the new left wing inclination of the drone, after queueTime has elapsed, as calculated by the autopilot
//	 */
//	private float nextLeftWingInclination;
//
//	/**
//	 * A getter that returns the next left wing inclination
//	 */
//	private float getNextLeftWingInclination() {
//		return this.nextLeftWingInclination;
//	}
//
//	private void setNextLeftWingInclination(float leftWingInclination){
//		this.nextLeftWingInclination = leftWingInclination;
//	}
//
//	/**
//	 * A variable containing the current right wing inclination of the drone
//	 */
//	private float rightWingInclination;
//
//	/**
//	 * A variable containing the new right wing inclination of the drone, after queueTime has elapsed, as calculated by the autopilot
//	 */
//	private float nextRightWingInclination;
//
//	/**
//	 * A getter that returns the next right wing inclination
//	 */
//	private float getNextRightWingInclination() {
//		return this.nextRightWingInclination;
//	}
//
//	private void setNextRightWingInclination(float rightWingInclination){
//		this.nextRightWingInclination = rightWingInclination;
//	}
//
//
//
//	/**
//	 * A variable containing the new horizontal stabilizers inclination of the drone, after queueTime has elapsed, as calculated by the autopilot
//	 */
//	private float nextHorStabInclination;
//
//	private void setNextHorStabInclination(float horStabInclination){
//		this.nextHorStabInclination = horStabInclination;
//	}
//
//	/**
//	 * A getter that returns the next horizontal stabilizer inclination
//	 */
//	private float getNextHorStabInclination() {
//		return this.nextHorStabInclination;
//	}
//
//	/**
//	 * A variable containing the new vertical stabilizers inclination of the drone, after queueTime has elapsed, as calculated by the autopilot
//	 */
//	private float nextVerStabInclination;
//
//	/**
//	 * A getter that returns the next vertical stabilizer inclination
//	 */
//	private float getNextVerStabInclination() {
//		return this.nextVerStabInclination;
//	}
//
//	private void setNextVerStabInclination(float verStabInclination){
//		this.nextVerStabInclination = verStabInclination;
//	}
//
//	/**
//	 * Moves the drone for a given amount of time, taking in account autopilot input.
//	 *
//	 * @param duration the amount of time the drone should be moved
//	 * @author anthonyrathe
//	 * @throws IOException
//	 */
//	public void evolve(float duration) throws IOException {
//		while (duration > 0) {
//			if (this.nextStateAvailable()) {
//				if (duration >= this.getQueueTime()) {
//					this.move(this.getQueueTime());
//					this.nextState();
//					duration = duration - this.getQueueTime();
//					this.setQueueTime((float) 0.0);
//				} else {
//					this.move(duration);
//					this.setQueueTime(getQueueTime() - duration);
//					duration = (float) 0.0;
//				}
//			} else {
//				AutopilotInputs input = updateAutopilotInput(duration);// TODO input stream
//				AutoPilot AP = this.getAutopilot();
//				AutopilotOutputs APO = AP.simulationStarted(autopilotConfig, input);
//				this.setThrust(APO.getThrust());
//				this.setLeftWingInclination(APO.getLeftWingInclination());
//				this.setRightWingInclination(APO.getRightWingInclination());
//				this.setHorStabInclination(APO.getHorStabInclination());
//				this.setNextVerStabInclination(APO.getVerStabInclination());
//				if (duration >= AP_CALC_TIME) {
//					duration = duration - AP_CALC_TIME;
//					this.move(AP_CALC_TIME);
//					this.nextState();
//				} else {
//					this.move(duration);
//					this.setQueueTime(AP_CALC_TIME - duration);
//					duration = (float) 0.0;
//				}
//			}
//		}
//	}
//
//
//	/*
//	########################## Transformations and projections ##########################
//	 */
//
//	/**
//	 * projects a vector of the drone axis on the world axis
//	 *
//	 * @param vector the vector to project
//	 * @return a new vector containing the projection
//	 * @author Martijn Sauwens
//	 */
//	public Vector droneOnWorld(Vector vector) {
//		return getDroneToWorldTransformMatrix().matrixVectorProduct(vector);
//	}
//
//	/**
//	 * projects a vector in the world axis to a vector in the drone axis
//	 *
//	 * @param vector the vector to project on the drone axis
//	 * @return a new vector containing the projection
//	 * @author Martijn Sauwens
//	 */
//	public Vector worldOnDrone(Vector vector) {
//		SquareMatrix transform = this.getDroneToWorldTransformMatrix();
//		SquareMatrix transpose = transform.transpose();
//
//		return transpose.matrixVectorProduct(vector);
//	}
//
//	/**
//	 * Calculates the transformation matrix for the drone to world axis
//	 *
//	 * @return a square matrix containing the transformation matrix for the current drone configuration;
//	 * @author Jasper Callaerts & Martijn Sauwens
//	 */
//	private SquareMatrix getDroneToWorldTransformMatrix() {
//		SquareMatrix heading = SquareMatrix.getHeadingTransformMatrix(this.getHeading());
//		SquareMatrix pitch = SquareMatrix.getPitchTransformMatrix(this.getPitch());
//		SquareMatrix roll = SquareMatrix.getRollTransformMatrix(this.getRoll());
//
//		return heading.matrixProduct(pitch).matrixProduct(roll);
//	}
//
//
//	/**
//	 * Calculates the projection of the provided rotation vector
//	 * @param rotationVector the rotation vector to be projected
//	 * @return a vector containing the heading, pitch and roll rotation vector
//	 * 		   the vector has the following format:
//	 * 		   Vector(headingRotation, pitchRotation, rollRotation)
//	 * 		   for more info concering the projection see the individual transformation functions
//	 * note: see notes on the calculations for clarification
//	 * @author Martijn Sauwens
//	 */
//	public Vector getRotationHPR(Vector rotationVector){
//		Vector headingRotation = this.getHeadingRotationVector(rotationVector);
//		Vector pitchRotation = this.getPitchRotationVector(rotationVector);
//		Vector rollRotation = this.getRollRotationVector(rotationVector);
//
//		Vector sumHeadingPitch = headingRotation.vectorSum(pitchRotation);
//		return sumHeadingPitch.vectorSum(rollRotation);
//	}
//
//
//	/**
//	 * Calculates the heading rotation vector component for a given rotation vector
//	 * the heading vector stands always with the y-axis in the world axis system
//	 * @param rotationVector the rotation vector to be projected
//	 * @return a vector containing the heading rotation (given in the same format as the orientation vectors)
//	 * 		   return new Vector( projectedHeading, 0, 0 )
//	 * @author Martijn Sauwens
//	 */
//	public Vector getHeadingRotationVector(Vector rotationVector){
//		float heading = this.getHeading();
//		float pitch = this.getPitch();
//
//		float part1 = rotationVector.getyValue();
//		float part2 = (float) (rotationVector.getzValue()*Math.cos(heading)*Math.tan(pitch));
//		float part3 = (float) (rotationVector.getxValue()*Math.sin(heading)*Math.tan(pitch));
//
//		float result = part1 + part2 + part3;
//
//		return new Vector(result, 0.0f, 0.0f);
//	}
//
//
//	/**
//	 * Calculates the pitch rotation vector component for a given rotation vector
//	 * the pitch vector stands parallel to the x axis after the heading transformation
//	 * has been applied to the world axis system
//	 * @param rotationVector the rotation vector to be projected
//	 * @return a vector containing the pitch rotation, given in the heading transformed world axis system
//	 * 		   return new Vector (0 , projectedPitch, 0)
//	 * @author Martijn Sauwens
//	 */
//	public Vector getPitchRotationVector(Vector rotationVector){
//		float heading = this.getHeading();
//
//		float part1 = (float) (rotationVector.getxValue()*Math.cos(heading));
//		float part2 = (float) (- rotationVector.getzValue()*Math.sin(heading));
//
//		float result = part1 + part2;
//
//		return new Vector(0.0f, result, 0.0f);
//	}
//
//	/**
//	 * Calculates the roll rotation vector component for a given rotation vector
//	 * the roll vector stands parallel to the the z- axis after the heading and the pitch transformation
//	 * has been applied to the world axis system
//	 * @param rotationVector the rotation vector to be projected
//	 * @return a vector containing the roll rotation, given in the double transformed world axis system
//	 * 		   return new Vector(0, 0, rollRotationVector)
//	 * @author Martijn Sauwens
//	 */
//	public Vector getRollRotationVector(Vector rotationVector){
//		float heading = this.getHeading();
//		float pitch = this.getPitch();
//
//		float part1 = (float) (rotationVector.getxValue()*Math.sin(heading));
//		float part2 = (float) (rotationVector.getzValue()*Math.cos(heading));
//
//		float result = (float) ((part1 + part2)/Math.cos(pitch));
//
//		return new Vector(0.0f, 0.0f, result);
//
//	}
//
//
//	/**
//	 * Calculates the angular acceleration in the heading, pitch, roll system for a given
//	 * angular acceleration vector
//	 * @param angularAccelerationVector the angular acceleration vector in the world axis system
//	 * @return a vector of format (HeadingAcceleration, PitchAcceleration, RollAcceleration)
//	 * @author Martijn Sauwens
//	 */
//	public Vector getAngularAccelerationHPR(Vector angularAccelerationVector){
//		float headingAcceleration = this.getHeadingAngularAcceleration(angularAccelerationVector);
//		float pitchAcceleration = this.getPitchAngularAcceleration(angularAccelerationVector);
//		float rollAcceleration = this.getRollAngularAcceleration(angularAccelerationVector);
//
//		return new Vector(headingAcceleration, pitchAcceleration, rollAcceleration);
//	}
//
//	/**
//	 * Calculates the heading angular acceleration component for the given angular acceleration vector
//	 * the heading acceleration vector stands parallel to the y-axis in the world axis system
//	 * @param angularAccelerationVector the vector containing the angular acceleration given in the
//	 *                                  world-axis system
//	 * @return the heading angular acceleration
//	 * @author Martijn Sauwens
//	 */
//	public float getHeadingAngularAcceleration(Vector angularAccelerationVector){
//		float heading = this.getHeading();
//		float pitch = this.getPitch();
//		Vector rotationVector = this.getRotationVector();
//		Vector AAVector = angularAccelerationVector;
//
//		float headingRotation = this.getHeadingRotationVector(rotationVector).getxValue();
//		float pitchRotation = this.getPitchRotationVector(rotationVector).getyValue();
//
//		//split up in parts for convenience
//		float part1 = (float) (AAVector.getyValue() + AAVector.getzValue()*Math.cos(heading)*Math.tan(pitch));
//		float part2 = (float) (-rotationVector.getzValue()*headingRotation*Math.sin(heading)*Math.tan(pitch));
//		float part3 = (float) (rotationVector.getzValue()*pitchRotation*Math.cos(heading)/(Math.cos(pitch)*Math.cos(pitch)));
//		float part4 = (float) (AAVector.getxValue()*Math.sin(heading)*Math.tan(pitch));
//		float part5 = (float) (rotationVector.getxValue()*headingRotation*Math.cos(heading)*Math.tan(pitch));
//		float part6 = (float) (rotationVector.getxValue()*pitchRotation*Math.sin(heading)/(Math.cos(pitch)*Math.cos(pitch)));
//
//		return part1 + part2 + part3 + part4 + part5 + part6;
//	}
//	/**
//	 * Calculates the pitch angular acceleration component for the given angular acceleration vector
//	 * the pitch acceleration vector stands parallel to the x-axis after the heading transformation
//	 * of the world axis system
//	 * @param angularAccelerationVector the vector containing the angular acceleration
//	 *                                  given in the world-axis system
//	 * @return the pitch angular acceleration
//	 * @author Martijn Sauwens
//	 */
//	public float getPitchAngularAcceleration(Vector angularAccelerationVector){
//		float heading = this.getHeading();
//		Vector rotationVector = this.getRotationVector();
//		Vector AAVector = angularAccelerationVector;
//
//		float headingRotation = this.getHeadingRotationVector(rotationVector).getxValue();
//		//split up in parts for convenience
//		float part1 = (float) (AAVector.getxValue()*Math.cos(heading) - rotationVector.getxValue()*headingRotation*Math.sin(heading));
//		float part2 = (float) (- AAVector.getzValue()*Math.sin(heading) - rotationVector.getzValue()*headingRotation*Math.cos(heading));
//		return part1 + part2;
//	}
//
//	/**
//	 * Calculates the roll angular acceleration component for the given angular acceleration vector
//	 * the roll acceleration stands parallel to the z-axis after the heading and the pitch transformation
//	 * of the world axis system (in that order)
//	 * @param angularAccelerationVector the vector containing the angular acceleration
//	 *                                  given in the world-axis system
//	 * @return the roll angular acceleration
//	 * @author Martijn Sauwens
//	 */
//	public float getRollAngularAcceleration(Vector angularAccelerationVector){
//		float heading = this.getHeading();
//		float pitch = this.getPitch();
//		Vector rotationVector = this.getRotationVector();
//		Vector AAVector = angularAccelerationVector;
//
//		float headingRotation = this.getHeadingRotationVector(rotationVector).getxValue();
//		float pitchRotation = this.getPitchRotationVector(rotationVector).getyValue();
//
//		//split up for convenience
//		float part1 = (float) (AAVector.getzValue()*Math.cos(heading)/Math.cos(pitch) -
//				rotationVector.getzValue()*headingRotation*Math.sin(heading)/Math.cos(pitch));
//		float part2 = (float) (rotationVector.getzValue()*pitchRotation*Math.sin(heading)*Math.sin(pitch)/(Math.cos(pitch)*Math.cos(pitch)));
//		float part3 = (float) (AAVector.getxValue()*Math.sin(heading)/Math.cos(pitch) + rotationVector.getxValue()
//				*headingRotation*Math.cos(heading)/Math.cos(pitch));
//		float part4 = (float) (rotationVector.getxValue()*pitchRotation*Math.sin(heading)*Math.sin(pitch)/(Math.cos(pitch)*Math.cos(pitch)));
//
//		return part1 + part2 + part3 + part4;
//	}
//
//
//	/**
//	 * calculates the next orientation based on the current orientation and the angular acceleration
//	 * @param deltaTime the time step taken
//	 * @param angularAcceleration the angular acceleration
//	 * @return a vector containing the orientation for the next step
//	 */
//	public Vector getNextOrientation(float deltaTime, Vector angularAcceleration){
//		//set up the needed variables
//		Vector currentOrientation = this.getOrientation();
//		Vector currentRotation = this.getRotationVector();
//		Vector RotationHPR = this.getRotationHPR(currentRotation);
//		Vector angularAccelerationHPR = this.getAngularAccelerationHPR(angularAcceleration);
//
//		//set up the next rotations
//		Vector rotationVelocity = RotationHPR.scalarMult(deltaTime);
//		Vector rotationAcceleration = angularAccelerationHPR.scalarMult(deltaTime*deltaTime/2.0f);
//
//		//return the next Orientation
//		return currentOrientation.vectorSum(rotationVelocity).vectorSum(rotationAcceleration);
//	}
//
//	/**
//	 * Calculates the rotation vector for the next time interval expressed in the world axis
//	 * @param deltaTime the time step to be taken
//	 * @param angularAcceleration the angular acceleration given in the world axis
//	 * @return a vector containing the next rotation vector given in the world axis
//	 */
//	public Vector getNextRotationVector(float deltaTime, Vector angularAcceleration){
//		Vector currentRotation = this.getRotationVector();
//		Vector deltaRotation = angularAcceleration.scalarMult(deltaTime);
//
//		return currentRotation.vectorSum(deltaRotation);
//	}
//
//	/**
//	 * Calculates the new velocity under the assumption that the acceleration remains constant
//	 * @param deltaTime the time between the steps
//	 * @param acceleration the (constant) acceleration between the steps
//	 * @return a vector containing the velocity of the next time step
//	 */
//	public Vector getNextVelocity(float deltaTime, Vector acceleration){
//		Vector currentVelocity = this.getVelocity();
//		Vector deltaVelocity = acceleration.scalarMult(deltaTime);
//		return currentVelocity.vectorSum(deltaVelocity);
//
//	}
//
//
//	/**
//	 * Calculates the next position under the assumption that the acceleration remains constant
//	 * @param deltaTime the time between the steps
//	 * @param acceleration the acceleration of the drone
//	 * @return a vector containing the position of the next time step
//	 */
//	public Vector getNextPosition(float deltaTime, Vector acceleration){
//		Vector currentPosition = this.getPosition();
//		Vector currentVelocity = this.getVelocity();
//		Vector deltaPosVelocityPart = currentVelocity.scalarMult(deltaTime);
//		Vector deltaPosAccelerationPart = acceleration.scalarMult(deltaTime*deltaTime/2.0f);
//
//		return currentPosition.vectorSum(deltaPosVelocityPart).vectorSum(deltaPosAccelerationPart);
//
//	}
//
//	/**
//	 * Calculates the angular acceleration based on the moment, moment of inertia and rotation vector
//	 * the resulting vector is projected onto the drone axis system
//	 *
//	 * @return a vector containing the angular acceleration of the drone
//	 * @author Martijn Sauwens
//	 */
//	public Vector calcAngularAcceleration() {
//		SquareMatrix inverseInertiaTensor = this.getInertiaTensor().invertDiagonal();
//		Vector moment = this.getTotalMomentDrone();
//		Vector rotationDrone = this.worldOnDrone(this.getRotationVector());
//		Vector rotationXImpulseMoment = rotationDrone.crossProduct(this.getImpulseMoment());
//
//		Vector momentDiff = moment.vectorDifference(rotationXImpulseMoment);
//		return inverseInertiaTensor.matrixVectorProduct(momentDiff);
//	}
//
//
//
//	/**
//	 * Calculates the total moment exerted on the drone in the drone axis system
//	 *
//	 * @return a vector containing the total exerted moment on the drone in the drone axis system
//	 * @author Martijn Sauwens
//	 * note: the gravity and thrust force are ignored because they are parallel to their force arms
//	 */
//	public Vector getTotalMomentDrone() {
//		WingPhysX[] wingArray = this.getWingArray();
//		int nbOfWings = wingArray.length;
//		//the array containing the lift vectors projected on the drone axis
//		Vector[] momentVectorsDrone = new Vector[nbOfWings];
//
//		for (int index = 0; index != nbOfWings; index++) {
//			WingPhysX currentWing = wingArray[index];
//			Vector liftOnDrone = this.worldOnDrone(currentWing.getLift());
//			Vector positionWing = currentWing.getRelativePosition();
//			Vector momentOnDrone = positionWing.crossProduct(liftOnDrone);
//			momentVectorsDrone[index] = momentOnDrone;
//		}
//
//		return Vector.sumVectorArray(momentVectorsDrone);
//	}
//
//	/**
//	 * calculates the moment of inertia of the drone on a given point in time, the impulse moment is given in the
//	 * drone axis system
//	 *
//	 * @return a vector containing the moment of inertia
//	 * @throws NullPointerException thrown if the Inertia tensor is not initialized
//	 * @author Martijn Sauwens
//	 */
//	public Vector getImpulseMoment() throws NullPointerException {
//		SquareMatrix inertiaTensor = this.getInertiaTensor();
//		if (inertiaTensor == null) {
//			throw new NullPointerException();
//		}
//
//		//calculate the angular velocity of the drone in the drone axis system
//		Vector rotationDrone = this.worldOnDrone(this.getRotationVector());
//
//		return inertiaTensor.matrixVectorProduct(rotationDrone);
//
//	}
///*
//	########################## Update methods for drone and autopilot ##########################
//	 */
//
//	/**
//	 * Method that checks if a suggested inclination is valid.
//	 *
//	 * @param inclination the inclinaton of a wing
//	 * @author anthonyrathe
//	 */
//	private boolean canHaveAsInclination(float inclination) {
//		return inclination >= -Math.PI/2 && inclination <= Math.PI/2;
//	}
//
//	/**
//	 * Method that sets the new left wing inclination.
//	 *
//	 * @param newLeftWingInclination the new inclination at which the left wing should be set
//	 * @author anthonyrathe
//	 */
//	private void setLeftWingInclination(float newLeftWingInclination) {
//		if (this.canHaveAsInclination(newLeftWingInclination)) {
//			this.getLeftWing().setWingInclination(newLeftWingInclination);
//		} else {
//			throw new IllegalArgumentException(Drone.INCLINATION_OUT_OF_RANGE);
//		}
//	}
//
//	/**
//	 * Method that gets the current left wing inclination.
//	 *
//	 * @author anthonyrathe & Martijn Sauwens
//	 */
//	public float getLeftWingInclination() {
//		return this.getLeftWing().getWingInclination();
//	}
//
//	/**
//	 * Method that sets the new right wing inclination.
//	 *
//	 * @param newRightWingInclination the new inclination at which the right wing should be set
//	 * @author anthonyrathe & Martijn Sauwens
//	 */
//	private void setRightWingInclination(float newRightWingInclination) {
//		if (this.canHaveAsInclination(newRightWingInclination)) {
//			this.getRightWing().setWingInclination(newRightWingInclination);
//		} else {
//			throw new IllegalArgumentException(Drone.INCLINATION_OUT_OF_RANGE);
//		}
//	}
//
//	/**
//	 * Method that gets the current right wing inclination.
//	 *
//	 * @author anthonyrathe & Martijn Sauwens
//	 */
//	public float getRightWingInclination() {
//		return this.getRightWing().getWingInclination();
//	}
//
//	/**
//	 * Method that sets the new horizontal stabilizer inclination.
//	 *
//	 * @param newHorStabInclination the new inclination at which the horizontal stabilizer should be set
//	 * @author anthonyrathe & Martijn Sauwens
//	 */
//	private void setHorStabInclination(float newHorStabInclination) {
//		if (this.canHaveAsInclination(newHorStabInclination)) {
//			this.getHorizontalStab().setWingInclination(newHorStabInclination);
//		} else {
//			throw new IllegalArgumentException(Drone.INCLINATION_OUT_OF_RANGE);
//		}
//	}
//
//	/**
//	 * Method that gets the current horizontal stabilizer inclination.
//	 *
//	 * @author anthonyrathe & Martijn Sauwens
//	 */
//	public float getHorStabInclination() {
//		return this.getHorizontalStab().getWingInclination();
//	}
//
//
//	/**
//	 * Method that sets the new vertical stabilizer inclination.
//	 *
//	 * @param newVerStabInclination the new inclination at which the vertical stabilizer should be set
//	 * @author anthonyrathe & Martijn Sauwens
//	 */
//
//	private void setVerStabInclination(float newVerStabInclination) {
//		if (this.canHaveAsInclination(newVerStabInclination)) {
//			this.getVerticalStab().setWingInclination(newVerStabInclination);
//		} else {
//			throw new IllegalArgumentException(Drone.INCLINATION_OUT_OF_RANGE);
//		}
//	}
//
//
//	/**
//	 * Method that gets the current vertical stabilizer inclination.
//	 *
//	 * @author anthonyrathe & Martijn Sauwens
//	 */
//
//	public float getVerStabInclination() {
//		return this.getVerticalStab().getWingInclination();
//	}
//
//
//	public void setAPImage(byte[] image){
//		APImage = image;
//	}
//
//	private byte[] getAPImage(){
//		return APImage;
//	}
//
//
//	/**
//	 * Getter of the right wing of the drone
//	 */
//	public HorizontalWingPhysX getRightWing() {
//		return rightWing;
//	}
//
//
//	/**
//	 * @param rightWing the desired right wing
//	 * @return true if and only if no other right wing is attached
//	 */
//	public boolean canHaveAsRightWing(HorizontalWingPhysX rightWing) {
//		return this.getRightWing() == null;
//	}
//
//	/**
//	 * Getter for the left wing of the drone
//	 */
//	public HorizontalWingPhysX getLeftWing() {
//		return leftWing;
//	}
//
//	/**
//	 * Setter for the left wing of the drone
//	 * @param leftWing the left wing of the drone
//	 * @author Martijn Sauwens
//	 */
//	public void setLeftWing(HorizontalWingPhysX leftWing) throws IllegalArgumentException, NullPointerException {
//		if (leftWing == null) {
//			throw new NullPointerException();
//		}
//
//		if (canHaveAsLeftWing(leftWing)) {
//			try {
//				leftWing.setDrone(this);
//				this.leftWing = leftWing;
//			} catch (IllegalArgumentException e) {
//				throw new IllegalArgumentException(e);
//			}
//		}
//		this.leftWing = leftWing;
//	}
//
//	/**
//	 * returns true if and only if there is no left wing attached to the drone
//	 *
//	 * @param leftWing the left wing of the drone
//	 */
//	public boolean canHaveAsLeftWing(HorizontalWingPhysX leftWing) {
//		return this.getLeftWing() == null;
//	}
//
//	/**
//	 * getter of the horizontal stabilizer
//	 *
//	 * @return the horizontal stabilizer wing
//	 */
//	public HorizontalWingPhysX getHorizontalStab() {
//		return horizontalStab;
//	}
//
//	/**
//	 * setter for the horizontal stabilizer
//	 *
//	 * @param horizontalStab the horizontal stabilizer to be attached to the drone
//	 * @author Martijn Sauwens
//	 */
//	public void setHorizontalStab(HorizontalWingPhysX horizontalStab) throws IllegalArgumentException, NullPointerException {
//		if (horizontalStab == null) {
//			throw new NullPointerException();
//		}
//		if (this.canHaveAsHorizontalStab(horizontalStab)) {
//			try {
//
//				horizontalStab.setDrone(this);
//				this.horizontalStab = horizontalStab;
//
//			} catch (IllegalArgumentException e) {
//				throw new IllegalArgumentException(e);
//			}
//		}
//	}
//
//	/**
//	 * Returns true if and only if there is no horizontal stabilizer attached to the drone
//	 *
//	 * @param horizontalStab the horizontal stabilizer
//	 * @author Martijn Sauwens
//	 */
//	public boolean canHaveAsHorizontalStab(HorizontalWingPhysX horizontalStab) {
//		return this.getHorizontalStab() == null;
//	}
//
//	/**
//	 * Getter for the vertical stabilizer
//	 * @author Martijn Sauwens
//	 */
//	public VerticalWingPhysX getVerticalStab() {
//		return verticalStab;
//	}
//
//	/**
//	 * Setter for the vertical stabilizer
//	 *
//	 * @param verticalStab the vertical stabilizer to be attached
//	 * @author Martijn Sauwens
//	 */
//	public void setVerticalStab(VerticalWingPhysX verticalStab) throws IllegalArgumentException, NullPointerException {
//		if (verticalStab == null) {
//			throw new NullPointerException();
//		}
//		if (this.canHaveAsVerticalStab(verticalStab)) {
//			try {
//				verticalStab.setDrone(this);
//				this.verticalStab = verticalStab;
//			} catch (IllegalArgumentException e) {
//				throw new IllegalArgumentException(e);
//			}
//		}
//	}
//
//---- START OF STREAM STUFF ---//
//	/**
//	 * Creates an AutopilotConfig file which contains all values from the AutopilotInterfaces config interface
//	 * @throws IOException
//	 * @author Jonathan Craessaerts
//	 */
//	public void setupAutopilotConfig()throws IOException{
//	    	DataOutputStream dataOutputStream =
//	                new DataOutputStream(new FileOutputStream(dataStreamLocationConfig));
//
//
//	        AutopilotConfig value = new AutopilotConfig() {
//	            public float getGravity() { return Drone.GRAVITY; }
//	            public float getWingX() { return  Math.abs(getRightWing().getRelativePosition().getxValue()); }
//	            public float getTailSize() { return Math.abs(getHorizontalStab().getRelativePosition().getzValue()); }
//	            public float getEngineMass() { return getEngineMass(); }
//	            public float getWingMass() { return getLeftWing().getMass(); }
//	            public float getTailMass() { return getLeftWing().getMass(); }
//	            public float getMaxThrust() { return getMaxThrust(); }
//	            public float getMaxAOA() { return getLeftWing().calcAngleOfAttack(); }
//	            public float getWingLiftSlope() { return getLeftWing().getLiftSlope(); }
//	            public float getHorStabLiftSlope() { return getHorizontalStab().getLiftSlope(); }
//	            public float getVerStabLiftSlope() { return getVerticalStab().getLiftSlope(); }
//	            public float getHorizontalAngleOfView() { return getHorizontalAngleOfView(); }
//	            public float getVerticalAngleOfView() { return getVerticalAngleOfView(); }
//	            public int getNbColumns() { return nbColumns; }
//	            public int getNbRows() { return nbRows; }
//	        };
//	        AutopilotConfigWriter.write(dataOutputStream, value);
//	    	dataOutputStream.close();
//	    }
//		/**
//		 * Creates an AutopilotInputs file which contains all values from the AutopilotInput interface
//		 * @throws IOException
//		 * @author Jonathan Craessaerts
//		 */
//	    public void setupAutopilotInputs()throws IOException{
//	    	DataOutputStream dataOutputStream =
//	                new DataOutputStream(new FileOutputStream(dataStreamLocationInputs));
//
//	    	Vector pos = getPosition();
//	    	Vector posOnWorld = droneOnWorld(pos);
//	    	float x = posOnWorld.getxValue();
//	    	float y = posOnWorld.getyValue();
//	    	float z = posOnWorld.getzValue();
//
//	    	float heading = getOrientation().getxValue();
//	    	float pitch = getOrientation().getyValue();
//	    	float roll = getOrientation().getzValue();
//
//
//	    	AutopilotInputs value = new AutopilotInputs() {
//	            public byte[] getImage() {
//					return getAPImage(); }
//	            public float getX() { return x; }
//	            public float getY() { return y; }
//	            public float getZ() { return z; }
//	            public float getHeading() { return heading; }
//	            public float getPitch() { return pitch; }
//	            public float getRoll() { return roll; }
//	            public float getElapsedTime() { // TODO
//	    				return (Float) null; }
//	        };
//
//	        AutopilotInputsWriter.write(dataOutputStream, value);
//
//	    	dataOutputStream.close();
//	    }
//	    /**
//	     * Variable for the filename that's created when making the AutopilotConfig datastream
//	     */
//	    private String dataStreamLocationConfig = "APConfig.txt";
//
//	    /**
//	     * Variable for the filename that's created when making the AutopilotInputs datastream
//	     */
//	    private String dataStreamLocationInputs = "APInputs.txt";

	// ---- END OF STREAM STUFF ---- //

//
//	/**
//	 * Getter for the engine mass of the drone
//	 */
//	public float getEngineMass() {
//		return engineMass;
//	}
//
//	/**
//	 * Checker for the mass of the engine
//	 *
//	 * @param engineMass the mass of the engine
//	 * @return true if and only if the mass is strictly positive
//	 */
//	public boolean canHaveAsEngineMass(float engineMass) {
//		return engineMass > 0;
//	}
//
//	/**
//	 * Getter for the engige position
//	 *
//	 * @return a vector containing the engine position
//	 */
//	public Vector getEnginePos() {
//		return this.enginePos;
//	}
//
//
//	/**
//	 * sets the engine position based on the configuration of the drone
//	 *
//	 * @post the center of mass is (0,0,0) in the drone's axis.
//	 * @author Martijn Sauwens
//	 */
//	private void setEnginePosition() throws NullPointerException {
//		HorizontalWingPhysX horizontalStab = this.getHorizontalStab();
//		VerticalWingPhysX verticalStab = this.getVerticalStab();
//		float engineMass = this.getEngineMass();
//
//		if (horizontalStab == null || verticalStab == null)
//			throw new NullPointerException();
//
//		if (this.getEngineMass() == 0.0f)
//			throw new IllegalArgumentException(UNINITIALIZED_ENGINEMASS);
//
//		float horizontalStabPos = horizontalStab.getRelativePosition().getzValue();
//		float verticalStabPos = verticalStab.getRelativePosition().getzValue();
//		float horizontalStabMass = horizontalStab.getMass();
//		float verticalStabMass = verticalStab.getMass();
//
//		this.enginePos = new Vector(0, 0, -(horizontalStabMass * horizontalStabPos + verticalStabMass * verticalStabPos) / engineMass);
//	}
//
//	/**
//	 * Getter for the inertia tensor, the tensor is given in the drone axis system
//	 * @return a square (diagonal) matrix containing the inertia tensor of the drone
//	 */
//	public SquareMatrix getInertiaTensor() {
//		return this.inertiaTensor;
//	}
//
//	/**
//	 * Calculates the inertia tensor based on the point masses given to the system
//	 * The inertia tensor is calculated in the drone axis system.
//	 *
//	 * @throws IllegalArgumentException thrown if not all the parts of the drone are initialized
//	 * post: new inertiaTensor == SquareMatrix({Ixx, 0.0f, 0.0f,
//	 * 0.0f, Iyy, 0.0f,
//	 * 0.0f, 0.0f, Izz});
//	 * with Ixx sum(mi*(yi^2 + zi^2), Iyy = mi*(xi^2 + zi^2), Izz = mi*(xi^2 + yi^2)
//	 * and  mi the mass of the selected point, xi, yi, zi the coordinate of the point mass
//	 * for more info: https://nl.wikipedia.org/wiki/Traagheidsmoment
//	 * @author Martijn Sauwens
//	 */
//	private void setInertiaTensor() throws IllegalArgumentException {
//		float Ixx = 0;
//		float Iyy = 0;
//		float Izz = 0;
//
//		if (!this.canCalcInertiaTensor())
//			throw new IllegalArgumentException(UNINITIALIZED_POINTMASS);
//
//		WingPhysX[] wingArray = this.getWingArray();
//
//		// calculate the inertia caused by the wings
//		for (WingPhysX wing : wingArray) {
//			float mass = wing.getMass();
//			Vector wingPos = wing.getRelativePosition();
//			float xValue = wingPos.getxValue();
//			float yValue = wingPos.getyValue();
//			float zValue = wingPos.getzValue();
//
//			Ixx += mass * (yValue * yValue + zValue * zValue);
//			Iyy += mass * (xValue * xValue + zValue * zValue);
//			Izz += mass * (xValue * xValue + yValue * yValue);
//		}
//
//		//calculate the inertia caused by the engine
//		float engineMass = this.getEngineMass();
//		Vector enginePos = this.getEnginePos();
//		float zValueEngine = enginePos.getzValue();
//
//
//		//the engine is always located on the z axis
//		Ixx += engineMass * zValueEngine * zValueEngine;
//		Iyy += engineMass * zValueEngine * zValueEngine;
//
//		float[] inertiaArray = {Ixx, 0.0f, 0.0f,
//				0.0f, Iyy, 0.0f,
//				0.0f, 0.0f, Izz};
//		this.inertiaTensor = new SquareMatrix(inertiaArray);
//
//	}
//
//	/**
//	 * checks if the drone is in the right state to calculate the inertia tensor
//	 *
//	 * @return true if and only if all the point masses of the drone are initialized and the
//	 * inertia tensor hasn't been calculated before (immutable character of the tensor)
//	 * @author Martijn Sauwens
//	 */
//	private boolean canCalcInertiaTensor() {
//		if (this.inertiaTensor != null)
//			return false;
//		if (this.getEnginePos() == null)
//			return false;
//
//		for (WingPhysX wing : this.getWingArray()) {
//			if (wing == null)
//				return false;
//		}
//
//		return true;
//	}
//
//	/**
//	 * Calculates the total mass of the drone
//	 * @return the mass of the wings and the engine
//	 * @author Martijn Sauwens
//	 */
//	public float getTotalMass(){
//		float totalMass = this.getEngineMass();
//
//		WingPhysX[] wingArray = this.getWingArray();
//		for(WingPhysX wing: wingArray){
//			totalMass += wing.getMass();
//		}
//
//		return totalMass;
//	}

