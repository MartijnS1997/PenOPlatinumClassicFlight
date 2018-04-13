
package internal.Autopilot;

import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;


import static java.lang.Math.*;

import org.omg.CORBA.SetOverrideTypeHelper;


/**
 * Created by Martijn on 18/02/2018, extended by Jonathan on 12/3/2018
 * A class of landing controllers, responsible for controlling the landing of the drone
 */

public class AutopilotLandingController extends Controller {

    public AutopilotLandingController(AutoPilot autopilot) {
        // implement constructor
        super(autopilot);
//        this.getVelocityPID().setSetPoint(this.referenceVelocity);
//        this.getOrientationPID().setSetPoint(this.referenceOrientation);
//        this.getAltitudePID().setSetPoint(this.referenceAltitude);

    }

    /**
     * Returns true if the plane came to a standstill on the ground
     * @param inputs the current inputs (this is the base of the check)
     * @return true if the approximate velocity is below velocity threshold
     */
    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 inputs) {
        Vector velocityApprox = this.getVelocityApprox(this.getCurrentInputs(), inputs);
        return velocityApprox.getSize() <= MAXIMUM_LANDING_VELOCITY;
    }

    /**
     * Generates the control actions for the autopilot
     *
     * @param inputs the inputs of the autopilot
     * @return the control actions
     */
    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {
        this.setCurrentInputs(inputs);

        if(this.isFirstControlCall()){
            this.setStartElapsedTime(this.getCurrentInputs());
            this.setFirstControlCall();
        }

        LandingPhases landingPhase = this.getCurrentLandingPhase();
        ControlOutputs outputs = new ControlOutputs();

        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();

//        System.out.println("Current velocity: " + this.getVelocityApprox(prevInputs, getCurrentInputs()));

        switch (landingPhase){
            case STABILIZE:
                this.stabilizeFlight(outputs);
                break;
            case RAPID_DESCEND:
                this.getRapidDescendControls(outputs);
//                System.out.println("rapidDescend");
                break;
            case SOFT_DESCEND:
//                System.out.println("soft descend");
                this.getSoftDescendControls(outputs);
                break;
            case SLOW_DOWN:
//            	System.out.println("Slowdown");
            	this.slowDown(outputs);
            	break;
        }

        AutopilotInputs_v2 previousInputs = getPreviousInputs();
        angleOfAttackControl(outputs, previousInputs, inputs);
        // System.out.println(Controller.extractPosition(inputs).getyValue());

        //System.out.println(outputs);
        return outputs;

    }

    private LandingPhases getCurrentLandingPhase(){
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        //check if the rapid descend phase was started
        if(!this.isHasStartedRapidDescend()){
            //check if the flight is stable
        	if(!slowEnough(currentInputs)){
        		return LandingPhases.SLOW_DOWN;
        	}
        	else if(!mayInitializeLanding(currentInputs)){
                //if not, stabilize
                return LandingPhases.STABILIZE;
            }
            //System.out.println("Drone is stabilized");
            this.setHasStartedRapidDescend();
            this.configureSoftDescendHeight(currentInputs);
            return LandingPhases.RAPID_DESCEND;


        //check if the soft landing phase was started
        }else if(!this.isHasStartedSoftDescend()){
            Vector position = Controller.extractPosition(currentInputs);
            //check if we are low enough to initiate the soft descend
//            System.out.println("Current y:" + position.getyValue());
//            System.out.println("transition: " + getStartSoftDescendPhaseHeight());
            if(position.getyValue() > this.getStartSoftDescendPhaseHeight()){
                //if not continue rapid descend

                return LandingPhases.RAPID_DESCEND;
            }
            //if not start the soft descend
            this.setHasStartedSoftDescend();
//            System.out.println("Soft Started: " + currentInputs.getY());

            return LandingPhases.SOFT_DESCEND;
        }else{
//            System.out.println("CurrentS y: " + currentInputs.getY());
            return LandingPhases.SOFT_DESCEND;
        }
    }

    /**
     * Configures the soft descend height of the soft landing phase of the landing controller
     * only call after leaving the stabilizer stage and before the first call of rapid descend controls
     * @param inputs the inputs to infer the height from
     */
    private void configureSoftDescendHeight(AutopilotInputs_v2 inputs){
        //get the current height
        float height = inputs.getY();
        //then set the soft descend phase height
        float softStartHeight = SOFT_DESCEND_MIN_START_HEIGHT;
        this.setStartSoftDescendPhaseHeight(softStartHeight);

    }

    /**
     * Generates the outputs for the rapid descend phase of the landing
     * @param outputs the outputs to write to
     */
    private void getRapidDescendControls(ControlOutputs outputs){
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
        //set the setpoint
        PIDController pitchPID = this.getPitchPIDController();
        pitchPID.setSetPoint(RAPID_DESCEND_PHASE_REF_PITCH);
        //call the pitch and roll PID
        this.pitchStabilizer(outputs, currentInputs, prevInputs);
        this.rollStabilizer(outputs, currentInputs, prevInputs);
        //set the thrust
        outputs.setThrust(RAPID_DESCEND_THRUST);
        //we are finished here
    }

    /**
     * Generates the control outputs for the soft landing phase
     * @param outputs the soft landing phase
     */
    private void getSoftDescendControls(ControlOutputs outputs){
//        System.out.println("querying soft descend");
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
        AutopilotConfig config = this.getConfig();
        //set the setPoint
//        PIDController pitchPID = this.getPitchPIDController();
//        pitchPID.setSetPoint(SOFT_DESCEND_PHASE_REF_PITCH);
        //call the pitch and roll PID
        this.pitchStabilizer(outputs, currentInputs, prevInputs);
        this.rollStabilizer(outputs, currentInputs, prevInputs);
        //change the main wing inclination
        outputs.setRightWingInclination(outputs.getRightWingInclination() - SOFT_DESCEND_PHASE_MAIN_WING_INCLINATION_DELTA);
        outputs.setLeftWingInclination(outputs.getLeftWingInclination() - SOFT_DESCEND_PHASE_MAIN_WING_INCLINATION_DELTA);
        outputs.setThrust(SOFT_DESCEND_THRUST);
        outputs.setRightBrakeForce(config.getRMax());
        outputs.setLeftBrakeForce(config.getRMax());
        outputs.setFrontBrakeForce(config.getRMax());
    }




    /**
     * Checks if the drone may initialize landing
     * @param inputs the current inputs of the autopilot
     * @return true if and only if the drone is stabilized and has stabilized for at least minimal stabilizing time
     */
    private boolean mayInitializeLanding(AutopilotInputs_v2 inputs) {
        return (this.getCurrentInputs().getElapsedTime() - this.getStartElapsedTime()) > MINIMAL_STABILIZING_TIME && this.isStabilized(inputs);
    }


    /**
     * Stabilizes the flight before commencing the landing sequence
     * @param outputs the output object to write the control actions to
     */
    private void stabilizeFlight(ControlOutputs outputs){
        //get the current and previous inputs
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();

        //stabilize the pitch and the roll
        //super.rollControl(outputs, currentInputs);
        pitchStabilizer(outputs, currentInputs, prevInputs);
        rollStabilizer(outputs, currentInputs, prevInputs);
       	outputs.setThrust(STABILIZING_THURST);


    }

    private void slowDown(ControlOutputs outputs){

        outputs.setThrust(0);
	    outputs.setRightWingInclination(MAIN_MAX_INCLINATION);
	    outputs.setLeftWingInclination(MAIN_MAX_INCLINATION);

    }

    /**
     * Stabilizes the pitch of the drone
     * @param outputs the outputs to write to
     * @param currentInputs the current inputs to extract the flight data from
     * @param prevInputs the previous inputs to extract the flight data from
     */
    private void pitchStabilizer(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs) {
        //stabilize the pitch
        //extract the current orientation
        Vector orientation = Controller.extractOrientation(currentInputs);
        float pitch = orientation.getyValue();
//        System.out.println(pitch);
        PIDController pitchPid =this.getPitchPIDController();
        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        float PIDControlActions =  pitchPid.getPIDOutput(pitch, deltaTime);
        //System.out.println("Pitch result PID" + PIDControlActions);
        //adjust the horizontal stabilizer
        float horizontalInclination = this.getStabilizerStableInclination() - PIDControlActions;
        horizontalInclination = signum(horizontalInclination) * min(abs(horizontalInclination), HOR_STABILIZER_MAX);
        outputs.setHorStabInclination(horizontalInclination);
    }

    /**
     * Stabilizer for the roll of the drone
     * @param outputs the outputs to write to
     * @param currentInputs the current inputs to extract the flight data from
     * @param prevInputs the previous inputs to extract the flight data from
     */
    private void rollStabilizer(ControlOutputs outputs, AutopilotInputs_v2 currentInputs, AutopilotInputs_v2 prevInputs){
        Vector orientation = Controller.extractOrientation(currentInputs);
        float roll = orientation.getzValue();
        PIDController rollPid = this.getRollPIDController();
        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        float PIDControlActions = rollPid.getPIDOutput(roll, deltaTime);
        float rightMainWing = this.getMainStableInclination() + PIDControlActions;
        float leftMainWing = this.getMainStableInclination() - PIDControlActions;

        outputs.setRightWingInclination(capMainWingInclination(rightMainWing));
        outputs.setLeftWingInclination(capMainWingInclination(leftMainWing));

    }

    /**
     * Enforces a maximum onto the main wing inclination based on the MAIN_CAP_DELTA_INCLINATION variable
     * @param inclination the inclination to adjust
     * @return if the inclination is in the range getMainStableInclination +- MAIN_CAP_DELTA_INCLINATION
     *         the same value as the inclination is returned, if it exceeds the border value, the inclination
     *         is set to the border value
     */
    private float capMainWingInclination(float inclination){
        //first determine the lower cap:
        float lowerCap = this.getMainStableInclination() - MAIN_CAP_DELTA_INCLINATION;
        float upperCap = this.getMainStableInclination() + MAIN_CAP_DELTA_INCLINATION;
        if(inclination < lowerCap){
            return lowerCap;
        }
        if(inclination > upperCap){
            return upperCap;
        }
        return inclination;
    }

    /**
     * Checks if the roll and the pitch are stabilized, they are if they are within the appropriate margins
     * (see the constants below)
     * @param inputs the inputs to read the current state from
     * @return true if and only if the roll and the pitch are within the allowed margin
     */
    private boolean isStabilized(AutopilotInputs_v2 inputs){
        //check if the roll is within limits
        boolean rollStabilized = this.isRollStabilized(inputs);
        boolean pitchStabilized = this.isPitchStabilized(inputs);

        return rollStabilized && pitchStabilized;

    }

    /**
     * Checks if the pitch is stabilized , if so, we're finished
     * @param inputs the inputs to check
     * @return true if the pitch is within acceptable margins
     */
    private boolean isPitchStabilized(AutopilotInputs_v2 inputs){
        //extract the orientation
        Vector orientation = Controller.extractOrientation(inputs);
        float pitch = orientation.getyValue();
        return abs(pitch) < PITCH_STABILIZING_MARGIN;
    }

    /**
     * Checks if the roll is stabilized, if so, continue to the pitch control
     * @param inputs the inputs to check
     * @return true if the roll is within acceptable regions
     */
    private boolean isRollStabilized(AutopilotInputs_v2 inputs){
        Vector orientation = Controller.extractOrientation(inputs);
        float roll = orientation.getzValue();
        return abs(roll) < ROLL_STABILIZING_MARGIN;
    }

    //TODO
    private boolean slowEnough(AutopilotInputs_v2 inputs){
    	return (this.getVelocityApprox(this.getPreviousInputs(), getCurrentInputs()).getzValue() > LANDING_SPEED);
    }


    private final float LANDING_SPEED = -40f;

//    private void setHorizontalStabilizer(ControlOutputs outputs){
//        //we want to go for zero (stable inclination of the horizontal stabilizer is zero), so the corrective action needs also to be zero
//        Vector orientation = Controller.extractOrientation(this.getCurrentInputs());
//        Vector orientationPID = this.getOrientationPID().getPIDOutput(orientation, this.getCurrentInputs().getElapsedTime());
//        //extract the pitch (positive is upward looking, negative is downward looking)
//        float pitch = orientationPID.getyValue();
//        float pitchConstant = 2;
//        //calculate the desired action of the stabilizer(negative is upward movement, positive downward)
//        float desiredAngle = (float) (-pitch/PI*pitchConstant);//the pitch/PI is to get a % of the pitch that we're off
//        float outputInclination = min(abs(HOR_STABILIZER_MAX), abs(desiredAngle));
//        outputs.setHorStabInclination(outputInclination*signum(desiredAngle));
//    }


    /**
     * Constants
     */
    private final static float STOP_VELOCITY = 0.0f;
    private final static float STANDARD_THRUST = 128.41895f * 3.5f;
    private final static float WING_INCL = (float) (PI / 180);
    private final static float HORIZONTAL_STABILIZER = (float) (PI / (180));
    private final static float PITCH_THRESHOLD = (float) (5 * PI / 180);
    private static final float INIT_LANDING_HEIGHT = 8f;
    private final static float MAIN_STABLE = (float) (7*PI/180);
    private final static float STABILIZER_STABLE = 0;
    private final static float ROLL_THRESHOLD = (float)(3*PI/180);
    private final static float INCLINATION_AOA_ERROR_MARGIN = (float)(3*PI/180);
    private final static float HOR_STABILIZER_MAX = (float)(7*PI/180);
    private final static float PLANE_HEIGHT_FROM_GROUND = 1.2f;
    private final static float MAXIMUM_LANDING_VELOCITY = 1f;
    private  static final float MAIN_MAX_INCLINATION = (float) (10*PI/180);

    //stabilizing constants &instances
    private final static float STABILIZING_THURST = 550f;
    private final static float ROLL_STABILIZING_MARGIN = (float) (2*PI/180);
    private final static float PITCH_STABILIZING_MARGIN = (float) (2*PI/180);
    private final static float MAIN_CAP_DELTA_INCLINATION = (float) (3*PI/180);
    private final static float MINIMAL_STABILIZING_TIME = 2f;
    private final static float STABILIZING_PHASE_REF_PICTCH = 0f;

    //the pitch controller used in all stages of the flight but with different set points
    private static float PITCH_GAIN = 1;
    private static float PITCH_INTEGRAL = 0.2f;
    private static float PITCH_DERIVATIVE = 0.0f;
    private PIDController pitchPIDController = new PIDController(PITCH_GAIN, PITCH_INTEGRAL, PITCH_DERIVATIVE);

    //the roll PID controller and its constants, used in all phases of the flight
    private static float ROLL_GAIN = 1;
    private static float ROLL_INTEGRAL = 0.2f;
    private static float ROLL_DERIVATIVE = 0;
    private PIDController rollPIDController = new PIDController(ROLL_GAIN, ROLL_INTEGRAL, ROLL_DERIVATIVE);

    /**
     * Flag to indicate if the first control action was called
     */
    private boolean firstControlCall = true;

    /**
     * The elapsed time at the start of the simulation
     */
    private float startElapsedTime;

    /**
     * Flag to indicate if the rapid landing was initialized
     */
    private boolean hasStartedRapidDescend = false;

    /**
     * Flag to indicate if the soft landing was initialized
     */
    private boolean hasStartedSoftDescend = false;

    //rapid descend phase constants and controllers
    private final static float RAPID_DESCEND_PHASE_REF_PITCH = (float) (-10*PI/180);
    private final static float RAPID_DESCEND_THRUST = 0f;

    //soft descend constants
    private final static float SOFT_DESCEND_PHASE_REF_PITCH = (float) (-2*PI/180);
    private final static float SOFT_DESCEND_PHASE_MAIN_WING_INCLINATION_DELTA = (float)(-4*PI/180);
    private final static float SOFT_DESCEND_THRUST = 0f;
    private final static float SOFT_DESCEND_MIN_START_HEIGHT = 5f;
    //Bartje was hier :)

    /**
     * The height at the start at the descend
     */
    private float heightAtStartOfDescend;

    /**
     * The height to reach before we initiate the soft landing phase
     */
    private float startSoftDescendPhaseHeight;


    private Vector referenceVelocity = new Vector(0,0,-STOP_VELOCITY);
    private VectorPID orientationPID = new VectorPID(1.0f, 0f, 0f);
    private Vector referenceOrientation = new Vector();
    private PIDController altitudePID = new PIDController(1.0f, 0.1f,0.2f);
    private float referenceAltitude = 10f;
    private boolean startedDescend = false;
    /**
     * true if the controller is called for control actions for the first time
     * @return true if and only if the controller is queried for the first time
     */
    public boolean isFirstControlCall() {
        return firstControlCall;
    }

    /**
     * Sets the flag for the first call (one way use)
     */
    public void setFirstControlCall() {
        this.firstControlCall = false;
    }

    /**
     * Getter for the elapsed time at the moment of the first control actions query
     * @return the elapsed time at the first control query
     */
    private float getStartElapsedTime() {
        return startElapsedTime;
    }

    /**
     * Setter for the elapsed time at the moment of the first control actions query
     * @param inputs the inputs at the moment of the first invocation
     */
    private void setStartElapsedTime(AutopilotInputs_v2 inputs) {
        if(!this.isFirstControlCall())
            throw new IllegalArgumentException("not first call");
        this.startElapsedTime = inputs.getElapsedTime();
    }

    /**
     * Getter for the rapid descend flag
     * @return true if the controller has initiated the rapid descend phase
     */
    public boolean isHasStartedRapidDescend() {
        return hasStartedRapidDescend;
    }

    /**
     * Setter for the rapid descend phase flag
     */
    public void setHasStartedRapidDescend() {
        this.hasStartedRapidDescend = true;
    }

    /**
     * Getter for the soft descend phase flag
     * @return true if the controller has initiated the soft descend phase
     */
    public boolean isHasStartedSoftDescend() {
        return hasStartedSoftDescend;
    }

    /**
     * Setter for the soft descend phase flag
     */
    private void setHasStartedSoftDescend() {
        this.hasStartedSoftDescend = true;
    }

    /**
     * Getter for the height that the drone was at when initiating the rapid descend
     * @return a float corresponding to the height of the start of the rapid descend
     */
    private float getHeightAtStartOfDescend() {
        return heightAtStartOfDescend;
    }

    /**
     * setter for the height when the drone started the rapid descend
     * @param inputs the inputs to infer the height from
     */
    private void setHeightAtStartOfDescend(AutopilotInputs_v2 inputs) {
        this.heightAtStartOfDescend = inputs.getY();
    }

    /**
     * Getter for the height where the controller should start the soft landing phase
     * @return a float corresponding to the height of the start of the soft landing phase
     */
    private float getStartSoftDescendPhaseHeight() {
        return startSoftDescendPhaseHeight;
    }

    /**
     * Setter for the the height at which to initialize the soft landing
     * @param startSoftDescendPhaseHeight the soft landing height
     */
    private void setStartSoftDescendPhaseHeight(float startSoftDescendPhaseHeight) {
        this.startSoftDescendPhaseHeight = startSoftDescendPhaseHeight;
    }

//    private VectorPID getVelocityPID() {
//        return this.velocityPID;
//    }

    private VectorPID getOrientationPID() {
        return orientationPID;
    }

    private PIDController getAltitudePID() {
        return altitudePID;
    }

    private PIDController getPitchPIDController() {
        return pitchPIDController;
    }

    public PIDController getRollPIDController(){
        return rollPIDController;
    }

    /**
     * Getter for the flag to indicate if the descend is initiated
     * @return the value of the flag
     */
    private boolean isStartedDescend() {
        return startedDescend;
    }

    /**
     * Toggles the start descend flag, once we start to descend, there is no way back
     */
    private void setStartedDescend() {
        this.startedDescend = true;
    }

    //TODO implement these methods accordingly
    @Override
    protected float getMainStableInclination() {
        return MAIN_STABLE;
    }

    @Override
    protected float getStabilizerStableInclination() {
        return STABILIZER_STABLE;
    }

    @Override
    protected float getRollThreshold() {
        return ROLL_THRESHOLD;
    }

    @Override
    protected float getInclinationAOAErrorMargin() {
        return INCLINATION_AOA_ERROR_MARGIN;
    }

    @Override
    protected float getStandardThrust() {
        return STANDARD_THRUST;
    }

    /**
     * Enumerations for the landing phases
     */
    private enum LandingPhases {
        STABILIZE,RAPID_DESCEND, SOFT_DESCEND, SLOW_DOWN;
    }
}


