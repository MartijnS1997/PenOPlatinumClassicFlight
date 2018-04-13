package internal.Autopilot;

import AutopilotInterfaces.Autopilot;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import static java.lang.Math.*;

/**
 * Created by Martijn on 18/02/2018.
 * A class of takeoff controllers, responsible for controlling the takeoff of the drone
 */
public class AutopilotTakeoffController extends Controller{

    public AutopilotTakeoffController(AutoPilot autopilot){
        //implement constructor
        super(autopilot);
//        this.getVelocityPID().setSetPoint(this.referenceVelocity);
//        this.getOrientationPID().setSetPoint(this.referenceOrientation);
//        this.getAltitudePID().setSetPoint(this.referenceAltitude);
    }


    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 inputs) {
        Vector position = Controller.extractPosition(inputs);
        Vector target = getTarget();
        if(position.distanceBetween(target) < TARGET_DISTANCE_THRESHOLD){
            System.out.println("altitude reached");
            System.out.println("Z - position: " + inputs.getZ());
            System.out.println("Approx velocity: " + this.getVelocityApprox(this.getPreviousInputs(), this.getCurrentInputs()));
            //load the image in and check if there are any cubes in sight
            //if not use this controller further
            AutoPilotCamera APCamera = this.getAutopilot().getAPCamera();
            APCamera.loadNewImage(inputs.getImage());
            return APCamera.getAllCubeCenters().size() > 0;
        }

        return false;
    }

    /**
     * Controls the thrust of the takeoff controller
     * @param outputs the outputs for the drone
     */
    private void setThrust(ControlOutputs outputs){
        //get the maximal thrust
        float maxThrust = this.getConfig().getMaxThrust();
        float pitch = this.getCurrentInputs().getPitch();
        float thrustCoeff = 50;
        float outputThrust = (float) (STANDARD_THRUST + pitch*thrustCoeff*180/PI);
        outputs.setThrust(max(min(outputThrust, maxThrust), 0f));

    }


    /**
     * Determines the control outputs for the main wings in takeoff
     * @param outputs the outputs for the drone (are overwritten by this method)
     */
    private void setMainWing(ControlOutputs outputs){

        outputs.setLeftWingInclination(MAIN_STABLE);
        outputs.setRightWingInclination(MAIN_STABLE);
    }

    /**
     * Generates the control actions to stabilize the drone and pitch towards the first target of the drone
     * @param outputs the outputs to write the control actions to
     */
    private void stabilizerActions(ControlOutputs outputs){
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
        Vector orientation = Controller.extractOrientation(currentInputs);
        float pitchAngle = -this.pitchToTarget(currentInputs);
        //the pitch should be 0;
        //System.out.println(pitch);
        PIDController pitchPid = this.getPitchPID();
        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        float PIDControlActions =  pitchPid.getPIDOutput(pitchAngle,  deltaTime);
        //System.out.println("Pitch result PID" + PIDControlActions);
        //adjust the horizontal stabilizer
        float horizontalInclination = this.getStabilizerStableInclination() - PIDControlActions;
        horizontalInclination = signum(horizontalInclination) * min(abs(horizontalInclination), HOR_STABILIZER_MAX);
        outputs.setHorStabInclination(horizontalInclination);
    }

    /**
     * Calculates the angle between the heading vector of the drone (0,0,-1) and the reference vector
     * (the vector between the current position of the drone and the target)
     * while indicating the direction to steer in the signum of the angle (positive means upward, negative downward)
     * @param inputs the inputs to extract the current drone state from
     * @return a positive angle if upward steering is required, a negative one if downward is necessary
     */
    private float pitchToTarget(AutopilotInputs_v2 inputs){
        //get the target
        Vector target = this.getTarget();
        //get the current position
        Vector dronePos = Controller.extractPosition(inputs);
        //calculate the diff vector to the target
        Vector diffVectorWorld = target.vectorDifference(dronePos);
        //get the orientation of the drone
        Vector orientation = Controller.extractOrientation(inputs);
        //transform the world diff vector to the drone axis system
        Vector diffVectorDrone = PhysXEngine.worldOnDrone(diffVectorWorld, orientation);
        //get the heading vector in the drone axis system
        Vector heading = new Vector(0,0,-1);
        //project the difference vector onto the yz-plane of the drone axis system
        Vector yzNormal = new Vector(1,0,0);
        Vector projDiffVector = diffVectorDrone.orthogonalProjection(yzNormal);
        //calculate the angle between the heading and the ref vector
        float angle = abs(projDiffVector.getAngleBetween(heading));
        //get the direction by normalizing the cross product a positive cross product means flying up, neg down
        float direction = signum(heading.crossProduct(projDiffVector).getxValue()); // get the x-value for the direction

        float result = angle*direction;
        //check for NaN
        if(Float.isNaN(result)){
            //default output
            return 0;
        }
        //otherwise return as normal
        return result;


    }



    /**
     * Checks if the desired altitude has been reached
     */
    private void checkDesiredAltitude(){
        if(this.getCurrentInputs().getY() >= this.getTarget().getyValue()){
            this.setReachedDesiredAltitude();
        }
    }


    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {
//        System.out.println("Altitude: " + (inputs.getY() - this.getConfig().getWheelY() - this.getConfig().getTyreRadius()));
//        System.out.println("z-pos: " + inputs.getZ());
        this.setCurrentInputs(inputs);
        ControlOutputs outputs = new ControlOutputs();
        this.checkDesiredAltitude();
        //first the simple flight pattern, getting up in the air
        if(!isAtDesiredAltitude()){
            simpleTakeoffControls(outputs);
            pitchControl(outputs);
        }else{
            //if we have once reached the desired altitude, go for steady state
            this.setThrust(outputs);
            //this.setHorizontalStabilizer(outputs);
            stabilizerActions(outputs);
            this.setMainWing(outputs);
        }
        //outputs.setHorStabInclination(-HORIZONTAL_STABILIZER_TAKEOFF);
        rollControl(outputs, inputs);
        angleOfAttackControl(outputs, this.getPreviousInputs(), this.getCurrentInputs());

        //System.out.println("outputs takeoffController: " + outputs);

        return outputs;
    }

    /**
     * Sets the controls for the first part of the takeoff where the drone needs to get actually of the ground
     * @param outputs the outputs of the controller for the drone
     */
    private void simpleTakeoffControls(ControlOutputs outputs) {
        float maxThrust = this.getConfig().getMaxThrust();
        float outputThrust = maxThrust;
        outputs.setThrust(outputThrust);
        outputs.setRightWingInclination(MAIN_TAKEOFF);
        outputs.setLeftWingInclination(MAIN_TAKEOFF);
        outputs.setHorStabInclination(-HORIZONTAL_STABILIZER_TAKEOFF);
    }

    private void pitchControl(ControlOutputs outputs){
        float pitch = Controller.extractOrientation(this.getCurrentInputs()).getyValue();
        if(abs(pitch) >= PITCH_THRESHOLD){
            outputs.setHorStabInclination(0f);
        }
    }

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
     * Checks if the desired altitude has been reached
     * @return true if and only if the desired altitude has been reached
     */
    private boolean isAtDesiredAltitude() {
        return hasReachedDesiredAltitude;
    }

    /**
     * Setter for the desired altitude flag
     */
    private void setReachedDesiredAltitude() {
        this.hasReachedDesiredAltitude = true;
    }


    /**
     * Getter for the reference altitude of the drone
     * @return the reference altitude as a float
     */
    public float getReferenceAltitude() {
        return referenceAltitude;
    }

    /**
     * Setter for the reference altitude, the altitude used for reference by the altitude PID
     * @param PIDReferenceAltitude the reference altitude
     */
    public void setReferenceAltitude(float PIDReferenceAltitude) {
        //don't forget to set the reference altitude
        this.referenceAltitude = PIDReferenceAltitude;
        this.desiredAltitude = PIDReferenceAltitude*0.95f; //set the desired altitude a little lower such that we start
        //with some pitch upwards
    }

    /**
     * Getter for the controller responsible for the pitch of the drone
     * @return the PID controller
     */
    private PIDController getPitchPID() {
        return pitchPID;
    }

    private Vector getTarget() {
        return target;
    }

    public void setTarget(Vector target) {
        this.target = target;
    }

    /**
     * The stable inclination of the main wings
     */
    private final static float MAIN_STABLE = (float) (5*PI/180);


    /**
     * The stable inclination of the stabilizer wings
     */
    private final static float STABILIZER_STABLE = 0;

    /**
     * The takeoff inclination of the main wings
     */
    private final static float MAIN_TAKEOFF = (float) (7*PI/180);
    /**
     * The takeoff inclination of the stabilizer
     */
    private final static float HORIZONTAL_STABILIZER_TAKEOFF = (float) (5*PI/180);

//    /**
//     * The maximum inclination of the main wings
//     */
//    private final static float MAIN_MAX = (float) (10*PI/180);

    /**
     * The maximum inclination of the stabilizer wings
     */
    private final static float HOR_STABILIZER_MAX = (float)(10*PI/180);

    /**
     * The threshold for the roll for roll control
     */
    private final static float ROLL_THRESHOLD = (float)(3*PI/180);


    private final static float PITCH_THRESHOLD = (float)(5*PI/180);
    /**
     * The safety margin taken for the AOA
     */
    private final static float INCLINATION_AOA_ERROR_MARGIN = (float)(3*PI/180);

    /**
     * Standard thrust: the reset thrust for control actions
     */
    private final static float STANDARD_THRUST = 550;

    /**
     * The desired altitude to reach with the drone before starting the altitude control
     */
    private float desiredAltitude;

    /**
     * Flag to check if the desired altitude has been reached
     */
    private boolean hasReachedDesiredAltitude = false;

    /**
     * Reference for the entry of the height (needed for the PID)
     */
    private float referenceAltitude;

    /**
     * A reference for the position to fly to
     */
    private Vector target;
//
//    /**
//     * A pid controller for the velocity of the drone
//     */
//    private VectorPID velocityPID = new VectorPID(1.0f, 0.1f, 0.1f);
//    /**
//     * A pid controller for the orientation of the drone
//     */
//    private VectorPID orientationPID = new VectorPID(1.0f, 0f, 0f);
//
//    private PIDController altitudePID = new PIDController(1.0f, 0.1f,0.2f);

    private final static float PITCH_GAIN = 1.0f;
    private final static float PITCH_DERIVATIVE = 0.2f;
    private final static float PITCH_INTEGRAL = 0.5f;

    private PIDController pitchPID = new PIDController(PITCH_GAIN,PITCH_INTEGRAL,PITCH_DERIVATIVE);

    /**
     * the distance to the first target before the controller passes control to the next controller
     * note: the drone also needs to see a cube to pass the control;
     */
    private final static float TARGET_DISTANCE_THRESHOLD = 80f;
    
}
