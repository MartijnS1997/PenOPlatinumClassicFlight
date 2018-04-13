package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;
import jdk.nashorn.internal.objects.NativeReferenceError;
import org.lwjgl.system.CallbackI;

import javax.naming.ldap.Control;

import static java.lang.Math.*;

/**
 * Created by Martijn on 21/03/2018.
 */
public class StabilizerController extends Controller {

    /**
     * Default constructor, generated to make java happy
     * @param autopilot the autopilot the controller is associated with
     */
    public StabilizerController(AutoPilot autopilot) {
        super(autopilot);
    }

    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs) {
        this.setCurrentInputs(inputs);
        //generate the outputs object to write the control actions to
        ControlOutputs outputs = new ControlOutputs();
        this.getPitchControlActions(outputs);
        this.getRollControlActions(outputs);
        //now check for AOA issues
        this.angleOfAttackControl(outputs, getPreviousInputs(), getCurrentInputs());
        //return the outputs
        return outputs;
    }

    /**
     * Resets the stabilizer controller's PID controllers so
     * the same object can be re-used after some time
     */
    public void reset(){
        this.getPitchController().reset();
        this.getRollController().reset();
    }

    /**
     * Getter for the control actions based on the current pitch of the drone
     * @param outputs the outputs to write the result of the calculations to
     */
    private void getPitchControlActions(ControlOutputs outputs){
        //get the two inputs
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
        //calculate the passed time between our prev simulation step
        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        //get the pitch
        float pitch = currentInputs.getPitch();
        //get the PID
        PIDController pitchPid = this.getPitchController();
        //get the outputs (we want pitch zero so out current pitch is the error)
        float pidOutputs = pitchPid.getPIDOutput(pitch, deltaTime);
        //based on the output set the horizontal rudder, cap it
        float horizontalInclination = this.getStabilizerStableInclination() - pidOutputs;
        horizontalInclination = signum(horizontalInclination) * min(abs(horizontalInclination), MAX_HOR_STAB_INCL);
        outputs.setHorStabInclination(horizontalInclination);

    }

    /**
     * Calculates the control actions to be taken based on the current roll
     * @param outputs the outputs to write the result of the calculation to
     */
    private void getRollControlActions(ControlOutputs outputs){
        //get the current and the previous inputs
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
        //get the time passed between two iterations
        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        //get the current roll
        float roll = currentInputs.getRoll();
        //then get the pid
        PIDController rollPid = this.getRollController();
        //get the output, reference roll is 0, so our current roll is the error
        float pidOutputs = rollPid.getPIDOutput(roll, deltaTime);
        //now set the main wings based on the outputs of the PID controller
        float rightMainWing = this.getMainStableInclination() + pidOutputs;
        float leftMainWing = this.getMainStableInclination() - pidOutputs;

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
        float lowerCap = this.getMainStableInclination() - MAIN_INCL_CAP_DELTA;
        float upperCap = this.getMainStableInclination() + MAIN_INCL_CAP_DELTA;
        if(inclination < lowerCap){
            return lowerCap;
        }
        if(inclination > upperCap){
            return upperCap;
        }
        return inclination;
    }


    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 inputs) {
        float currentTime = inputs.getElapsedTime();
        float startTime = this.getControlStartElapsedTime();

        float controllerTime =  currentTime - startTime;
        //we have reached the stabilizing objective if we are flying stable and have stabilized for long enough
        return controllerTime > MIN_STABILIZATION_TIME && isStabilized(inputs);
    }

    /**
     * checks if the drone is stabilized
     * @param inputs the current inputs from the testbed
     * @return true if the roll and pitch are within the acceptable margins
     */
    private boolean isStabilized(AutopilotInputs_v2 inputs){
        Vector orientation = Controller.extractOrientation(inputs);
        float pitch = orientation.getyValue();
        float roll = orientation.getzValue();

        return (roll <= ROLL_STABILITY_TARGET) && (pitch <= PITCH_STABILITY_TARGET);
    }

    @Override
    protected float getMainStableInclination() {
        return MAIN_STABLE_INCLINATION;
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
        return ERROR_MARGIN_AOA_CALC;
    }

    @Override
    protected float getStandardThrust() {
        return STANDARD_THRUST;
    }

    /*
     * Getters and setters
     */

    /**
     * Setter for the start of the control elapsed time (needs to be set by the controller selector)
     * is used to check if we have stabilized for long enough to call it a stable flight
     * @return the time at which the controller was activated for this particular flight phase
     */
    public float getControlStartElapsedTime() {
        return controlStartElapsedTime;
    }

    /**
     * Setter for the control start elapsed time (see getter for more info)
     * @param controlStartElapsedTime the time to be set
     */
    public void setControlStartElapsedTime(float controlStartElapsedTime) {
        this.controlStartElapsedTime = controlStartElapsedTime;
    }

    /**
     * Getter for the PID controller used to control the pitch
     * @return a PID controller
     */
    private PIDController getPitchController() {
        return pitchController;
    }

    /**
     * Getter for the PID controller used to control the roll
     * @return a PID controller
     */
    private PIDController getRollController() {
        return rollController;
    }

    /*
     * Instances used
     */

    /**
     * The time at which the controller is activated for the first time between two flight phases
     * this value is used to infer the stabilization time from
     */
    private float controlStartElapsedTime = 0f;


    /**
     * The PID controllers used by the controller to stabilize the flight
     */

    private final static float PITCH_GAIN = 1.0f;
    private final static float PITCH_INTEGRAL = 0.1f;
    private final static float PITCH_DERIVATIVE = 0.1f;
    private PIDController pitchController = new PIDController(PITCH_GAIN, PITCH_INTEGRAL, PITCH_DERIVATIVE);

    private final static float ROLL_GAIN = 1.0f;
    private final static float ROLL_INTEGRAL= 0.1f;
    private final static float ROLL_DERIVATIVE = 0.1f;
    private PIDController rollController = new PIDController(ROLL_GAIN, ROLL_INTEGRAL, ROLL_DERIVATIVE);

    /**
     * Constants used for configuring the controller
     */
    private final static float MAIN_STABLE_INCLINATION = (float) (5*PI/180);
    private final static float MAIN_INCL_CAP_DELTA = (float)(2*PI/180);
    private final static float STABILIZER_STABLE = 0f;
    private final static float MAX_HOR_STAB_INCL = (float) (10*PI/180);
    private final static float ROLL_THRESHOLD = (float) (1*PI/180);
    private final static float ERROR_MARGIN_AOA_CALC = (float) (3*PI/180);
    private final static float STANDARD_THRUST = 550f;

    /**
     * Constants used to evaluate the stability of the drone
     */
    private final static float PITCH_STABILITY_TARGET = (float) (1*PI/180);
    private final static float ROLL_STABILITY_TARGET = (float)(1*PI/180);
    private final static float MIN_STABILIZATION_TIME = 1.0f;
    //note the minimal stability time is used for the case when the drone is flying momentarily at pitch
    //and roll at the right angle, but the rotation is way off. (and thus unstable)

}
