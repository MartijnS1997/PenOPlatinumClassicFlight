package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Exceptions.NoCubeException;
import internal.Helper.Vector;


import static java.lang.Math.PI;
import static java.lang.Math.abs;

import java.util.List;

/**
 * Created by Martijn on 7/12/2017.
 * controller for AOA different from 30
 */
public class AlphaFlightController extends AutoPilotFlightController {

    public AlphaFlightController(AutoPilot autoPilot){
        super(autoPilot);
        System.out.println("Using alpha controller");
    }


    private void setThrustOut(ControlOutputs outputs, float xPosition, float yPosition){
        //Todo implement: write the output to the outputs
        float pitch = this.getCurrentInputs().getPitch();
        float cubeSize =  this.getAutopilot().getAPCamera().getTotalQualifiedPixels();
        int threshold = Math.round(THRESHOLD_DISTANCE);
        float standardThrust = this.getAutopilot().getConfig().getMaxThrust()/4;
        float maxThrust = this.getAutopilot().getConfig().getMaxThrust();

        // Thrust
        float sigmoidFactor = 4f;
//        float thrust = (float) ((STANDARD_THRUST)*(1-sigmoid(abs(xPosition/sigmoidFactor))) + THRUST_FACTOR*this.getTotalMass()*GRAVITY*sigmoid(yPosition/sigmoidFactor));
        float thrust = (float) ((standardThrust)/(5*(abs(xPosition)+0.01)) + THRUST_FACTOR*this.getTotalMass()*GRAVITY*sigmoid(yPosition/sigmoidFactor));
        //System.out.println(xPosition);
        outputs.setThrust(Math.max(Math.min(thrust, maxThrust), 0));
    }

    private float sigmoid(float x) {
        return (float) (1/(1+Math.exp(-x)));
    }

    private void startDescend(ControlOutputs outputs, float xPosCube, float yPosCube){
        outputs.setHorStabInclination(Math.min(STANDARD_INCLINATION*Math.abs(yPosCube)/20f,MAX_HOR_STAB_INCLINATION));
    }

    private void startAscend(ControlOutputs outputs,  float xPosCube, float yPosCube){
        outputs.setHorStabInclination(-Math.min(STANDARD_INCLINATION*Math.abs(yPosCube)/20f,MAX_HOR_STAB_INCLINATION));
    }

    private void startTurnDescend(ControlOutputs outputs, float xPosCube, float yPosCube){
        outputs.setHorStabInclination(Math.min(STANDARD_INCLINATION*Math.abs(yPosCube)/30f,MAX_HOR_STAB_INCLINATION));
    }

    private void startTurnAscend(ControlOutputs outputs,  float xPosCube, float yPosCube){
        outputs.setHorStabInclination(-Math.min(STANDARD_INCLINATION*Math.abs(yPosCube)/30f,MAX_HOR_STAB_INCLINATION));

    }

    private void stopAscendDescend(ControlOutputs outputs,  float xPosCube, float yPosCube){
        outputs.setHorStabInclination(STABILIZER_STABLE_INCLINATION);
    }

    private void startTurnRight(ControlOutputs outputs,  float xPosCube, float yPosCube){
        if (yPosCube >= 0) {
            startTurnAscend(outputs, xPosCube, yPosCube);

        }else {
            startTurnDescend(outputs, xPosCube, yPosCube);

        }

        outputs.setRightWingInclination(TURNING_INCLINATION + MAIN_STABLE_INCLINATION);
        outputs.setLeftWingInclination(-TURNING_INCLINATION + MAIN_STABLE_INCLINATION);

    }

    private void startTurnLeft(ControlOutputs outputs, float xPosCube, float yPosCube){
        ///outputs.setVerStabInclination(-STANDARD_INCLINATION);
        if (yPosCube >= 0) {
            startTurnAscend(outputs, xPosCube, yPosCube);

        }else {
            startTurnDescend(outputs, xPosCube, yPosCube);

        }

        outputs.setRightWingInclination(-TURNING_INCLINATION + MAIN_STABLE_INCLINATION);
        outputs.setLeftWingInclination(TURNING_INCLINATION + MAIN_STABLE_INCLINATION);

    }

    private void stopTurn(ControlOutputs outputs, float xPosCube, float yPosCube){
        outputs.setVerStabInclination(STABILIZER_STABLE_INCLINATION);
        outputs.setRightWingInclination(MAIN_STABLE_INCLINATION);
        outputs.setLeftWingInclination(MAIN_STABLE_INCLINATION);
    }

    /**
     * Generates the appropriate control actions for the drone
     * @return the outputs for the autopilot
     * @param inputs
     */
    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs){

        //System.out.println("-------");
        this.setCurrentInputs(inputs);

        ControlOutputs controlOutputs = new ControlOutputs();
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();

        //APCamera.loadNextImage(currentInputs.getImage());
        AutoPilotCamera APCamera = this.getAutopilot().getAPCamera();
        APCamera.loadNewImage(currentInputs.getImage());

        float elapsedTime = this.getCurrentInputs().getElapsedTime();

        Vector center = null;
        try {
            center = APCamera.getCenterOfNCubes(1);
//            center = APCamera.getCenterOfNCubes(1).scalarMult(5f).vectorSum(APCamera.getCenterOfNCubes(5).scalarMult(-1f)).scalarMult(1f);
        } catch (NoCubeException e) {
            center = new Vector();
        }
//        Vector center = APCamera.getCenterOfNCubes(1);
        float xPosition = this.getxPID().getPIDOutput(-center.getxValue(), elapsedTime);
        float yPosition = this.getyPID().getPIDOutput(center.getyValue(), elapsedTime);
        
        int cubeSize = Math.round(center.getzValue());

        //int threshold = Math.max(Math.round(THRESHOLD_PIXELS*NORMAL_CUBE_SIZE/cubeSize),1);
        int threshold = Math.round(THRESHOLD_DISTANCE);
        int bias = 0;
        if (currentInputs.getPitch() > PI / 20) {
            bias = BIAS;
        } else if (currentInputs.getPitch() < -PI / 20) {
            bias = -BIAS;
        }
        //System.out.println(bias);

        // Thrust
        this.setThrustOut(controlOutputs, xPosition, yPosition);

        //String controlString = "Control action ";
        
        List<Vector> cubesPictuur = APCamera.getCubesInPicture();
        if (cubesPictuur.isEmpty()){
        	this.stopTurn(controlOutputs, xPosition, yPosition);
        	this.stopAscendDescend(controlOutputs, xPosition, yPosition);
        	this.startTurnRight(controlOutputs, 100, 0);
        }

        // Roll
        if (xPosition > threshold) {
            // Turn right
            //System.out.println("This is your captain speaking: the red cube is located at our right-hand-side");
            this.startTurnRight(controlOutputs, xPosition, yPosition);
            //controlString += "Turning Right: \n";
        } else if (xPosition >= -threshold && xPosition <= threshold) {
            // Stop turning
            this.stopTurn(controlOutputs, xPosition, yPosition);

            // Start Ascending/Descending
            // Ascend/Descend
            if (yPosition < -threshold - bias && (xPosition >= -threshold && xPosition <= threshold)) {
                // Descend
                //System.out.println("This is your captain speaking: the red cube is located underneath us");
                this.startDescend(controlOutputs, xPosition, yPosition);
                //controlString += "Start Descend: \n";

            } else if ((yPosition >= -threshold - bias && yPosition <= threshold - bias) && (xPosition >= -threshold && xPosition <= threshold)) {
                // Stop descending/ascending
                this.stopAscendDescend(controlOutputs, xPosition, yPosition);
                //controlString += "Stop Ascending: \n";

            } else if (yPosition > threshold - bias && (xPosition >= -threshold && xPosition <= threshold)) {
                // Ascend
                //System.out.println("This is your captain speaking: the red cube is located above us");
                this.startAscend(controlOutputs, xPosition, yPosition);
                //controlString += "Start Ascending: \n";

            }
        } else if (xPosition < -threshold) {
            // Turn left
            //System.out.println("This is your captain speaking: the red cube is located at our left-hand-side");
            this.startTurnLeft(controlOutputs, xPosition, yPosition);
            //controlString += "Start Turn left: \n";
        }



        this.rollControl(controlOutputs, this.getCurrentInputs());

        this.angleOfAttackControl(controlOutputs, this.getPreviousInputs(), this.getCurrentInputs());
        //System.out.println("Controls delivered");
        //System.out.println(controlOutputs);
        
        // If all blocks were hit, start landingprocedure
        APCamera.loadNewImage(currentInputs.getImage());
        int amountOfCubesInSight = APCamera.getCubesInPicture().size();
        
//        if (amountOfCubesInSight <= 0) {
//        	this.getAutopilot().setAPMode(3);
//        }


        return controlOutputs;
    }

    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 inputs) {
        return false;
    }

    /*
    Getters and setters
     */

    @Override
    protected float getMainStableInclination() {
        return MAIN_STABLE_INCLINATION;
    }
    @Override
    protected float getStabilizerStableInclination() {
        return STABILIZER_STABLE_INCLINATION;
    }
    @Override
    protected float getRollThreshold() {
        return ROLL_THESHOLD;
    }
    @Override
    protected float getInclinationAOAErrorMargin() {
        return ERROR_INCLINATION_MARGIN;
    }

    public PIDController getxPID() {
        return xPID;
    }

    public PIDController getyPID() {
        return yPID;
    }

    private PIDController xPID = new PIDController(1.f, 0.0f, 0.0f);
    private PIDController yPID = new PIDController(1.f, 0.0f, 0.0f);

    private static final float STANDARD_INCLINATION = (float) PI/12;
    public  static final float MAIN_STABLE_INCLINATION = (float) PI/12;
    private static final float MAX_HOR_STAB_INCLINATION = (float) PI/8;
    private static final float TURNING_INCLINATION = (float) PI/18;
    private static final float ERROR_INCLINATION_MARGIN = (float) (5*PI/180);
    private static final int   BIAS = 0;
    private static final float THRESHOLD_DISTANCE = 0f;
    private static final float THRUST_FACTOR = 1.0f;
    public static final float  STABILIZER_STABLE_INCLINATION = 0.0f;
    private static final float GRAVITY = 9.81f;
    private static final float ROLL_THESHOLD = (float) (PI * 3.0f/180.0f);

}
