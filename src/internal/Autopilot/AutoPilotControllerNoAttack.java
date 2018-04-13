package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Testbed.FlightRecorder;
import internal.Exceptions.NoCubeException;
import internal.Helper.Vector;

import static java.lang.Math.sin;

/**
 * Created by Martijn on 30/10/2017.
 * Appended and edited by Anthony Rath� on 6/11/2017
 * A class of AutopilotInterfaces Controllers
 */
@Deprecated
public class AutoPilotControllerNoAttack {

    /**
     * Constructor for the autopilotController
     * @param autoPilot
     */
    public AutoPilotControllerNoAttack(AutoPilot autoPilot){
        this.associatedAutopilot = autoPilot;
    }

    private void setThrustOut(ControlOutputs outputs){
        //Todo implement: write the output to the outputs
        float pitch = this.getCurrentInputs().getPitch();
        float cubeSize =  this.getAssociatedAutopilot().getAPCamera().getTotalQualifiedPixels();
        int threshold = Math.round(THRESHOLD_DISTANCE);

        // Thrust
        float thrust = (float) ((STANDARD_THRUST) + THRUST_FACTOR*this.getTotalMass()*GRAVITY*sin(Math.PI - pitch));
        //System.out.println(thrust);
        outputs.setThrust(Math.max(Math.min(thrust, MAXTHRUST), 0));
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
        //outputs.setVerStabInclination(STANDARD_INCLINATION);
        if (yPosCube >= 0) {
            startTurnAscend(outputs, xPosCube, yPosCube);

        }else {
            startTurnDescend(outputs, xPosCube, yPosCube);

        }

        outputs.setRightWingInclination(TURNING_INCLINATION + MAIN_STABLE_INCLINATION);
        outputs.setLeftWingInclination(-TURNING_INCLINATION + MAIN_STABLE_INCLINATION);
    }

    private void startTurnLeft(ControlOutputs outputs, float xPosCube, float yPosCube){
        //outputs.setVerStabInclination(-STANDARD_INCLINATION);
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
        //stopAscendDescend(outputs, xPosCube, yPosCube);
        //System.out.println("Turn NOT descending nor ascending");
    }

    private void rollControl(ControlOutputs outputs){
        AutopilotInputs input = this.getCurrentInputs();
        float roll = input.getRoll();

        if(roll >= ROLL_THESHOLD){
            outputs.setRightWingInclination(-MAIN_STABLE_INCLINATION);
        }
        else if(roll <= - ROLL_THESHOLD){
            outputs.setLeftWingInclination(-MAIN_STABLE_INCLINATION);
        }else{
            // change nothing
        }
    }

    /**
     * Generates the appropriate control actions for the drone
     * @return the outputs for the autopilot
     */
    public AutopilotOutputs getControlActions(){

        ControlOutputs controlOutputs = new ControlOutputs();
        AutoPilotCamera APCamera = this.getAssociatedAutopilot().getAPCamera();
        AutopilotInputs currentInputs = this.getCurrentInputs();
        //APCamera.loadNextImage(currentInputs.getImage());
        APCamera.loadNewImage(currentInputs.getImage());

        Vector center = null;
        try {
            center = APCamera.getCenterOfNCubes(2).scalarMult(2f).vectorSum(APCamera.getCenterOfNCubes(5).scalarMult(1f)).scalarMult(3f);
        } catch (NoCubeException e) {
            center = new Vector();
        }
//        Vector center = APCamera.getCenterOfNCubes(1);
//        Vector center = APCamera.getCenterOfNCubes(1);
        float xPosition = center.getxValue();
        float yPosition = -center.getyValue();

        //System.out.println("Center location: " + center);

        int cubeSize = Math.round(center.getzValue());
        //System.out.println(cubeSize);

        //int threshold = Math.max(Math.round(THRESHOLD_PIXELS*NORMAL_CUBE_SIZE/cubeSize),1);
        int threshold = Math.round(THRESHOLD_DISTANCE);
        int bias = 0;
        if (currentInputs.getPitch() > Math.PI/20) {
            bias = BIAS;
        }else if(currentInputs.getPitch() < -Math.PI/20) {
            bias = -BIAS;
        }
        //System.out.println(bias);

        // Thrust
        this.setThrustOut(controlOutputs);


        // Roll
        if(xPosition > threshold){
            // Turn right
            //System.out.println("This is your captain speaking: the red cube is located at our right-hand-side");
            this.startTurnRight(controlOutputs, xPosition, yPosition);
        }else if(xPosition >= -threshold && xPosition <= threshold){
            // Stop turning
            this.stopTurn(controlOutputs, xPosition, yPosition);

            // Start Ascending/Descending
            // Ascend/Descend
            if(yPosition < -threshold - bias && (xPosition >= -threshold && xPosition <= threshold)){
                // Descend
                //System.out.println("This is your captain speaking: the red cube is located underneath us");
                this.startDescend(controlOutputs, xPosition, yPosition);

            }else if((yPosition >= -threshold - bias && yPosition <= threshold - bias) && (xPosition >= -threshold && xPosition <= threshold)){
                // Stop descending/ascending
                this.stopAscendDescend(controlOutputs, xPosition, yPosition);

            }else if(yPosition > threshold - bias && (xPosition >= -threshold && xPosition <= threshold)){
                // Ascend
                //System.out.println("This is your captain speaking: the red cube is located above us");
                this.startAscend(controlOutputs, xPosition, yPosition);

            }
        }else if(xPosition < -threshold){
            // Turn left
            //System.out.println("This is your captain speaking: the red cube is located at our left-hand-side");
            this.startTurnLeft(controlOutputs, xPosition, yPosition);
        }

        this.rollControl(controlOutputs);

        //System.out.println(controlOutputs);

        return controlOutputs;
    }
    /*
    Helper methods
     */
    private float getTotalMass(){
        AutoPilot autopilot = this.getAssociatedAutopilot();
        float mainWings = autopilot.getConfig().getWingMass()*2;
        float stabilizers = autopilot.getConfig().getTailMass()*2;
        float engine = autopilot.getConfig().getEngineMass();

        return mainWings + stabilizers + engine;
    }

    /*
    Getters and setters
     */

    public FlightRecorder getFlightRecorder() {
        return flightRecorder;
    }

    public void setFlightRecorder(FlightRecorder flightRecorder) {
        this.flightRecorder = flightRecorder;
    }

    /**
     * getter for the associated AutopilotInterfaces
     * @return the associated autopilot
     */
    public AutoPilot getAssociatedAutopilot() {
        return associatedAutopilot;
    }

    public AutopilotInputs getCurrentInputs() {
        return currentInputs;
    }

    /**
     * Setter for the current inputs of the drone, the old currentInputs are automatically
     * set previous inputs
     * @param currentInputs the current input for the autopilot
     */
    public void setCurrentInputs(AutopilotInputs currentInputs) {
        //first write to the previous outputs
        this.previousInputs = this.getCurrentInputs();
        //then write to the new ones.
        this.currentInputs = currentInputs;
    }

    public AutopilotInputs getPreviousInputs() {
        return previousInputs;
    }

    private AutoPilot associatedAutopilot;
    private AutopilotInputs currentInputs;
    private AutopilotInputs previousInputs;
    private FlightRecorder flightRecorder;

    private static final float STANDARD_INCLINATION = (float)Math.PI/8;
    public static final float MAIN_STABLE_INCLINATION = (float)Math.PI/12;
    private static final float MAX_HOR_STAB_INCLINATION = (float)Math.PI/4;
    private static final float TURNING_INCLINATION = (float)Math.PI/8;
    private static final int BIAS = 0;
    private static final float THRESHOLD_DISTANCE = 5f;
    private static final float STANDARD_THRUST = 32.859283f*2;
    private static final float THRUST_FACTOR = 2.0f;
    private static final float THRESHOLD_THRUST_ANGLE = (float)(Math.PI/20);
    private static final float STANDARD_CUBE_SIZE = 10f;
    public static final float STABILIZER_STABLE_INCLINATION = 0.0f;
    private static final float GRAVITY = 9.81f;
    private static final float ROLL_THESHOLD = (float) (Math.PI * 3.0f/180.0f);
    private static final float MAXTHRUST = 250.0f;

    private class ControlOutputs implements AutopilotOutputs{

        public ControlOutputs(){
            //do nothing, everything stays initialized on zero
        }

        @Override
        public float getThrust() {
            return this.thrust;
        }

        @Override
        public float getLeftWingInclination() {
            return this.leftWingInclination;
        }

        @Override
        public float getRightWingInclination() {
            return this.rightWingInclination;
        }

        @Override
        public float getHorStabInclination() {
            return this.horStabInclination;
        }

        @Override
        public float getVerStabInclination() {
            return this.verStabInclination;
        }

        @Override
        public float getFrontBrakeForce() {
            return 0;
        }

        @Override
        public float getLeftBrakeForce() {
            return 0;
        }

        @Override
        public float getRightBrakeForce() {
            return 0;
        }

        /**
         * Setter for the Thrust
         * @param thrust the desired thrust
         */
        public void setThrust(float thrust) {
            this.thrust = thrust;
        }

        /**
         * Setter for the left wing inclination
         * @param leftWingInclination
         */
        public void setLeftWingInclination(float leftWingInclination) {
            this.leftWingInclination = leftWingInclination;
        }

        /**
         * Setter for the right wing inclination
         * @param rightWingInclination the desired right wing inclination
         */
        public void setRightWingInclination(float rightWingInclination) {
            this.rightWingInclination = rightWingInclination;
        }

        /**
         * Setter for the horizontal stabilizer inclination
         * @param horStabInclination the desired horizontal stabilizer inclination
         */
        public void setHorStabInclination(float horStabInclination) {
            this.horStabInclination = horStabInclination;
        }

        /**
         * Setter for the vertical stabilizer inclination
         * @param verStabInclination the desired vertical stabilizer inclination
         */
        public void setVerStabInclination(float verStabInclination) {
            this.verStabInclination = verStabInclination;
        }

        //initialize the writes to the stable state of the drone
        private float thrust = STANDARD_THRUST;
        private float leftWingInclination = MAIN_STABLE_INCLINATION;
        private float rightWingInclination = MAIN_STABLE_INCLINATION;
        private float horStabInclination = STABILIZER_STABLE_INCLINATION;
        private float verStabInclination = STABILIZER_STABLE_INCLINATION;

        @Override
        public String toString() {
            return "ControlOutputs{" +
                    "thrust=" + thrust +
                    ", leftWingInclination=" + leftWingInclination +
                    ", rightWingInclination=" + rightWingInclination +
                    ", horStabInclination=" + horStabInclination +
                    ", verStabInclination=" + verStabInclination +
                    '}';
        }
    }
}
