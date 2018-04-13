package internal.Autopilot;

import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotInputs;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Physics.PhysXEngine;
import internal.Helper.Vector;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

/**
 * Created by Martijn on 18/02/2018.
 * A class of controllers
 */
//Todo fix the init issue, create dummy controller with same position but at different elapased time (velocity zero)
public abstract class Controller {

    public Controller(AutoPilot autopilot){
        this.autopilot = autopilot;
        this.setPreviousInputs(dummyData);
        this.currentInputs = dummyData;
        this.pathGenerator = new PathGenerator();
    }

    /**
     * Generates the control actions for the autopilot
     * @param inputs the inputs of the autopilot
     * @return the control actions
     */
    public abstract AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs);

    /**
     * Checks if the objective of the current controller has been reached
     * @param inputs the current inputs (this is the base of the check)
     * @return true if the controller is ready with its task
     */
    public abstract boolean hasReachedObjective(AutopilotInputs_v2 inputs);

    /*
    Getters and setters
     */

    /**
     * Getter for the main wing stable inclination
     * @return the main wing stable inclination
     */
    protected abstract float getMainStableInclination();

    /**
     * Getter for the stabilizer wings stable inclination
     * @return the stabilizer stable inclination
     */
    protected abstract float getStabilizerStableInclination();

    /**
     * Getter for the threshold for when roll control is activated
     * @return the roll threshold
     */
    protected abstract float getRollThreshold();

    /**
     * Getter for the extra error margin builtin to account for AOA calculation errors
     * @return the error margin for the AOA
     */
    protected abstract float getInclinationAOAErrorMargin();

    /**
     * Getter for the standard thrust for controlling the drone
     * @return the standard thrust
     */
    protected abstract float getStandardThrust();



    /*
     * Supplementary control methods
     */
    protected void rollControl(ControlOutputs outputs, AutopilotInputs_v2 currentInput){
        float roll = currentInput.getRoll();

        if(roll >= this.getRollThreshold()){
            outputs.setRightWingInclination(-this.getMainStableInclination());
        }
        else if(roll <= - this.getRollThreshold()){
            outputs.setLeftWingInclination(-this.getMainStableInclination());
        }else{
            // change nothing
        }
    }

    /**
     * Checks if the current control outputs are realisable under the angle of attack constraint provided
     * by the autopilot configuration. If not the controls are adjusted to fit the constraints
     * @param controlOutputs the control outputs to be checked
     * @param prevInputs
     * @param currentInputs
     */
    protected void angleOfAttackControl(ControlOutputs controlOutputs, AutopilotInputs_v2 prevInputs, AutopilotInputs_v2 currentInputs){

        //first check if the current and the previous steps are initialized, if not so delete all control actions
        //and set to standard value
        if(currentInputs == null || prevInputs == null){
            controlOutputs.reset();
            return;
        }
        //first prepare all the variables
        PhysXEngine.PhysXOptimisations optimisations = this.getAutopilot().getPhysXOptimisations();
        Vector orientation = extractOrientation(currentInputs);
        Vector velocity = this.getVelocityApprox(prevInputs, currentInputs);
        //System.out.println("Velocity: " + velocity);
        Vector rotation = this.getRotationApprox(prevInputs, currentInputs);
        float angleOfAttack = this.getConfig().getMaxAOA();

        //change until the controls fit
        AOAControlMainLeft(controlOutputs, optimisations,angleOfAttack, orientation, rotation, velocity);
        AOAControlMainRight(controlOutputs, optimisations, angleOfAttack, orientation, rotation, velocity);
        AOAControlHorStabilizer(controlOutputs, optimisations, angleOfAttack, orientation, rotation, velocity);
        AOAControlVerStabilizer(controlOutputs, optimisations, angleOfAttack, orientation, rotation, velocity);
    }

    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlMainLeft(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack,  Vector orientation, Vector rotation, Vector velocity){
        //System.out.println("Left Main");
        float inclinationBorder1 = optimisations.getMaxLeftMainWingInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxLeftMainWingInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getLeftWingInclination();

        float realisableInclination = setBetween(desiredInclination, inclinationBorder1, inclinationBorder2, this.getInclinationAOAErrorMargin());

        controlOutputs.setLeftWingInclination(realisableInclination);

        return desiredInclination == realisableInclination;
    }

    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlMainRight(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack, Vector orientation, Vector rotation, Vector velocity){
        //System.out.println("MainRight");
        float inclinationBorder1 = optimisations.getMaxRightMainWingInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxRightMainWingInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getRightWingInclination();

        float realisableInclination = setBetween(desiredInclination, inclinationBorder1, inclinationBorder2, this.getInclinationAOAErrorMargin());

        controlOutputs.setRightWingInclination(realisableInclination);

        return desiredInclination == realisableInclination;
    }

    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlHorStabilizer(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack, Vector orientation, Vector rotation, Vector velocity){
        //System.out.println("Horizontal stabilizer");
        float inclinationBorder1 = optimisations.getMaxHorStabInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxHorStabInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getHorStabInclination();

        float realisableInclination = setBetween(desiredInclination, inclinationBorder1, inclinationBorder2, this.getInclinationAOAErrorMargin());

        controlOutputs.setHorStabInclination(realisableInclination);

        return desiredInclination == realisableInclination;
    }

    /**
     * Checks if the control outputs are realisable under the AOA restrictions, if not change them to fit
     * between the borders of what is allowed.
     * @param controlOutputs the control outputs of the controller
     * @param optimisations the physics optimisations used for the calculations
     * @param angleOfAttack the maximum angle of attack
     * @param orientation the orientation of the drone
     * @param rotation the rotation of the drone (world-axis)
     * @param velocity the velocity of the drone (world-axis)
     * @return true if the controls were changed, false if not
     * @author Martijn Sauwens
     */
    private boolean AOAControlVerStabilizer(ControlOutputs controlOutputs, PhysXEngine.PhysXOptimisations optimisations, float angleOfAttack, Vector orientation, Vector rotation, Vector velocity){
        //System.out.println("Vertical stabilizer");
        float inclinationBorder1 = optimisations.getMaxVerStabInclination(orientation, rotation, velocity, angleOfAttack);
        float inclinationBorder2 = optimisations.getMaxVerStabInclination(orientation, rotation, velocity, -angleOfAttack);

        float desiredInclination = controlOutputs.getVerStabInclination();

        float realisableInclination = setBetween(desiredInclination, -inclinationBorder1, -inclinationBorder2, this.getInclinationAOAErrorMargin());

        controlOutputs.setVerStabInclination(realisableInclination);

        //System.out.println("\n");
        return desiredInclination == realisableInclination;
    }


    /*
    Helper methods
     */
    //TODO account for the fact that the distance between the borders could be smaller than the error margin
    private float setBetween(float value, float border1, float border2, float errorMargin){
        //first check if the value isn't already between the borders:
        float[] borders = sortValue(border1, border2);
        float lowerBorder = borders[0];
        float upperBorder = borders[1];
        //System.out.println("Lower border: " + lowerBorder*RAD2DEGREE + "; Upper border: " + upperBorder*RAD2DEGREE + "; value: " + value*RAD2DEGREE);
        //check if it is already between the borders
        if(value >= lowerBorder && value <= upperBorder) {
            //System.out.println("Selected value: " + value*RAD2DEGREE);
            return value;
        }


        //if not so, set it between with a given error margin
        //check if the value is closest to the lower border
        if(abs(lowerBorder - value) <= abs(upperBorder - value)){
            //System.out.println("Selected value: " + (lowerBorder - signum(lowerBorder)*errorMargin)*RAD2DEGREE);
            return lowerBorder - signum(lowerBorder)*errorMargin;
        }else{
            //System.out.println("Selected value: " + (upperBorder - signum(upperBorder)*errorMargin)*RAD2DEGREE);
            return upperBorder - signum(upperBorder)*errorMargin;
        }

    }

    /**
     * Sorts the two values
     * @param value1 the first value to be sorted
     * @param value2 the second value to be sorted
     * @return an array of size 2 with the smallest value first and the largest second.
     */
    private float[] sortValue(float value1, float value2){

        float[] sortedArray = new float[2];
        if(value1 <= value2) {
            sortedArray[0] = value1;
            sortedArray[1] = value2;
        }else{
            sortedArray[0] = value2;
            sortedArray[1] = value1;
        }

        return sortedArray;
    }

    protected float getTotalMass(){
        AutopilotConfig config = this.getConfig();
        float mainWings = config.getWingMass()*2;
        float stabilizers = config.getTailMass()*2;
        float engine = config.getEngineMass();

        return mainWings + stabilizers + engine;
    }

    /**
     * Extracts the delta time between two steps (simulation time)
     * @param previousInputs the previous inputs of the autopilot
     * @param currentInputs the current inputs of the autopilot
     * @return the time elapsed between the last two simulation steps
     */
    protected static float getDeltaTime(AutopilotInputs_v2 previousInputs, AutopilotInputs_v2 currentInputs){
        float prevTime = previousInputs.getElapsedTime();
        float currTime = currentInputs.getElapsedTime();

        return currTime - prevTime;
    }

    /**
     * Calculate an approximation of the velocity
     * @param prevInputs the autopilot inputs at moment k-1 needed for the derivative
     * @param currentInputs the autopilot inputs at moment k needed for the derivative
     * @return the approximation of the velocity
     * elaboration: see textbook numerical math for derivative methods, the
     * derivative of f(k+1) - f(k-1) / (2*timeStep) has O(h²) correctness
     */
    public static Vector getVelocityApprox(AutopilotInputs_v2 prevInputs, AutopilotInputs_v2 currentInputs){
        float prevTime = prevInputs.getElapsedTime();
        float currentTime = currentInputs.getElapsedTime();

        Vector prevPos = extractPosition(prevInputs);
        Vector currentPos = extractPosition(currentInputs);

        Vector posDiff = currentPos.vectorDifference(prevPos);
        float timeDiff = currentTime - prevTime;

        return posDiff.scalarMult(1/timeDiff);
    }

    /**
     * Approximates the current rotation of the drone based on the old rotations
     * @param prevInputs  the previous autopilot inputs (moment k-1) needed for the derivative
     * @param currentInputs  the current autopilot inputs (moment k) needed for the derivative
     * @return an approx for the rotation (first calculate the rotation in heading pitch and roll components
     *         and transform them to the actual rotational components)
     */
    public static Vector getRotationApprox(AutopilotInputs_v2 prevInputs, AutopilotInputs_v2 currentInputs){

        //get the passed time interval
        float prevTime = prevInputs.getElapsedTime();
        float currentTime = currentInputs.getElapsedTime();

        //extract the orientations from the inputs
        Vector prevOrient = extractOrientation(prevInputs);
        Vector currentOrient = extractOrientation(currentInputs);

        Vector orientDiff = currentOrient.vectorDifference(prevOrient);
        float timeDiff = currentTime - prevTime;

        // the given rotation vector is given in heading pitch and roll components
        Vector rotationHPR = orientDiff.scalarMult(1/timeDiff);
        // convert back to the world axis rotation vector
        return PhysXEngine.HPRtoRotation(rotationHPR, currentOrient);
    }

    /**
     * Extractor of the orientation in vector format
     * @param inputs the autopilotInput object containing the current inputs
     * @return a vector containing the orientation of the drone in vector format
     */
    protected static Vector extractOrientation(AutopilotInputs_v2 inputs){
        return new Vector(inputs.getHeading(), inputs.getPitch(), inputs.getRoll());
    }

    /**
     * Extractor of the orientation in vector format
     * @param inputs the autopilotInput object containing the current inputs
     * @return a vector containing the position of the drone in vector format
     */
    protected static Vector extractPosition(AutopilotInputs_v2 inputs){
        return new Vector(inputs.getX(), inputs.getY(), inputs.getZ());
    }

    protected static List<Vector> extractPath(AutopilotInterfaces.Path path){

        float xPath[] = path.getX();
        float yPath[] = path.getY();
        float zPath[] = path.getZ();

        // map the path to a list
        List<Vector> pathList = IntStream.range(0, xPath.length)
                .mapToObj(i -> new Vector(xPath[i], yPath[i], zPath[i]))
                .collect(Collectors.toList());

        return pathList;
    }


    /**
     * Getter for the autopilot of the drone
     * @return the autopilot
     */
    protected AutoPilot getAutopilot() {
        return autopilot;
    }


    /**
     * Getter for the configuration of the drone
     * @return the configuration
     */
    protected AutopilotConfig getConfig() {
        return config;
    }

    /**
     * Setter for the configuration
     */
    protected void setConfig(AutopilotConfig config){
        this.config = config;
    }
    
    /**
     * Getter for the current inputs (the autopilot inputs object)
     * @return the current outputs
     */
    public AutopilotInputs_v2 getCurrentInputs() {
        return currentInputs;
    }
    
    /**
     * Setter for the current inputs of the drone, the old currentInputs are automatically
     * set previous inputs
     * @param currentInputs the current input for the autopilot
     */
    protected void setCurrentInputs(AutopilotInputs_v2 currentInputs) {
        //first write to the previous outputs
        this.setPreviousInputs(this.getCurrentInputs());

        //then write to the new ones.
        this.currentInputs = currentInputs;
    }

    /**
     * Returns the previous AutopilotInterfaces Inputs
     */
    public AutopilotInputs_v2 getPreviousInputs() {
        return previousInputs;
    }

    /**
     * Setter for the autopilot inputs
     * @param previousInputs the pervious inputs
     */
    protected void setPreviousInputs(AutopilotInputs_v2 previousInputs){
        this.previousInputs = previousInputs;
    }

    private AutopilotInputs_v2 currentInputs;
    private AutopilotInputs_v2 previousInputs;

    /**
     * Object that stores the autopilot of the drone
     */
    private AutoPilot autopilot;
    
    /**
     * Object that stores the pathgenerator of the controller
     */
    private final PathGenerator pathGenerator;

    /**
     * Object that stores the configuration of the drone
     */
    private AutopilotConfig config;


    private static float RAD2DEGREE = (float) (180/PI);

    /**
     * An implementation of AutopilotOutputs used in the controller for cascading control (passes trough the basic
     * controller, roll control and AOA controll)
     */
    class ControlOutputs implements AutopilotOutputs{

        ControlOutputs(){
            //do nothing, everything stays initialized on zero
        }

        /**
         * gets a deep copy of the given output instance
         * @return the copy
         */
        public ControlOutputs copy(){
            ControlOutputs copy = new ControlOutputs();
            copy.setRightWingInclination(this.getRightWingInclination());
            copy.setLeftWingInclination(this.getLeftWingInclination());
            copy.setHorStabInclination(this.getHorStabInclination());
            copy.setVerStabInclination(this.getVerStabInclination());
            copy.setThrust(this.getThrust());
            copy.setFrontBrakeForce(this.getFrontBrakeForce());
            copy.setLeftBrakeForce(this.getLeftBrakeForce());
            copy.setRightBrakeForce(this.getRightBrakeForce());

            return copy;
        }

        /**
         * Set to default values
         * used to reset the outputs if the controller is not fully initialized
         */
        protected void reset(){

            this.setRightWingInclination(getMainStableInclination());
            this.setLeftWingInclination(getMainStableInclination());
            this.setHorStabInclination(getStabilizerStableInclination());
            this.setVerStabInclination(getStabilizerStableInclination());
            this.setFrontBrakeForce(this.getFrontBrakeForce());
            this.setLeftBrakeForce(this.getLeftBrakeForce());
            this.setRightBrakeForce(this.getRightBrakeForce());
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
            return this.frontBrakeForce;
        }

        @Override
        public float getLeftBrakeForce() {
            return this.leftBrakeForce;
        }

        @Override
        public float getRightBrakeForce() {
            return this.rightBrakeForce;
        }





        public void setLeftBrakeForce(float leftBrakeForce) {
            this.leftBrakeForce = leftBrakeForce;
        }

        public void setRightBrakeForce(float rightBrakeForce) {
            this.rightBrakeForce = rightBrakeForce;
        }


        public void setFrontBrakeForce(float frontBrakeForce) {
            this.frontBrakeForce = frontBrakeForce;
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
        private float thrust = getStandardThrust();
        private float leftWingInclination = getMainStableInclination();
        private float rightWingInclination = getMainStableInclination();
        private float horStabInclination = getStabilizerStableInclination();
        private float verStabInclination = getStabilizerStableInclination();

        private float leftBrakeForce = 0;
        private float rightBrakeForce = 0;
        private float frontBrakeForce = 0;

        @Override
        public String toString() {
            return "ControlOutputs{" +
                    "thrust=" + thrust +
                    ", leftWingInclination=" + leftWingInclination*RAD2DEGREE +
                    ", rightWingInclination=" + rightWingInclination*RAD2DEGREE +
                    ", horStabInclination=" + horStabInclination*RAD2DEGREE +
                    ", verStabInclination=" + verStabInclination*RAD2DEGREE +
                    ", leftBrakeForce=" + leftBrakeForce +
                    ", rightBrakeForce=" + rightBrakeForce +
                    ", frontBrakeForce=" + frontBrakeForce +
                    '}';
        }
    }
    
    /**
     * dummy data used for the initialization of the drone
     */
    private  static AutopilotInputs_v2 dummyData = new AutopilotInputs_v2() {
        @Override
        public byte[] getImage() {
            return new byte[0];
        }

        @Override
        public float getX() {
            return 0;
        }

        @Override
        public float getY() {
            return 0f;//DroneBuilder_v2.START_Y;//DroneBuilder.GAMMA_STARTPOS.getyValue();
        }

        @Override
        public float getZ() {
            return 0;
        }

        @Override
        public float getHeading() {
            return (float) (0);
        }

        @Override
        public float getPitch() {
            return 0;
        }

        @Override
        public float getRoll() {
            return 0;
        }

        @Override
        public float getElapsedTime() {
            return 0;
        }
    };

    class VectorPID{
        VectorPID(float gainConstant, float integralConstant, float derivativeConstant){
            // set the constants
            this.gainConstant = gainConstant;
            this.integralConstant = integralConstant;
            this.derivativeConstant = derivativeConstant;
        }

        /**
         * Calculates the output for the current inputs of the PID controller
         * @param input the input signal of the controller (from the feedback loop)
         * @param elapsedTime the elapsed time during the simulation
         * @return the output of the PID controller for the given inputs
         * note: algorithm comes from https://en.wikipedia.org/wiki/PID_controller
         */
        Vector getPIDOutput(Vector input, float elapsedTime){

            // P part is proportional (set to 1)
            // I part reduces overall error
            // D part reduces the oscillation of the path
            // variables needed for calculation
            Vector setPoint = this.getSetPoint();
            Vector prevError = this.getPreviousError();
            Vector integral = this.getIntegral();
            float Kp = this.getGainConstant();
            float Ki = this.getIntegralConstant();
            float Kd = this.getDerivativeConstant();
            float deltaTime = elapsedTime - this.getPreviousTime();

            //determine the PID control factors
            Vector error = setPoint.vectorDifference(input);
            Vector derivative = (error.vectorDifference(prevError)).scalarMult(1/deltaTime);
            integral = (error.scalarMult(deltaTime)).vectorSum(integral);

            // calculate the output
            Vector[] outputArray = {error.scalarMult(Kp),integral.scalarMult(Ki), derivative.scalarMult(Kd)};
            Vector output = Vector.sumVectorArray(outputArray);

            // save the state
            this.setIntegral(integral);
            this.setPreviousError(error);
            this.setPreviousTime(elapsedTime);

            return output;
        }

        private Vector getIntegral() {
            return integral;
        }

        private void setIntegral(Vector integral) {
            this.integral = integral;
        }

        private Vector getPreviousError() {
            return previousError;
        }

        private void setPreviousError(Vector previousError) {
            this.previousError = previousError;
        }

        private Vector getSetPoint() {
            return setPoint;
        }

        public void setSetPoint(Vector setPoint) {
            this.setPoint = setPoint;
        }

        private float getPreviousTime() {
            return previousTime;
        }

        private void setPreviousTime(float previousTime) {
            this.previousTime = previousTime;
        }

        private float getGainConstant() {
            return gainConstant;
        }

        private void setGainConstant(float gainConstant) {
            this.gainConstant = gainConstant;
        }

        private float getIntegralConstant() {
            return integralConstant;
        }

        private void setIntegralConstant(float integralConstant) {
            this.integralConstant = integralConstant;
        }

        private float getDerivativeConstant() {
            return derivativeConstant;
        }

        private void setDerivativeConstant(float derivativeConstant) {
            this.derivativeConstant = derivativeConstant;
        }

        private Vector integral = new Vector();
        private Vector previousError = new Vector();
        private Vector setPoint =new Vector();
        private float previousTime = 0.0f;
        private float gainConstant;
        private float integralConstant;
        private float derivativeConstant;
    }

    class PIDController {
        /**
         * Constructor for a PID controller object
         * @param gainConstant the constant for the gain of de PID controller (also denoted as Kp)
         * @param integralConstant the constant for the integral of the PID controller (also denoted as Ki)
         * @param derivativeConstant the constant for the derivative of the PID controller (also denoted as Kd)
         */
        PIDController(float gainConstant, float integralConstant, float derivativeConstant){
            // set the constants
            this.gainConstant = gainConstant;
            this.integralConstant = integralConstant;
            this.derivativeConstant = derivativeConstant;
        }

        /**
         * Constructs a PID controller with the gain, integral and derivative parameters set to 1.0
         */
        private PIDController(){
            this(1.0f, 1.0f, 1.0f);
        }


        /**
         * Calculates the output for the current inputs of the PID controller
         * @param input the input signal of the controller (from the feedback loop)
         * @param deltaTime the time step between two PID entries
         * @return the output of the PID controller for the given inputs
         * note: algorithm comes from https://en.wikipedia.org/wiki/PID_controller
         */
        float getPIDOutput(float input, float deltaTime){

            // P part is proportional (set to 1)
            // I part reduces overall error
            // D part reduces the oscillation of the path
            // variables needed for calculation
            float setPoint = this.getSetPoint();
            float prevError = this.getPreviousError();
            float integral = this.getIntegral();
            float Kp = this.getGainConstant();
            float Ki = this.getIntegralConstant();
            float Kd = this.getDerivativeConstant();

            //determine the PID control factors
            float error = setPoint - input;
            float derivative = (error - prevError)/deltaTime;
            integral = integral + error*deltaTime;

            // calculate the output
            float output = Kp * error + Ki*integral + Kd*derivative;

            // save the state
            this.setIntegral(integral);
            this.setPreviousError(error);

            return output;
        }

        /**
         * Resets the current PID by setting the integral part to zero
         * so the PID can be reused after some time of de-activation
         */
        public void reset(){
            this.integral = 0;
        }

        /*
        Getters and setters
         */

        /**
         * get the integral part (saved over the course of the algorithm)
         * @return the integral part of the PID
         */
        private float getIntegral() {
            return integral;
        }

        /**
         * set the integral part of the PID (saved over the course of the algorithm)
         * @param integral
         */
        private void setIntegral(float integral) {
            this.integral = integral;
        }

        /**
         * get the error of the previous iteration
         * @return the previous error
         */
        private float getPreviousError() {
            return previousError;
        }

        /**
         * Set the previous error (used when the new values for the error are loaded)
         * @param previousError
         */
        private void setPreviousError(float previousError) {
            this.previousError = previousError;
        }

        /**
         * The set point is the desired value used for reference, in our case it is 0.0
         * @return
         */
        public float getSetPoint() {
            return setPoint;
        }

        /**
         * Set the set point of the PID
         * @param setPoint
         */
        public void setSetPoint(float setPoint) {
            this.setPoint = setPoint;
        }

        /**
         * Constant used for the gain (proportional) part of the PID
         * @return
         */
        private float getGainConstant() {
            return gainConstant;
        }

        /**
         * Constant used for the integral part of the PID
         * @return
         */
        private float getIntegralConstant() {
            return integralConstant;
        }

        /**
         * Constant used for the derivative part of the PID
         * @return
         */
        private float getDerivativeConstant() {
            return derivativeConstant;
        }

        public void setGainConstant(float gainConstant) {
            this.gainConstant = gainConstant;
        }

        public void setIntegralConstant(float integralConstant) {
            this.integralConstant = integralConstant;
        }

        public void setDerivativeConstant(float derivativeConstant) {
            this.derivativeConstant = derivativeConstant;
        }

        private float integral = 0.0f;
        private float previousError = 0.0f;
        private float setPoint = 0.0f;
        private float gainConstant;
        private float integralConstant;
        private float derivativeConstant;
        private float prevCalcOutput;
    }
}
