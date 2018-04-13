package internal.Autopilot;

import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotInputs;
import AutopilotInterfaces.AutopilotOutputs;
import AutopilotInterfaces.AutopilotInputs_v2;
import internal.Autopilot.Controller.ControlOutputs;
import internal.Exceptions.NoCubeException;
import internal.Helper.SquareMatrix;
import internal.Helper.Vector;

import static java.lang.Math.*;

//TODO base the incrementation of the wings on the current outputs instead of the previous
/**
 * Created by Martijn on 19/02/2018.
 * A flight controller made for the 15Â° AOA assignment
 * TODO: Implement the controller fully
 */
public class GammaFlightController extends AutoPilotFlightController {

    public GammaFlightController(AutoPilot autoPilot) {
        super(autoPilot);
        System.out.println("using gamma controller");
    }


    public ControlOutputs getControlActions(AutopilotInputs_v2 inputs){
        this.setCurrentInputs(inputs);
        ControlOutputs outputs = new ControlOutputs();
        AutoPilotCamera APCamera = this.getAutopilot().getAPCamera();
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
        PIDController xPIDController = this.getxPID();
        PIDController yPIDController = this.getyPID();

        APCamera.loadNewImage(currentInputs.getImage());
        float elapsedTime = this.getCurrentInputs().getElapsedTime();

        Vector center;

        try{
            center = APCamera.getCenterOfNCubes(1);
            System.out.println("CubeCenter: " + center);
        }catch(NoCubeException e){
            center = new Vector(-10, 0, 4);
        }

        //center = rollCorrectCenterCubes(inputs, center);

        //FOR DEBUGGING
        //System.out.println(center);
        //END FOR DEBUGGING

        float deltaTime = Controller.getDeltaTime(prevInputs, currentInputs);
        float xPosition = xPIDController.getPIDOutput(-center.getxValue(), deltaTime);
        float yPosition = yPIDController.getPIDOutput(center.getyValue(), deltaTime);
        int nbColumns = APCamera.getNbColumns();
        int nbRows = APCamera.getNbRows();
        float cubeCoeff = (float) min(MAX_CUBE_COEFF, sqrt(nbRows*nbColumns)/center.getzValue());
        //System.out.println("PID positions x= " + xPosition + " ; y= " + yPosition);
        //System.out.println("Cube coefficients: " + cubeCoeff);
        xControlActions(outputs, xPosition,cubeCoeff);
        yControlActions(outputs, yPosition, cubeCoeff, currentInputs.getPitch());
        setThrustOut(outputs, cubeCoeff);

        //System.out.println("Outputs Horizontal: " + outputs.getHorStabInclination()*RAD2DEGREE + "; Vertical: " + outputs.getVerStabInclination()*RAD2DEGREE );

        rollControl(outputs, this.getCurrentInputs());
        angleOfAttackControl(outputs, this.getPreviousInputs(), this.getCurrentInputs());

        return outputs;
    }

    /**
     * Corrects the inputs from the camera for roll effects from piloting the drone
     * @param inputs the inputs used to extract the current roll of the drone
     */
    private Vector rollCorrectCenterCubes(AutopilotInputs_v2 inputs, Vector currentCoord){
        //get the roll from the inputs
        float roll = inputs.getRoll();
        System.out.println("Current roll: " + roll);
        //make the transformation matrix
        SquareMatrix rollMatrix = new SquareMatrix(new float[]{(float) cos(roll), (float) -sin(roll), 0,
                (float) sin(roll), (float) cos(roll) , 0,
                0               ,  0                , 1});
        //then transform the inputs back to the original:
        Vector correctedVector = rollMatrix.matrixVectorProduct(currentCoord);
        //save the corrected form
       return correctedVector;
        //return
    }

    @Override
    protected void rollControl(Controller.ControlOutputs outputs, AutopilotInputs_v2 currentInput){
        float roll = currentInput.getRoll();

        if(roll >= this.getRollThreshold()&&isSteeringLeft(outputs)){
            outputs.setRightWingInclination(this.getMainStableInclination());
            outputs.setLeftWingInclination(this.getMainStableInclination());
        }
        else if(roll <= - this.getRollThreshold()&&isSteeringRight(outputs)){
            outputs.setLeftWingInclination(this.getMainStableInclination());
            outputs.setRightWingInclination(this.getMainStableInclination());
        }else{
            // change nothing
        }
    }

    private boolean isSteeringRight(Controller.ControlOutputs outputs){
        return outputs.getRightWingInclination() < this.getMainStableInclination();
    }

    private boolean isSteeringLeft(Controller.ControlOutputs outputs){
        return outputs.getRightWingInclination() > this.getMainStableInclination();
    }

    private void xControlActions(ControlOutputs outputs, float xPos, float cubeCoeff){
        float horizontalStabIncl = STABILIZER_STABLE_INCLINATION;
        float rightMainIncl = MAIN_STABLE_INCLINATION;
        float leftMainIncl = MAIN_STABLE_INCLINATION;
        float roll = this.getCurrentInputs().getRoll();
        if(xPos > X_THRESHOLD){
            // cube coeff: to increase pitch for faraway objects
            // squared for large corrections if large error
            horizontalStabIncl = (float) (signum(xPos) * min(MAX_HOR_STAB_INCLINATION, STANDARD_VER_STAB_INCL*pow(abs(xPos)/1f,2))); //*cube coeff
            leftMainIncl = (float) (signum(xPos)*sqrt(abs(roll))* TURNING_INCLINATION +  MAIN_STABLE_INCLINATION);
            rightMainIncl = 0;
        }else if(xPos < X_THRESHOLD*(signum(xPos))){
            horizontalStabIncl = (float) (signum(xPos) * min(MAX_HOR_STAB_INCLINATION, STANDARD_VER_STAB_INCL*pow(abs(xPos)/1f,2))); //*cube coeff
            rightMainIncl = (float) (-signum(xPos)*sqrt(abs(roll))*TURNING_INCLINATION +  MAIN_STABLE_INCLINATION);
            leftMainIncl = 0;
        }
        outputs.setHorStabInclination(horizontalStabIncl);
        outputs.setRightWingInclination(rightMainIncl);
        outputs.setLeftWingInclination(leftMainIncl);
    }

    private void yControlActions(ControlOutputs outputs, float yPos, float cubeCoeff, float pitch){
        float horizontalStabIncl;
        //TODO verify if this works
        yPos = (float) (yPos*(1+pitch*2/PI));
        if(abs(yPos) > Y_THRESHOLD){
            horizontalStabIncl = (float) (-signum(yPos) * min(MAX_HOR_STAB_INCLINATION, STANDARD_HOR_STAB_INCLINATION*cubeCoeff*pow(abs(yPos)/1f,2)));
        }else{
            horizontalStabIncl = STABILIZER_STABLE_INCLINATION;
        }
        outputs.setHorStabInclination(horizontalStabIncl);
    }

    private void setThrustOut(ControlOutputs outputs, float cubeCoeff){
        //Todo implement: write the output to the outputs
        float pitch = this.getCurrentInputs().getPitch();
        float maxThrust =  this.getAutopilot().getConfig().getMaxThrust();
        int threshold = Math.round(THRESHOLD_DISTANCE);
        float gravity = this.getAutopilot().getConfig().getGravity();

        // Thrust
        float thrust = (float) ((maxThrust/4) + THRUST_FACTOR*this.getTotalMass()*gravity*cubeCoeff);
        //System.out.println("thrust: " + thrust);
        outputs.setThrust(Math.max(Math.min(thrust, maxThrust), 0));
        if (getVelocityApprox().getzValue() < -50.f || pitch < 0){
           outputs.setThrust(0f);
        }
    }
    /**
     * Calculate an approximation of the velocity
     * @return the approximation of the velocity
     * elaboration: see textbook numerical math for derivative methods, the
     * derivative of f(k+1) - f(k-1) / (2*timeStep) has O(hÂ²) correctness
     */
    public Vector getVelocityApprox(){
        //get the inputs at moment k - 1 for the derivative
        AutopilotInputs_v2 prevInputs = this.getPreviousInputs();
        //get the inputs at moment k
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        float prevTime = prevInputs.getElapsedTime();
        float currentTime = currentInputs.getElapsedTime();

        Vector prevPos = extractPosition(prevInputs);
        Vector currentPos = extractPosition(currentInputs);

        Vector posDiff = currentPos.vectorDifference(prevPos);
        float timeDiff = currentTime - prevTime;

        return posDiff.scalarMult(1/timeDiff);
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
        return ROLL_THRESHOLD;
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
    private PIDController yPID = new PIDController(1.f, 0.05f, 0.1f);
   // private PIDController rollPID = new PIDController(1f, 0.0f, 0.0f);


    //private static final float STANDARD_INCLINATION = (float) (5*PI/180);
    public  static final float MAIN_STABLE_INCLINATION = (float) (8*PI/180);//TODO Deze value is brak AF
    //public  static final float MAIN_MAX_INCLINATION = (float) (10*PI/180);
    private static final float MAX_HOR_STAB_INCLINATION = (float) (10*PI/180);
    private static final float STANDARD_HOR_STAB_INCLINATION = (float) (5*PI/180);
    //private static final float MAX_VER_STAB_INCLINATION = (float) (10*PI/180f);
    private static final float STANDARD_VER_STAB_INCL = (float) (5*PI/180f);
    private static final float TURNING_INCLINATION = (float) (10*PI/180);
    private static final float ERROR_INCLINATION_MARGIN = (float) (2*PI/180);
    //private static final int   BIAS = 0;
    private static final float THRESHOLD_DISTANCE = 1f;
    //private static final float STANDARD_THRUST = 32.859283f*2;
    private static final float THRUST_FACTOR = 2.0f;
    // private static final float THRESHOLD_THRUST_ANGLE = (float)(PI/20);
    private static final float MAX_CUBE_COEFF = 3f;
    public  static final float STABILIZER_STABLE_INCLINATION = 0.0f;
    //private static final float GRAVITY = 9.81f;
    private static final float ROLL_THRESHOLD = (float) (PI * 4f/180.0f);
    //private static final float RAD2DEGREE = (float) (180f/ PI);
    //private static final float CHECK_INTERVAL = 1/20.f;
    private static final float X_THRESHOLD = 0f;
    private static final float Y_THRESHOLD = 0f;
}

//    private void bankingTurnLeft(ControlOutputs outputs){
//        //first get the previous turn values:
//        ControlOutputs prevOutputs = this.getPreviousOutputs();
//        float leftMainInclination = prevOutputs.getLeftWingInclination();
//        float rightMainInclination = prevOutputs.getRightWingInclination();
//        float horizontalInclination = prevOutputs.getHorStabInclination();
//        float verticalInclination = prevOutputs.getVerStabInclination();
//
//
//    }
//
//    private void bankingTurnRight(ControlOutputs outputs){
//        //first get the previous turn values:
//        ControlOutputs prevOutputs = this.getPreviousOutputs();
//        float leftMainInclination = prevOutputs.getLeftWingInclination();
//        float rightMainInclination = prevOutputs.getRightWingInclination();
//        float horizontalInclination = prevOutputs.getHorStabInclination();
//        float verticalInclination = prevOutputs.getVerStabInclination();
//    }
//
//    private void descend(ControlOutputs outputs){
//        //first get the previous turn values:
//        ControlOutputs prevOutputs = this.getPreviousOutputs();
//        float leftMainInclination = prevOutputs.getLeftWingInclination();
//        float rightMainInclination = prevOutputs.getRightWingInclination();
//        float horizontalInclination = prevOutputs.getHorStabInclination();
//        float verticalInclination = prevOutputs.getVerStabInclination();
//    }
//
//    private void ascend(ControlOutputs outputs){
//        //first get the previous turn values:
//        ControlOutputs prevOutputs = this.getPreviousOutputs();
//        float leftMainInclination = prevOutputs.getLeftWingInclination();
//        float rightMainInclination = prevOutputs.getRightWingInclination();
//        float horizontalInclination = prevOutputs.getHorStabInclination();
//        float verticalInclination = prevOutputs.getVerStabInclination();
//    }
//
//    //TODO implement that steers to steady outputs (steady main wing inclination and such)
//    private void steady(ControlOutputs outputs){
//        //first get the previous turn values:
//        ControlOutputs prevOutputs = this.getPreviousOutputs();
//        float leftMainInclination = prevOutputs.getLeftWingInclination();
//        float rightMainInclination = prevOutputs.getRightWingInclination();
//        float horizontalInclination = prevOutputs.getHorStabInclination();
//        float verticalInclination = prevOutputs.getVerStabInclination();
//
//        //for the right wing
//        if(rightMainInclination > getMainStableInclination()){
//            decrementMainRight(outputs, getMainStableInclination());
//        }else{
//            incrementMainRight(outputs, getMainStableInclination());
//        }
//
//        //for the left wing
//        if(leftMainInclination > getMainStableInclination()){
//            decrementMainLeft(outputs, getMainStableInclination());
//        }else{
//            incrementMainLeft(outputs, getMainStableInclination());
//        }
//
//        //for the horizontal wing
//        if(horizontalInclination > getStabilizerStableInclination()){
//            decrementHorizontalStabilizer(outputs, getStabilizerStableInclination());
//        }else{
//            incrementHorizontalStabilizer(outputs, getStabilizerStableInclination());
//        }
//    }
//
//    /**
//     * Checks if there are control actions that need to be taken
//     * @param PIDAdjustedCenters the center vector (adjusted by the PID) the x and y coordinates contain
//     *                           the on screen coordinate of the cube
//     *                           the z coordinate contains the size of the cube
//     * @return true if control actions must be taken
//     */
//    private boolean needsActions(Vector PIDAdjustedCenters){
//        //if we have to take control actions, the adjusted center modulus has to be located outside
//        //the "idle" circle
//        //the maximum size on screen:
//        AutopilotConfig config = this.getAutopilot().getConfig();
//        int maxSize = config.getNbColumns() * config.getNbRows();
//
//        //then get the relative difference in on screen size and max size
//        // z-coordinate contains the on screen size of the cube
//        float relDeltaSize = (maxSize - PIDAdjustedCenters.getzValue())/maxSize;
//        float scaleConst = 0f;
//
//        //the rel deltaSize serves as a multiplier for the real length of the on screen vector
//        //stating the fact that controls must be done earlier if the drone is faraway from the cube
//        //and that we should refrain from control actions if the drone is already big on screen
//
//        //calculate the on screen size
//        float xPos = PIDAdjustedCenters.getxValue();
//        float yPos = PIDAdjustedCenters.getyValue();
//        float xySize = (float) sqrt(xPos*xPos + yPos*yPos);
//
//        return xySize*( relDeltaSize*scaleConst + 1 ) >= getActionThreshold();
//
//    }
//
//    @Override
//    public AutopilotOutputs getControlActions(AutopilotInputs inputs) {
//        this.setCurrentInputs(inputs);
//        ControlOutputs outputs = this.getPreviousOutputs().copy(); //get a copy from the previous outputs
//        //get the middle of the cubes:
//        AutoPilotCamera APCamera = this.getAutopilot().getAPCamera();
//        APCamera.loadNewImage(getCurrentInputs().getImage());
//
//        //the elapsed time needed for the PID
//        float elapsedTime = this.getCurrentInputs().getElapsedTime();
//
//        Vector onScreenCubeCenter = new Vector();
//
//        try {
//            onScreenCubeCenter = APCamera.getCenterOfNCubes(1);
//        } catch (NoCubeException e) {
//            //e.printStackTrace();
//            //do nothing, maybe a transition later
//        }
//
//        float xPosition = this.getxPid().getPIDOutput(-onScreenCubeCenter.getxValue(), elapsedTime);
//        float yPosition = this.getyPID().getPIDOutput(onScreenCubeCenter.getyValue(), elapsedTime);
//
//        Vector PIDAdjustedPos = new Vector(xPosition, yPosition, onScreenCubeCenter.getzValue());
//        //now get the controls for each region of the cube position
//        //first check if any controls are needed
//        if(needsActions(PIDAdjustedPos)){
//            //control actions are needed
//            //do actions based on the angle
//            float angle = (float) atan2(yPosition, xPosition);
//
//        }else{
//            //no actions are needed, only assure that we remain steady
//            steady(outputs);
//        }
//
//        //save the previous outputs
//        setPreviousOutputs(outputs);
//        return outputs;
//    }
//
//    /**
//     * Controlling actions for the pitch, as long as the pitch exceeds the thresholds adjust the horizontal stabilizers
//     * @param outputs
//     */
//    private void pitchControl(ControlOutputs outputs){
//        AutopilotInputs inputs = this.getCurrentInputs();
//        float pitch = inputs.getPitch();
//        if(pitch > getPitchThreshold()){
//            this.incrementHorizontalStabilizer(outputs);
//        }else if(pitch < -getPitchThreshold()){
//            this.decrementHorizontalStabilizer(outputs);
//        }
//        ///else do nothing
//    }
//
//    private void rollControll(ControlOutputs outputs){
//
//    }
//
//    /**
//     * Increments the current value of horizontal stabilizer in the outputs with a fixed amount, if the new inclination exceeds the max inclination
//     * the value is set to this provided value
//     * @param outputs the outputs of the controller
//     * @param maxInclination the maximal inclination
//     * note: the increment value is given by getStabilizerInclinationDelta();
//     */
//    private void incrementHorizontalStabilizer(ControlOutputs outputs, float maxInclination){
//        //first get the previous value for the stabilizer
//        float horStabInclination = outputs.getHorStabInclination();
//        float newInclination = horStabInclination + getStabilizerInclinationDelta();
//        if(newInclination > maxInclination){
//            outputs.setHorStabInclination(maxInclination);
//        }else{
//            outputs.setHorStabInclination(newInclination);
//        }
//    }
//
//    private void incrementHorizontalStabilizer(ControlOutputs outputs){
//        incrementHorizontalStabilizer(outputs, Float.MAX_VALUE);
//    }
//
//    /**
//     * Decrements the inclination of the horizontal stabilizer in the outputs
//     * with a standard amount, if the minimal inclination is reached, no further decrement is done
//     * @param outputs the outputs of the controller (here the value is modified)
//     * @param minInclination the min cap for the decrement of the inclination
//     * note: the decrement value is given by getStabilizerInclinationDelta();
//     */
//    private void decrementHorizontalStabilizer(ControlOutputs outputs, float minInclination){
//        //first get the previous value for the stabilizer
//        float horStabInclination = outputs.getHorStabInclination();
//        float newInclination = horStabInclination - getStabilizerInclinationDelta();
//        if(newInclination < minInclination){
//            outputs.setHorStabInclination(minInclination);
//        }else{
//            outputs.setHorStabInclination(newInclination);
//        }
//    }
//
//    /**
//     * Decrements the horizontal stabilizer inclination without any cap
//     * @param outputs the outputs of the controller
//     */
//    private void decrementHorizontalStabilizer(ControlOutputs outputs){
//        decrementHorizontalStabilizer(outputs, -Float.MAX_VALUE);
//    }
//
//    /**
//     * Increments the main left inclination of the current outputs with a fixed incrementation
//     * if the inclination exceeds the max inclination, it is set to the max inclination
//     * @param outputs the outputs for the controller (here the inclination is modified
//     * @param maxInclination the maximal inclination
//     */
//    private void incrementMainLeft(ControlOutputs outputs, float maxInclination){
//        float leftMainInclination = outputs.getLeftWingInclination();
//        float newInclination = leftMainInclination + getMainInclinationDelta();
//        if(newInclination > maxInclination){
//            outputs.setLeftWingInclination(maxInclination);
//        }else{
//            outputs.setLeftWingInclination(newInclination);
//        }
//    }
//
//    private void incrementMainLeft(ControlOutputs outputs){
//        incrementMainLeft(outputs, Float.MAX_VALUE);
//    }
//
//    /**
//     * Decrements the main left inclination in the given outputs, if the decrement exceeds the minimal
//     * inclination, the inclination is set to this provided minimum
//     * @param outputs the outputs of the autopilot
//     * @param minInclination the minimal inclination
//     * note: the decrement value is given by getMainInclinationDelta();
//     */
//    private void decrementMainLeft(ControlOutputs outputs, float minInclination){
//        float rightMainInclination = outputs.getRightWingInclination();
//        float newInclination = rightMainInclination - getMainInclinationDelta();
//        if(newInclination < minInclination){
//            outputs.setRightWingInclination(minInclination);
//        }else{
//            outputs.setRightWingInclination(newInclination);
//        }
//    }
//
//    private void decrementMainLeft(ControlOutputs outputs){
//        decrementMainLeft(outputs, -Float.MAX_VALUE);
//    }
//
//    /**
//     * Increments the main right wing inclination in the given outputs, if this value exceeds the upper limit (maxIncl)
//     * it is set to the maximal inclination allowed
//     * @param outputs the outputs of the autopilot controller
//     * @param maxInclination the maximal inclination of said wing
//     */
//    private void incrementMainRight(ControlOutputs outputs, float maxInclination){
//        float rightMainInclination = outputs.getRightWingInclination();
//        float newInclination = rightMainInclination + getMainInclinationDelta();
//        if(newInclination > maxInclination){
//            outputs.setRightWingInclination(maxInclination);
//        }else{
//            outputs.setRightWingInclination(newInclination);
//        }
//    }
//
//    private void incrementMainRight(ControlOutputs outputs){
//        incrementMainRight(outputs, Float.MAX_VALUE);
//    }
//
//    /**
//     * Decrements the main right wing inclination in the given outputs, if this value exceeds the lower limit (minIncl)
//     * it is set to the minimal inclination allowed
//     * @param outputs the outputs of the autopilot controller
//     * @param minInclination the minimal inclination of said wing
//     */
//    private void decrementMainRight(ControlOutputs outputs, float minInclination){
//        float rightMainInclination = outputs.getRightWingInclination();
//        float newInclination = rightMainInclination - getMainInclinationDelta();
//        if(newInclination > minInclination){
//            outputs.setRightWingInclination(minInclination);
//        }else{
//            outputs.setRightWingInclination(newInclination);
//        }
//    }
//
//    @Override
//    protected float getMainStableInclination() {
//        return MAIN_STABLE_INCLINATION;
//    }
//
//    @Override
//    protected float getStabilizerStableInclination() {
//        return STABILIZER_STABLE_INCLINATION;
//    }
//
//    @Override
//    protected float getRollThreshold() {
//        return ROLL_THRESHOLD;
//    }
//
//    @Override
//    protected float getInclinationAOAErrorMargin() {
//        return this.getAutopilot().getConfig().getMaxAOA();
//    }
//
//    @Override
//    protected float getStandardThrust() {
//        return STANDARD_THRUST;
//    }
//
//    public static float getMainInclinationDelta() {
//        return MAIN_INCLINATION_DELTA;
//    }
//
//    public static float getMainMaxInclination() {
//        return MAIN_MAX_INCLINATION;
//    }
//
//    public static float getStabilizerInclinationDelta() {
//        return STABILIZER_INCLINATION_DELTA;
//    }
//
//    public static float getStabilizerMaxInclination() {
//        return STABILIZER_MAX_INCLINATION;
//    }
//
//    public static float getPitchThreshold() {
//        return PITCH_THRESHOLD;
//    }
//
//    public static float getActionThreshold() {
//        return ACTION_THRESHOLD;
//    }
//
//    /**
//     * getter for the previous outputs of the controller
//     * @return the pevious outputs of the controller
//     */
//    private ControlOutputs getPreviousOutputs() {
//        return previousOutputs;
//    }
//
//    /**
//     * Setter for the previous outputs of the controller
//     * @param previousOutputs the previous outputs of the controller
//     */
//    private void setPreviousOutputs(ControlOutputs previousOutputs) {
//        this.previousOutputs = previousOutputs;
//    }
//
//
//    public PIDController getyPid() {
//        return yPid;
//    }
//
//    public void setyPid(PIDController yPid) {
//        this.yPid = yPid;
//    }
//
//    public PIDController getxPid() {
//        return xPid;
//    }
//
//    public void setxPid(PIDController xPid) {
//        this.xPid = xPid;
//    }
//
//    /**
//     * Instance that holds the previous outputs of the drone
//     */
//    private ControlOutputs previousOutputs = new ControlOutputs();
//
//    /**
//     * Instance that holds the PID controller for drone y axis image
//     */
//    private PIDController yPid = new PIDController(1.f, 0.f, 0.f);
//
//    /**
//     * Instance that holds the PID controller for drone x axis image
//     */
//    private PIDController xPid = new PIDController(1.f, 0.f, 0.f);
//
//    public  final static float MAIN_STABLE_INCLINATION = (float) (5*PI/180);
//    private final static float MAIN_INCLINATION_DELTA = (float) (1*PI/180);
//    private final static float MAIN_MAX_INCLINATION = (float)(10*PI/180);
//    public  final static float STABILIZER_STABLE_INCLINATION = 0f;
//    private final static float STABILIZER_INCLINATION_DELTA = (float) (2*PI/180);
//    private final static float STABILIZER_MAX_INCLINATION = (float)(5*PI/180);
//    private final static float ROLL_THRESHOLD = (float) (15*PI/180);
//    private final static float PITCH_THRESHOLD =(float) (10*PI/180);
//    private final static float ACTION_THRESHOLD = 5;
//    private final static float ACTIVATION_ANGLE =
//
//    //temporary constant, get more appropriate value
//    private final static float STANDARD_THRUST = 500f;


//    /**
//     * Generates the control actions of the autopilot that will be passed to the drone
//     * @param inputs the inputs of the autopilot
//     * @return an autopilot outputs object that contains the instructions for the testbed
//     */
//    @Override
//    public AutopilotOutputs getControlActions(AutopilotInputs inputs) {
//        this.setCurrentInputs(inputs);
//        ControlOutputs outputs = new ControlOutputs();
//    	// If all blocks were hit, start landingprocedure
//    	AutoPilotCamera APCamera = this.getAutopilot().getAPCamera();
//        APCamera.loadNewImage(inputs.getImage());
//        AutopilotInputs currentInputs = this.getCurrentInputs();
//        PIDController xPIDController = this.getxPID();
//        PIDController yPIDController = this.getyPID();
//
//        int amountOfCubesInSight = APCamera.getCubesInPicture().size();
//
////        if (amountOfCubesInSight <= 0) {
////        	this.getAutopilot().setAPMode(3);
////        }
//
//        float elapsedTime = this.getCurrentInputs().getElapsedTime();
//
//        Vector center;
//
//        try{
//            center = APCamera.getCenterOfNCubes(1);
//        }catch(NoCubeException e){
//            center = new Vector(-10, 0, 4);
//        }
//
//        //FOR DEBUGGING
//        //System.out.println(center);
//        //END FOR DEBUGGING
//
//        float xPosition = xPIDController.getPIDOutput(-center.getxValue(), elapsedTime);
//        float yPosition = yPIDController.getPIDOutput(center.getyValue(), elapsedTime);
//        int nbColumns = APCamera.getNbColumns();
//        int nbRows = APCamera.getNbRows();
//        float cubeCoeff = (float) min(MAX_CUBE_COEFF, sqrt(nbRows*nbColumns)/center.getzValue());
//        //System.out.println("PID positions x= " + xPosition + " ; y= " + yPosition);
//        //System.out.println("Cube coefficients: " + cubeCoeff);
//        xControlActions(outputs, xPosition,cubeCoeff);
//        yControlActions(outputs, yPosition, cubeCoeff, currentInputs.getPitch());
//        setThrustOut(outputs, cubeCoeff);
//
//        //System.out.println("Outputs Horizontal: " + outputs.getHorStabInclination()*RAD2DEGREE + "; Vertical: " + outputs.getVerStabInclination()*RAD2DEGREE );
//
//        rollControl(outputs, this.getCurrentInputs());
//        angleOfAttackControl(outputs, this.getPreviousInputs(), this.getCurrentInputs());
//
//        return outputs;
//    }