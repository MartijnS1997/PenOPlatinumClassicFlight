package internal.Autopilot;

import AutopilotInterfaces.AutopilotInputs;
import AutopilotInterfaces.AutopilotInputs_v2;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Exceptions.NoCubeException;
import internal.Helper.Vector;

import static java.lang.Math.*;
import static java.lang.Math.abs;
import static java.lang.Math.pow;

/**
 * Created by Martijn on 7/12/2017.
 * A controller for the 30 degree maxAOA
 */
public class BetaFlightController extends AutoPilotFlightController {

    public BetaFlightController(AutoPilot autoPilot){
        super(autoPilot);
    }

    /**
     * Generayes the control actions of the autopilot that will be passed to the drone
     * @param inputs the inputs of the autopilot
     * @return an autopilot outputs object that contains the instructions for the testbed
     */
    @Override
    public AutopilotOutputs getControlActions(AutopilotInputs_v2 inputs){
        this.setCurrentInputs(inputs);
        ControlOutputs outputs = new ControlOutputs();
        AutoPilotCamera APCamera = this.getAutopilot().getAPCamera();
        AutopilotInputs_v2 currentInputs = this.getCurrentInputs();
        PIDController xPIDController = this.getxPID();
        PIDController yPIDController = this.getyPID();

        APCamera.loadNewImage(currentInputs.getImage());
        float elapsedTime = this.getCurrentInputs().getElapsedTime();

        Vector center;

        try{
            center = APCamera.getCenterOfNCubes(1);
        }catch(NoCubeException e){
            center = new Vector(-10, 0, 4);
        }
        
        //FOR DEBUGGING
        //System.out.println(center);
        //END FOR DEBUGGING
        
        float xPosition = xPIDController.getPIDOutput(-center.getxValue(), elapsedTime);
        float yPosition = yPIDController.getPIDOutput(center.getyValue(), elapsedTime);
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

        // If all blocks were hit, start landingprocedure
        APCamera.loadNewImage(currentInputs.getImage());
        int amountOfCubesInSight = APCamera.getCubesInPicture().size();
        
        if (amountOfCubesInSight <= 0) {
        	this.getAutopilot().setAPMode(APModes.WAY_POINT);
        }
        
        return outputs;
    }

    private void xControlActions(ControlOutputs outputs, float xPos, float cubeCoeff){
        float verticalStabIncl;
        float rightMainIncl = MAIN_STABLE_INCLINATION;
        float leftMainIncl = MAIN_STABLE_INCLINATION;
        float roll = this.getCurrentInputs().getRoll();
        //System.out.println("Roll: " + this.currentInputs.getRoll());
        if(abs(xPos) > X_THRESHOLD){
            // cube coeff: to increase pitch for faraway objects
            // squared for large corrections if large error
            verticalStabIncl = (float) (signum(xPos) * min(MAX_VER_STAB_ANGLE, STANDARD_VER_STAB_INCL*pow(abs(xPos)/1f,2))); //*cube coeff
            rightMainIncl = (float) (-signum(xPos)*sqrt(abs(roll))*TURNING_INCLINATION +  MAIN_STABLE_INCLINATION);
            leftMainIncl = (float) (signum(xPos)*sqrt(abs(roll))* TURNING_INCLINATION +  MAIN_STABLE_INCLINATION);

        }else{
            verticalStabIncl = STABILIZER_STABLE_INCLINATION;
        }

        outputs.setVerStabInclination(verticalStabIncl);
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

    private PIDController xPID = new PIDController(1.f, 0.1f, 0.1f);
    private PIDController yPID = new PIDController(1.f, 0.1f, 0.3f);
    private PIDController rollPID = new PIDController(1f, 0.0f, 0.0f);


    private static final float STANDARD_INCLINATION = (float) (10*PI/180);
    public  static final float MAIN_STABLE_INCLINATION = (float) (7*PI/180);
    public  static final float MAIN_MAX_INCLINATION = (float) (10*PI/180);
    private static final float MAX_HOR_STAB_INCLINATION = (float) (15*PI/180);
    private static final float STANDARD_HOR_STAB_INCLINATION = (float) (5*PI/180);
    private static final float MAX_VER_STAB_ANGLE = (float) (12.5*PI/180f);
    private static final float STANDARD_VER_STAB_INCL = (float) (5*PI/180f);
    private static final float TURNING_INCLINATION = (float) (10*PI/180);
    private static final float ERROR_INCLINATION_MARGIN = (float) (3*PI/180);
    private static final int   BIAS = 0;
    private static final float THRESHOLD_DISTANCE = 1f;
    private static final float STANDARD_THRUST = 32.859283f*2;
    private static final float THRUST_FACTOR = 2.0f;
    private static final float THRESHOLD_THRUST_ANGLE = (float)(PI/20);
    private static final float MAX_CUBE_COEFF = 3f;
    public  static final float STABILIZER_STABLE_INCLINATION = 0.0f;
    private static final float GRAVITY = 9.81f;
    private static final float ROLL_THRESHOLD = (float) (PI * 15.0f/180.0f);
    private static final float RAD2DEGREE = (float) (180f/ PI);
    private static final float CHECK_INTERVAL = 1/20.f;
    private static final float X_THRESHOLD = 0f;
    private static final float Y_THRESHOLD = 0f;
}
