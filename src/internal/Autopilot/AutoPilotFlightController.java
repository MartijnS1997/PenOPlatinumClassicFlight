package internal.Autopilot;

import AutopilotInterfaces.*;
import AutopilotInterfaces.Path;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;
import internal.Testbed.FlightRecorder;

import java.util.List;

import static java.lang.Math.*;

/**
 * Created by Martijn on 30/10/2017.
 * Appended and edited by Anthony Rath� on 6/11/2017
 * A class of AutopilotInterfaces Controllers
 */
public abstract class AutoPilotFlightController extends Controller{


	/**
     * Constructor for the autopilotController
     * @param autopilot
     */
    public AutoPilotFlightController(AutoPilot autopilot){
        super(autopilot);
        
    }

    @Override
    public boolean hasReachedObjective(AutopilotInputs_v2 inputs) {
        //if there are no cubes visible, we have lost visual contact, we start our landing sequence
        //first do the simple check
        //if it fails, we'll do some more calculations
        if(this.cubeVisibleOnCamera()){
            return false;
        }
        //there was no drone visible on the camera, see if there are cubes in front of the drone
        return !cubeInFrontOfDrone(inputs);
    }

    /**
     * Checks if there are any cubes that can be reached in front of the drone (checks if there are in front of it)
     * @param inputs the current autopilot inputs
     * @return true if and only if there at least one cube in front of the drone
     */
    private boolean cubeInFrontOfDrone(AutopilotInputs_v2 inputs){
        Path path = this.getAutopilot().getPath();
        List<Vector> cubePositions = Controller.extractPath(path);
        Vector dronePosition = Controller.extractPosition(inputs);
        Vector droneOrientation = Controller.extractOrientation(inputs);
        for(Vector cubePosition: cubePositions){
            if(inFrontOfDrone(cubePosition, dronePosition, droneOrientation )){
                return true;
            }
        }

        return false;

    }

    /**
     * Checks if the provided cube position is in front of the drone or not
     * @param cubePosition the position of the cube
     * @param dronePosition the position of the drone
     * @param droneOrientation the orientation of the drone
     * @return true if and only if the cube is in front of the drone
     */
    private static boolean inFrontOfDrone(Vector cubePosition, Vector dronePosition, Vector droneOrientation){
        // take the difference
        Vector diffVectorWorld = cubePosition.vectorDifference(dronePosition);
        //transform it onto the drone axis system
        Vector diffVectorDrone = PhysXEngine.worldOnDrone(diffVectorWorld, droneOrientation);
        //then project the difference vector on the heading vector
        Vector headingVector = new Vector(0,0,-1);
        Vector projectedDiff = headingVector.projectOnVector(diffVectorDrone);
        //check the scalar product of the projection, if positive we're behind, if not we're ahead
        return projectedDiff.scalarProduct(headingVector) > 0;
    }

    /**
     * Getter for the approximate flight path of the drone
     * the path contains the positions of the cubes within a margin of 5 meters
     * @return the flight path
     */
    private AutopilotInterfaces.Path getFlightPath() {
        return flightPath;
    }

    /**
     * Setter for the flight path of the drone for this flightcontroller
     * @param flightPath the position of the cubes
     */
    public void setFlightPath(Path flightPath) {
        if(!canHaveAsFlightPath(flightPath)){
            throw new IllegalStateException("Path already configured");
        }
        this.flightPath = flightPath;
    }

    /**
     * Checks if the provided path can be set as the approximate flight path for the controller
     * @param flightPath the flight path that approximates the positions of the cubes to be tested
     * @return true if the provided flight path is a non null and the current flight path is (uninitialized)
     */
    private boolean canHaveAsFlightPath(Path flightPath){
        return flightPath!=null &&this.getFlightPath() == null;
    }

    //    private void logControlActions(ControlOutputs outputs, String controlString){
//
//        controlString += "Left wing inclination: "  + outputs.getLeftWingInclination()*RAD2DEGREE + "\n";
//        controlString += "Right wing inclination: " + outputs.getRightWingInclination()*RAD2DEGREE + "\n";
//        controlString += "Horizontal stabilizer inclination: " + outputs.getHorStabInclination()*RAD2DEGREE + "\n";
//        controlString += "Vertical wing inclination" + outputs.getVerStabInclination()*RAD2DEGREE + "\n";
//
//        // write the controls to the recorder
//        FlightRecorder recorder = this.getFlightRecorder();
//        recorder.appendControlLog(controlString);
//    }

    /**
     * determines the largest value of both entries, if both are negative an exception is thrown
     * @param entry1 the first entry to check
     * @param entry2 the second entry to check
     * @return the largest entry of both
     * @throws IllegalArgumentException thrown if both entries are negative
     */
    private float getMaxPos(float entry1, float entry2) throws IllegalArgumentException{
        if(entry1 >= entry2 && entry1 >= 0){
            return entry1;
        }else if(entry2 >= 0){
            return entry2;
        }else{
            return 0.0f;
        }
    }

    /**
     * determines the smallest value of both entries, if both are positive an exception is thrown
     * @param entry1 the first entry to check
     * @param entry2 the second entry to check
     * @return the smallest entry of both
     * @throws IllegalArgumentException thrown if both entries are positive
     */
    private float getMinNeg(float entry1, float entry2) throws IllegalArgumentException{
        if(entry1 <= entry2 && entry1 <= 0){
            return entry1;
        }else if(entry2 <= 0){
            return entry2;
        }else{
            return 0.0f;
        }
    }

    /*
    Getters and setters
     */


//    /**
//     * get the flight recorder of the drone
//     * @return
//     */
//    public FlightRecorder getFlightRecorder() {
//        return flightRecorder;
//    }
//
//    /**
//     * Setter for the flight recorder
//     * @param flightRecorder
//     */
//    public void setFlightRecorder(FlightRecorder flightRecorder) {
//        this.flightRecorder = flightRecorder;
//    }

    /**
     * Determines the cube that is located the furthest from the initial position of the drone
     * this will be the final cube to be reached before we initialize the landing sequence
     */
    public void determineFurthestPathObject(){
        //check if already configured, if so, we may leave
        if(hasAlreadySetFurthestCube()){
            return;
        }
        //get the start position
        Vector startPos = this.getAutopilot().getStartPosition();
        //get the furthest element of the path
        Path flightPath = this.getFlightPath();
        List<Vector> flightPathList = Controller.extractPath(flightPath);
        //find the one the furthest away
        //compare the accumulator to the sta
        Vector destination = flightPathList.stream().reduce(startPos.copy(), (v1, v2) -> startPos.distanceBetween(v1) > startPos.distanceBetween(v2) ? v1 : v2);

        this.setFurthestCube(destination);
    }

    /**
     * Getter the object furthest from the start position of the drone
     * @return the path object the furthest from the initial position of the drone
     */
    private Vector getFurthestCube() {
        return furthestCube;
    }

    /**
     * Setter for the furthest path object (see getter for more info)
     * @param furthestCube the furthest path object to be set
     */
    private void setFurthestCube(Vector furthestCube) {
        if(hasAlreadySetFurthestCube()){
            //ignore the request
            return;
        }
        this.furthestCube = furthestCube;
    }

    /**
     * Checks if the furthest path object is already set
     * @return returns true if so
     */
    private boolean hasAlreadySetFurthestCube(){
        return this.getFlightPath() != null;
    }

    /**
     * Checks if there are any cubes visible on the camera
     * @return true if there are any cubes visible
     */
    private boolean cubeVisibleOnCamera(){
        AutoPilotCamera camera = this.getAutopilot().getAPCamera();
        return camera.getAllCubeCenters().size()>0;
    }

    

    /*
    Getters and setters


     */

    @Override
    protected float getStandardThrust() {
        return STANDARD_THRUST;
    }

    /**
     * Get the PID used for the corrections on the x-coordinate inputs
     * @return the pid controller
     */
    public PIDController getxPID() {
        return xPID;
    }

    /**
     * Get the PID used for the corrections on the y-coordinate inputs
     * @return
     */
    public PIDController getyPID() {
        return yPID;
    }

    
    private FlightRecorder flightRecorder;
    private PIDController xPID = new PIDController(1.f, 0.2f, 0.2f);
    private PIDController yPID = new PIDController(1.f, 0.2f, 0.2f);


    private static final float STANDARD_THRUST = 32.859283f *2;
    private static final float RAD2DEGREE = (float) (180f/ PI);
    /**
     * The distance between the drone and the closest cube to call the objective reached
     */
    private static final float OBJECTTIVE_REACHED_DISTANCE = 10.0f;

    /**
     * The flight path to be approximately taken by the drone
     */
    private AutopilotInterfaces.Path flightPath;

    /**
     * The position of the cube that is located the furthest from the drone
     */
    private Vector furthestCube;


}
