package internal.Testbed;

import AutopilotInterfaces.AutopilotConfig;
import AutopilotInterfaces.AutopilotOutputs;
import internal.Helper.Vector;
import internal.Physics.PhysXEngine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import static java.lang.Math.PI;

/**
 * Created by Martijn on 26/02/2018.
 * A second version of the drone builder class suited for the construction of drones on the ground and in the air
 */
public class DroneBuilder_v2 {
    /**
     * Standard constructor for a drone builder
     */
    public DroneBuilder_v2(){
        System.out.println(TYRE_SLOPE);
    }

    public Drone createGroundDrone(){
        Map<Vector, Float> droneConfig = new HashMap<>();
        droneConfig.put(new Vector(0f, STABLE_Y_POS, 0f), 0f);
        return null;
    }

    public Drone createTestBounceDrone(){
        DroneState state = new DroneState() {
            @Override
            public Vector getPosition() {
                return new Vector(0f, WHEEL_Y_POS + TYRE_RADIUS, 0f);
            }

            @Override
            public Vector getVelocity() {
                return new Vector();
            }

            @Override
            public Vector getOrientation() {
                return new Vector();
            }

            @Override
            public Vector getRotation() {
                return new Vector();
            }

            @Override
            public float getPrevFrontTyreDelta() {
                return 0;
            }

            @Override
            public float getPrevRearLeftTyreDelta() {
                return 0;
            }

            @Override
            public float getPrevRearRightTyreDelta() {
                return 0;
            }
        };
        return new Drone(state, this.createConfig());
    }


    /**
     * Builds drones with the given orientation and rotation
     * @param droneState a map containing the position and the heading of the drone
     * @return a list of drones ready for simulation
     */
    public List<Drone> createDrones(Map<Vector, Float> droneState){
       List <Drone> droneList = new ArrayList<>();
       for(Vector position: droneState.keySet()){
           droneList.add(this.generateDrone(position, new Vector(droneState.get(position), 0f, 0f )));
       }
       System.out.println("drone position: " + droneList.get(0).getPosition());

       //now that all the drones are created filter for drones touching the ground and not
        List<Drone> flyingDrones = droneList.stream()
                .filter(drone -> !drone.getPhysXEngine().chassisTouchesGround(drone.getOrientation(), drone.getPosition()))
                .collect(Collectors.toList());
        //take all the drones that touch the ground
        List<Drone> dronesOnGround = droneList.stream().
                filter(drone -> drone.getPhysXEngine().chassisTouchesGround(drone.getOrientation(), drone.getPosition()))
                .collect(Collectors.toList());

        System.out.println("ground drones: " + dronesOnGround);
        System.out.println("flying drones: " + flyingDrones);

        configureGroundDrones(dronesOnGround);
        ConfigureAirborneDrones(flyingDrones);

        return droneList; //the references are still intact (ptr) and the drones are now in the correct state
    }

    /**
     * The drones that are flying are configured for  steady flight
     * @param flyingDrones the list of the flying drones
     */
    private void ConfigureAirborneDrones(List<Drone> flyingDrones) {
        //the drones that are located in the air need to be balanced out

        for(Drone drone: flyingDrones){
            drone.setAutopilotOutputs(getInitControlState());
            if(this.getFlyingDroneAbsVel() == 0f){
                //if the state is not yet determined, start the balance function
                this.setFlyingDroneAbsVel(this.getStableFlyingVelocity(drone));
            }
            //use the found absolute velocity to calculate the true velocity of the drone
            //this is equal to the -z vector in the drone axis system
            Vector stableVelocity = PhysXEngine.droneOnWorld(new Vector(0, 0, -this.getFlyingDroneAbsVel()), drone.getOrientation());
            drone.setVelocity(stableVelocity);
        }
    }

    /**
     * The drones that are standing on the drones are configured for their place on the tarmac
     * @param dronesOnGround the drones located on the ground
     */
    private void configureGroundDrones(List<Drone> dronesOnGround) {
        //the drones on the ground need to set a proper tyre delta
        for(Drone drone: dronesOnGround){
            drone.getPhysXEngine().getDroneChassis().setDroneTyreDelta(drone);
        }
    }

    private Drone generateDrone(Vector position, Vector orientation){
        //generate the drone state (until now only the position is determined
        DroneState droneState = new DroneState() {
            @Override
            public Vector getPosition() {
                return position;
            }

            @Override
            public Vector getVelocity() {
                return new Vector();
            }

            @Override
            public Vector getOrientation() {
                return orientation;
            }

            @Override
            public Vector getRotation() {
                return new Vector();
            }

            @Override
            public float getPrevFrontTyreDelta() {
                return 0;
            }

            @Override
            public float getPrevRearLeftTyreDelta() {
                return 0;
            }

            @Override
            public float getPrevRearRightTyreDelta() {
                return 0;
            }
        };
        //configure the drone
        return new Drone(droneState, this.createConfig());
    }

    /**
     * Calculates the velocity needed for a steady flight
     * @param drone the drone wherefore the steady state must be reached
     * @return the size of the velocity vector needed for stable flight
     */
    private float getStableFlyingVelocity(Drone drone){
        //save the orientation
        Vector trueOrientation = drone.getOrientation();
        //set the drone straight
        drone.setOrientation(new Vector());
        //generate the optimisations
        PhysXEngine.PhysXOptimisations optimisations = drone.getPhysXEngine().createPhysXOptimisations();
        //calculate the balance
        Vector velocity = (optimisations.balanceDrone(drone.getOrientation(), MAIN_STABLE_INCLINATION, HOR_STABLE_INCLINATION))[1];

        drone.setOrientation(trueOrientation);

        return velocity.getSize();
    }



    /**
     * generates a configuration for the drone to be created, assigns an unique drone id for communication
     * Aside from the ID, all the drones are the same
     */
    private AutopilotConfig createConfig(){
        return new AutopilotConfig() {
            @Override
            public String getDroneID() {
                return DroneBuilder_v2.this.generateDoneID();
            }

            @Override
            public float getGravity() {
                return DroneBuilder_v2.GRAVITY;
            }

            @Override
            public float getWingX() {
                return DroneBuilder_v2.MAIN_WING_X_POS;
            }

            @Override
            public float getTailSize() {
                return DroneBuilder_v2.STABILIZER_POSITION;
            }

            @Override
            public float getWheelY() {
                return DroneBuilder_v2.WHEEL_Y_POS;
            }

            @Override
            public float getFrontWheelZ() {
                return DroneBuilder_v2.FRONT_WHEEL_Z_POS;
            }

            @Override
            public float getRearWheelZ() {
                return DroneBuilder_v2.REAR_WHEEL_Z_POS;
            }

            @Override
            public float getRearWheelX() {
                return DroneBuilder_v2.REAR_WHEEL_X_POS;
            }

            @Override
            public float getTyreSlope() {
                return DroneBuilder_v2.TYRE_SLOPE;
            }

            @Override
            public float getDampSlope() {
                return DroneBuilder_v2.DAMP_SLOPE;
            }

            @Override
            public float getTyreRadius() {
                return DroneBuilder_v2.TYRE_RADIUS;
            }

            @Override
            public float getRMax() {
                return DroneBuilder_v2.MAX_BRAKE_FORCE;
            }

            @Override
            public float getFcMax() {
                return DroneBuilder_v2.MAX_FRICTION_COEFF;
            }

            @Override
            public float getEngineMass() {
                return DroneBuilder_v2.ENGINE_MASS;
            }

            @Override
            public float getWingMass() {
                return DroneBuilder_v2.MAIN_WING_MASS;
            }

            @Override
            public float getTailMass() {
                return DroneBuilder_v2.STABILIZER_MASS;
            }

            @Override
            public float getMaxThrust() {
                return DroneBuilder_v2.MAX_THRUST;
            }

            @Override
            public float getMaxAOA() {
                return DroneBuilder_v2.MAX_AOA;
            }

            @Override
            public float getWingLiftSlope() {
                return DroneBuilder_v2.MAIN_LIFT_SLOPE;
            }

            @Override
            public float getHorStabLiftSlope() {
                return DroneBuilder_v2.HORIZONTAL_STABILIZER_LIFT_SLOPE;
            }

            @Override
            public float getVerStabLiftSlope() {
                return DroneBuilder_v2.VERTICAL_STABILIZER_LIFT_SLOPE;
            }

            @Override
            public float getHorizontalAngleOfView() {
                return DroneBuilder_v2.HORIZONTAL_ANGLE_OF_VIEW_CAMERA;
            }

            @Override
            public float getVerticalAngleOfView() {
                return DroneBuilder_v2.VERTICAL_ANGLE_OF_VIEW_CAMERA;
            }

            @Override
            public int getNbColumns() {
                return DroneBuilder_v2.NUMBER_OF_COLUMN_PIXELS_CAMERA;
            }

            @Override
            public int getNbRows() {
                return DroneBuilder_v2.NUMBER_OF_ROW_PIXELS_CAMERA;
            }
        };
    }


    /**
     * Generates an unique drone ID for every created Drone
     * @return a unique drone identifier
     */
    private String generateDoneID(){
        return Integer.toString(getNextDroneID());
    }

    /**
     * Gets the next drone ID to be assigned (starting from 0)
     * @return a drone ID to assign to a newly created drone
     */
    private int getNextDroneID() {
        int currentDroneID = this.nextDroneID;
        this.nextDroneID = currentDroneID + 1;
        return currentDroneID;
    }

    /**
     * Checks if the provided ID is already assigned to a drone
     * @param ID the ID to check
     * @return true if and only if the ID is already assigned to a drone, false otherwise
     */
    public boolean alreadyAssignedID(int ID){
        return ID < this.nextDroneID;
    }

    public float getFlyingDroneAbsVel() {
        return flyingDroneAbsVel;
    }

    public void setFlyingDroneAbsVel(float flyingDroneAbsVel) {
        this.flyingDroneAbsVel = flyingDroneAbsVel;
    }

    /**
     * Set the initial control state for the drone such that there are no issues for flying
     * @return AutopilotInterfaces outputs for steady flight (only main wings are used)
     */
    private static AutopilotOutputs getInitControlState(){
        return new AutopilotOutputs(){
            @Override
            public float getThrust() {
                return 0;
            }

            @Override
            public float getLeftWingInclination() {
                return MAIN_STABLE_INCLINATION;
            }

            @Override
            public float getRightWingInclination() {
                return MAIN_STABLE_INCLINATION;
            }

            @Override
            public float getHorStabInclination() {
                return HOR_STABLE_INCLINATION;
            }

            @Override
            public float getVerStabInclination() {
                return VER_STABLE_INCLINATION;
            }

            @Override
            public float getFrontBrakeForce() {
                return FRONT_BREAK_FORCE;
            }

            @Override
            public float getLeftBrakeForce() {
                return LEFT_BREAK_FORCE;
            }

            @Override
            public float getRightBrakeForce() {
                return RIGHT_BREAK_FORCE;
            }
        };
    }

    /*
    Instance Variables
     */

    /**
     * Variable that holds the next drone ID to be assigned to a newly created drone
     */
    private int nextDroneID = 0;

    /**
     * Variable to store the balanced state of a drone that flies in the air
     * prevents re calculation of the balance point since all drones are created with the same physical properties
     */
    private float flyingDroneAbsVel = 0f;

    /*
    Constants and messages used for external communication
     */
    private final static float INIT_COMPRESSION = 0.05f;

    //Todo find good values for the tyre and lift slope
    private final static float GRAVITY = 9.81f;
    private final static float MAIN_WING_X_POS = 4.2f;
    private final static float STABILIZER_POSITION = 4.2f;
    private final static float WHEEL_Y_POS = 1f;
    private final static float FRONT_WHEEL_Z_POS = 1f;
    private final static float REAR_WHEEL_Z_POS = 0.5f;
    private final static float REAR_WHEEL_X_POS = 0.5f;
    private final static float TYRE_RADIUS = 0.20f;
    private final static float MAX_BRAKE_FORCE = 1000f;
    private final static float MAX_FRICTION_COEFF = 0.9f;
    private final static float ENGINE_MASS = 180f;
    private final static float MAIN_WING_MASS = 100f;
    private final static float STABILIZER_MASS = 100f;
    private final static float MAX_THRUST = 2000f;
    private final static float MAX_AOA = (float) (PI/12f);
    private final static float MAIN_LIFT_SLOPE = 10f;
    private final static float HORIZONTAL_STABILIZER_LIFT_SLOPE = 5f;
    private final static float VERTICAL_STABILIZER_LIFT_SLOPE = 5f;
    private final static float HORIZONTAL_ANGLE_OF_VIEW_CAMERA = (float) (120f*PI/180);
    private final static float VERTICAL_ANGLE_OF_VIEW_CAMERA = (float) (120f*PI/180);
    private final static int   NUMBER_OF_COLUMN_PIXELS_CAMERA = 200;
    private final static int   NUMBER_OF_ROW_PIXELS_CAMERA = 200;
    private final static float TYRE_SLOPE = (ENGINE_MASS+STABILIZER_MASS+MAIN_WING_MASS*2)*GRAVITY * 1/3f *1/INIT_COMPRESSION; // the slope of the tyre
    private final static float DAMP_SLOPE = 4000f;//4000f;

    private final static float MAIN_STABLE_INCLINATION = (float) (5f*PI/180f);
    private final static float HOR_STABLE_INCLINATION = 0f;
    private final static float VER_STABLE_INCLINATION = 0f;
    private final static float STABLE_COMPRESSION = 0.05f;
    private final static float STABLE_Y_POS= TYRE_RADIUS - STABLE_COMPRESSION + WHEEL_Y_POS;
    public final static float START_Y = WHEEL_Y_POS + TYRE_RADIUS;

    private final static float FRONT_BREAK_FORCE = 0f;
    private final static float LEFT_BREAK_FORCE = 0f;
    private final static float RIGHT_BREAK_FORCE = 0f;



}
