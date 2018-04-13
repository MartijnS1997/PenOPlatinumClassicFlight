package internal.Testbed;


import AutopilotInterfaces.AutopilotConfig;
import internal.Autopilot.AutoPilot;
import internal.Physics.HorizontalWingPhysX;
import internal.Physics.PhysXEngine;
import internal.Helper.Vector;
import internal.Physics.VerticalWingPhysX;

import static java.lang.Math.PI;

/**
 * Created by Martijn on 23/10/2017.
 * @author Martijn Sauwens
 */
public class DroneBuilder {

    /**
     * Constants used to create the drone & configure the autopilot
     */

    public final static float HORIZONTALVIEW = (float) (120*PI/180);
    public final static float VERTICALVIEW = (float) (120*PI/180);
    public final static int NB_COLS = 200;
    public final static int NB_ROWS = 200;

    public final static float  BETA_ENGINE_MASS = .250f;
    public final static float  BETA_MAX_THRUST = 5.0f;
    public final static float  BETA_MAIN_WING_MASS = .25f;
    public final static float  BETA_STABILIZER_MASS = 0.125f;
    public final static float  BETA_MAINWING_START_INCL = (float) PI/12.0f;
    public final static float  BETA_STABS_START_INCL = 0.0f;
    public final static float  BETA_MAX_ANGLE_OF_ATTACK = (float) ( PI/2.0 - 0.001f);
    public final static float  BETA_LIFT_COEFFICIENT = .3f;
    public final static float  BETA_LIFT_COEFFICIENT_STAB =.15f;
    public final static Vector BETA_LEFTWING_POS = new Vector(-.5f, 0.0f, 0.0f);
    public final static Vector BETA_RIGHTWING_POS = new Vector(.5f, 0.0f, 0.0f);
    public final static Vector BETA_STABILIZE_POS = new Vector(0.0f, 0.0f, .5f);
    public final static Vector BETA_STARTPOS = new Vector();
    public final static Vector BETA_START_VEL = new Vector(0,0,-6.32f);
    public final static Vector BETA_START_ORIENTATION = new Vector();
    public final static Vector BETA_START_ROTATION = new Vector();


    public final static float   ALPHA_ENGINE_MASS = 5.0f;
    public final static float   ALPHA_MAX_THRUST = 250.0f;
    public final static float   ALPHA_MAIN_WING_MASS = 2.5f;
    public final static float   ALPHA_STABILIZER_MASS = 1.25f*2f;
    public final static float   ALPHA_MAINWING_START_INCL = (float) PI/12.0f;
    public final static float   ALPHA_STABS_START_INCL = 0.0f;
    public final static float   ALPHA_MAX_ANGLE_OF_ATTACK = (float) ( PI/2.0 - 0.001f);
    public final static float   ALPHA_LIFT_COEFFICIENT = 5.0f;
    public final static float   ALPHA_LIFT_COEFFICIENT_STAB =1.0f;
    public final static Vector  ALPHA_LEFTWING_POS = new Vector(-1.0f, 0.0f, 0.0f);
    public final static Vector  ALPHA_RIGHTWING_POS = new Vector(1.0f, 0.0f, 0.0f);
    public final static Vector  ALPHA_STABILIZE_POS = new Vector(0.0f, 0.0f, 2.0f);
    public final static Vector  ALPHA_STARTPOS = new Vector(0,5f,0);
    public final static Vector  ALPHA_START_VEL = new Vector(0,0,-6.32f);
    public final static Vector  ALPHA_START_ORIENTATION = new Vector();
    public final static Vector  ALPHA_START_ROTATION = new Vector();

    public final static float    GAMMA_ENGINE_MASS = 180.0f;
    public final static float    GAMMA_MAX_THRUST = 2000.0f;
    public final static float    GAMMA_MAIN_WING_MASS = 100f;
    public final static float    GAMMA_STABILIZER_MASS = 50f;
    public final static float    GAMMA_MAINWING_START_INCL = (float)( 5*PI/180f);
    public final static float    GAMMA_STABS_START_INCL = 0.0f;
    public final static float    GAMMA_MAX_ANGLE_OF_ATTACK = (float) (PI/12.0);
    public final static float    GAMMA_LIFT_COEFFICIENT = 10.0f;
    public final static float    GAMMA_LIFT_COEFFICIENT_STAB =5.0f;
    public final static Vector   GAMMA_LEFTWING_POS = new Vector(-4.2f, 0.0f, 0.0f);
    public final static Vector   GAMMA_RIGHTWING_POS = new Vector(4.2f, 0.0f, 0.0f);
    public final static Vector   GAMMA_STABILIZE_POS = new Vector(0.0f, 0.0f, 4.2f);
    public final static Vector   GAMMA_STARTPOS = new Vector(0,5,0);
    public final static Vector   GAMMA_START_VEL = new Vector(0,0,0);
    public final static Vector   GAMMA_START_ORIENTATION = new Vector();
    public final static Vector   GAMMA_START_ROTATION = new Vector();

    public DroneBuilder(boolean balanced) {
        this.balanced = true;
    }


    public Drone createDrone(){
        return createDrone(ALPHA_CONFIG);
    }

    public Drone createDrone(String configMode) {
        HorizontalWingPhysX rightMain, leftMain, horizontalStabilizer;
        VerticalWingPhysX verticalStabilizer;
        Drone drone;

//        rightMain = new HorizontalWingPhysX(RIGHTWING_POS, LIFT_COEFFICIENT, MAIN_WING_MASS, MAX_ANGLE_OF_ATTACK, MAINWING_START_INCL);
//        leftMain = new HorizontalWingPhysX(LEFTWING_POS, LIFT_COEFFICIENT, MAIN_WING_MASS, MAX_ANGLE_OF_ATTACK, MAINWING_START_INCL);
//        horizontalStabilizer = new HorizontalWingPhysX(STABILIZE_POS, LIFT_COEFFICIENT_STAB, STABILIZER_MASS, MAX_ANGLE_OF_ATTACK, STABS_START_INCL);
//        verticalStabilizer = new VerticalWingPhysX(STABILIZE_POS, LIFT_COEFFICIENT_STAB, STABILIZER_MASS, MAX_ANGLE_OF_ATTACK, STABS_START_INCL);

        switch(configMode) {
            case PhysXEngine.ALPHA_MODE:
                System.out.println("Creating alpha drone");
                drone = new Drone(ALPHA_STARTPOS, ALPHA_START_VEL, ALPHA_START_ORIENTATION, ALPHA_START_ROTATION, createConfig(configMode));
                if(this.isBalanced()){
                    PhysXEngine.PhysXOptimisations optim = drone.getPhysXEngine().createPhysXOptimisations();
                    drone.setVelocity(optim.balanceDrone(drone.getOrientation(),ALPHA_MAINWING_START_INCL , 0.0f)[1]);
                }
                break;
            case PhysXEngine.BETA_MODE:
                drone = new Drone(BETA_STARTPOS, BETA_START_VEL, BETA_START_ORIENTATION, BETA_START_ROTATION, createConfig(configMode));
                if(this.isBalanced()){
                    PhysXEngine.PhysXOptimisations optim = drone.getPhysXEngine().createPhysXOptimisations();
                    drone.setVelocity(optim.balanceDrone(drone.getOrientation(), BETA_MAINWING_START_INCL, 0.0f)[1]);
                }
                break;
            case PhysXEngine.GAMMA_MODE:
                System.out.println("creating gamma drone");
                drone = new Drone(GAMMA_STARTPOS, GAMMA_START_VEL, GAMMA_START_ORIENTATION, GAMMA_START_ROTATION, createConfig(configMode));
                if(this.isBalanced()){
                    PhysXEngine.PhysXOptimisations optim = drone.getPhysXEngine().createPhysXOptimisations();
                    drone.setVelocity(optim.balanceDrone(drone.getOrientation(), GAMMA_MAINWING_START_INCL, 0.0f)[1]);
                }
                break;
            default:
                throw new IllegalArgumentException("INVALID MODE");
        }

        // if the drone needs to be balanced, do so (balancing is the act of setting the vertical force to 0
        // and the Z value to 0

        drone.setPosition(new Vector(0, 5f, 0));
        //System.out.println("Drone velocity: " + drone.getVelocity());
        //System.out.println("Drone thrust: " + drone.getThrust());

        return drone;
    }




    /**
     * Creates an un configured autopilot
     * @return an autopilot class object
     */
    public AutoPilot createAutoPilot(){
       return  new AutoPilot();
    }

    /**
     * Pseudo constructor for a configuration of an autopilotConfig
     * @return an AutopilotInterfaces config
     */
    public AutopilotConfig createConfig(String configMode) {
        switch (configMode) {
            case PhysXEngine.ALPHA_MODE:
                return new AutopilotConfig() {

                    @Override
                    public String getDroneID() {
                        return "0";
                    }

                    @Override
                    public float getGravity() {
                        return 9.81f;
                    }

                    @Override
                    public float getWingX() {
                        return ALPHA_RIGHTWING_POS.getxValue();
                    }

                    @Override
                    public float getTailSize() {
                        return ALPHA_STABILIZE_POS.getzValue();
                    }

                    @Override
                    public float getWheelY() {
                        return 0;
                    }

                    @Override
                    public float getFrontWheelZ() {
                        return 1f;
                    }

                    @Override
                    public float getRearWheelZ() {
                        return 1f;
                    }

                    @Override
                    public float getRearWheelX() {
                        return 1f;
                    }

                    @Override
                    public float getTyreSlope() {
                        return 0;
                    }

                    @Override
                    public float getDampSlope() {
                        return 0;
                    }

                    @Override
                    public float getTyreRadius() {
                        return 0;
                    }

                    @Override
                    public float getRMax() {
                        return 0;
                    }

                    @Override
                    public float getFcMax() {
                        return 0;
                    }

                    @Override
                    public float getEngineMass() {
                        return ALPHA_ENGINE_MASS;
                    }

                    @Override
                    public float getWingMass() {
                        return ALPHA_MAIN_WING_MASS;
                    }

                    @Override
                    public float getTailMass() {
                        return ALPHA_STABILIZER_MASS;
                    }

                    @Override
                    public float getMaxThrust() {
                        return ALPHA_MAX_THRUST;
                    }

                    @Override
                    public float getMaxAOA() {
                        return ALPHA_MAX_ANGLE_OF_ATTACK;
                    }

                    @Override
                    public float getWingLiftSlope() {
                        return ALPHA_LIFT_COEFFICIENT;
                    }

                    @Override
                    public float getHorStabLiftSlope() {
                        return ALPHA_LIFT_COEFFICIENT_STAB;
                    }

                    @Override
                    public float getVerStabLiftSlope() {
                        return ALPHA_LIFT_COEFFICIENT_STAB;
                    }

                    @Override
                    public float getHorizontalAngleOfView() {
                        return HORIZONTALVIEW;
                    }

                    @Override
                    public float getVerticalAngleOfView() {
                        return VERTICALVIEW;
                    }

                    @Override
                    public int getNbColumns() {
                        return NB_COLS;
                    }

                    @Override
                    public int getNbRows() {
                        return NB_ROWS;
                    }
                };

            case PhysXEngine.BETA_MODE:
                return new AutopilotConfig(){
                    @Override
                    public String getDroneID() {
                        return "0";
                    }

                    @Override
                    public float getGravity() {
                        return 9.81f;
                    }

                    @Override
                    public float getWingX() {
                        return  BETA_RIGHTWING_POS.getxValue();
                    }

                    @Override
                    public float getTailSize() {
                        return BETA_STABILIZE_POS.getzValue();
                    }

                    @Override
                    public float getWheelY() {
                        return 0;
                    }

                    @Override
                    public float getFrontWheelZ() {
                        return 0;
                    }

                    @Override
                    public float getRearWheelZ() {
                        return 0;
                    }

                    @Override
                    public float getRearWheelX() {
                        return 0;
                    }

                    @Override
                    public float getTyreSlope() {
                        return 0;
                    }

                    @Override
                    public float getDampSlope() {
                        return 0;
                    }

                    @Override
                    public float getTyreRadius() {
                        return 0;
                    }

                    @Override
                    public float getRMax() {
                        return 0;
                    }

                    @Override
                    public float getFcMax() {
                        return 0;
                    }

                    @Override
                    public float getEngineMass() {
                        return BETA_ENGINE_MASS;
                    }

                    @Override
                    public float getWingMass() {
                        return BETA_MAIN_WING_MASS;
                    }

                    @Override
                    public float getTailMass() {
                        return BETA_STABILIZER_MASS;
                    }

                    @Override
                    public float getMaxThrust() {
                        return BETA_MAX_THRUST;
                    }

                    @Override
                    public float getMaxAOA() {
                        return (float) (40*PI/180f);
                    }

                    @Override
                    public float getWingLiftSlope() {
                        return BETA_LIFT_COEFFICIENT;
                    }

                    @Override
                    public float getHorStabLiftSlope() {
                        return BETA_LIFT_COEFFICIENT_STAB;
                    }

                    @Override
                    public float getVerStabLiftSlope() {
                        return BETA_LIFT_COEFFICIENT_STAB;
                    }

                    @Override
                    public float getHorizontalAngleOfView() {
                        return HORIZONTALVIEW;
                    }

                    @Override
                    public float getVerticalAngleOfView() {
                        return VERTICALVIEW;
                    }

                    @Override
                    public int getNbColumns() {
                        return NB_COLS;
                    }

                    @Override
                    public int getNbRows() {
                        return NB_ROWS;
                    }
                };

            case PhysXEngine.GAMMA_MODE:
                return new AutopilotConfig() {

                    @Override
                    public String getDroneID() {
                        return "0";
                    }

                    @Override
                    public float getGravity() {
                        return 9.81f;
                    }

                    @Override
                    public float getWingX() {
                        return GAMMA_RIGHTWING_POS.getxValue();
                    }

                    @Override
                    public float getTailSize() {
                        return GAMMA_STABILIZE_POS.getzValue();
                    }

                    @Override
                    public float getWheelY() {
                        return 0;
                    }

                    @Override
                    public float getFrontWheelZ() {
                        return 1f;
                    }

                    @Override
                    public float getRearWheelZ() {
                        return 1f;
                    }

                    @Override
                    public float getRearWheelX() {
                        return 1f;
                    }

                    @Override
                    public float getTyreSlope() {
                        return 0;
                    }

                    @Override
                    public float getDampSlope() {
                        return 0;
                    }

                    @Override
                    public float getTyreRadius() {
                        return 0;
                    }

                    @Override
                    public float getRMax() {
                        return 0;
                    }

                    @Override
                    public float getFcMax() {
                        return 0;
                    }

                    @Override
                    public float getEngineMass() {
                        return GAMMA_ENGINE_MASS;
                    }

                    @Override
                    public float getWingMass() {
                        return GAMMA_MAIN_WING_MASS;
                    }

                    @Override
                    public float getTailMass() {
                        return GAMMA_STABILIZER_MASS;
                    }

                    @Override
                    public float getMaxThrust() {
                        return GAMMA_MAX_THRUST;
                    }

                    @Override
                    public float getMaxAOA() {
                        return GAMMA_MAX_ANGLE_OF_ATTACK;
                    }

                    @Override
                    public float getWingLiftSlope() {
                        return GAMMA_LIFT_COEFFICIENT;
                    }

                    @Override
                    public float getHorStabLiftSlope() {
                        return GAMMA_LIFT_COEFFICIENT_STAB;
                    }

                    @Override
                    public float getVerStabLiftSlope() {
                        return GAMMA_LIFT_COEFFICIENT_STAB;
                    }

                    @Override
                    public float getHorizontalAngleOfView() {
                        return HORIZONTALVIEW;
                    }

                    @Override
                    public float getVerticalAngleOfView() {
                        return VERTICALVIEW;
                    }

                    @Override
                    public int getNbColumns() {
                        return NB_COLS;
                    }

                    @Override
                    public int getNbRows() {
                        return NB_ROWS;
                    }
                };
            default:
                throw new IllegalArgumentException("wrong config mode");
        }

    }




    public boolean isBalanced() {
        return balanced;
    }

    /**
     * Varaible that stores if the drone needs to be balanced before flight
     */
    private boolean balanced;

    private final static String ALPHA_CONFIG = "ALPHA_CONFIG";
    private final static String BETA_CONFIG = "BETA_CONFIG";


}
/*
        float droneMass = 0.0f;
        float engineMass = 10.0f;
        float maxThrust = 100.0f;
        float mainWingMass = 5.0f;
        float stabilizerMass = 2.0f;
        float mainWingStartIncl = (float) (Math.PI / 12.0f);
        float stabInclination = 0.0f;
        float maxAngleOfAttack = (float) (Math.PI / 2.0f - 0.01f);
        float liftCoefficient = 1.0f;
        Vector leftWingPos = new Vector(-4.0f, 0.0f, 0.0f);
        Vector rightWingPos = new Vector(4.0f, 0.0f, 0.0f);
        Vector stabilizerPos = new Vector(0.0f, 0.0f, 8.0f);
        Vector startPos = new Vector(0.0f, 0.0f, 0.0f);
        Vector startVelocity = new Vector(0.0f, 0.0f, -20.490349f);
        Vector startOrientation = new Vector(0.0f, 0.0f, 0.0f);
        Vector startRotation = new Vector(0.0f, 0.0f, 0.0f);

        rightMain =new HorizontalWingPhysX(rightWingPos, liftCoefficient, mainWingMass, maxAngleOfAttack, mainWingStartIncl);

        leftMain =new HorizontalWingPhysX(leftWingPos, liftCoefficient, mainWingMass, maxAngleOfAttack, mainWingStartIncl);

        horizontalStabilizer =new HorizontalWingPhysX(stabilizerPos, liftCoefficient, stabilizerMass, maxAngleOfAttack,0.0f);

        verticalStabilizer =new VerticalWingPhysX(stabilizerPos, liftCoefficient, stabilizerMass, maxAngleOfAttack, stabInclination);

        drone =new Drone(droneMass, engineMass, maxThrust, startPos, startVelocity, startOrientation, startRotation, rightMain, leftMain, horizontalStabilizer, verticalStabilizer, null);

        return drone;
 */



