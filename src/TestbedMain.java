import AutopilotInterfaces.*;
import gui.*;
import internal.Autopilot.Controller;
import internal.Exceptions.AngleOfAttackException;
import internal.Exceptions.SimulationEndedException;
import internal.Helper.Vector;
import internal.Testbed.*;
import math.Vector3f;
import org.lwjgl.glfw.GLFWVidMode;


import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutionException;

import static org.lwjgl.glfw.GLFW.glfwGetPrimaryMonitor;
import static org.lwjgl.glfw.GLFW.glfwGetVideoMode;
//TODO implement in such a way that the testbed can support multiple drones (and thus multiple autopilots)
//TODO also be able to handle crashes and correctly close the autopilots that are connected via the testbed (close the socket?)
//note: can we split the rendering stage in two separate operations? rendering the camera image for the autopilot
//and while the AP is processing, rendering the other on screen images
/**
 * Created by Martijn on 13/11/2017.
 * supervised by Bart
 */
public class TestbedMain implements Runnable{

    /**
     * Constructor for a testbed main loop, special configurations (eg for demo)
     * @param connectionName the name of the connection (here localhost)
     * @param connectionPort the port where the connection goes throught
     * @param showAllWidows flag to show all the GUI windows or just the primal one
     * @param controllerMode the controller mode (Alpha or Beta mode, specified in the GUI)
     * @param predefMode specifies if the testbed runs a predefined world or not
     * @param predefWorldDirect specifies the directory of the txt file that contains the configuration of the world
     */
    public TestbedMain(String connectionName, int connectionPort, boolean showAllWidows, String controllerMode, boolean predefMode, String predefWorldDirect) {
        //used to configure the testbed
        this.setConnectionName(connectionName);
        this.setConnectionPort(connectionPort);
        this.showAllWindows = showAllWidows;
        this.setControllerMode(controllerMode);
        this.setPredefMode(predefMode);
        this.setPredefWorldDirect(predefWorldDirect);
    }

    /**
     * Constructor for the testbed main, sets the configuration needed for a standard flight
     * @param connectionName the name of the connection mainly for the
     * @param connectionPort the port which trough to connect
     * @param showAllWindows show all the GUI windows or just a single one
     * @param controllerMode the mode for the controller (alpha or beta, use the PhysX defined modes)
     */
    public TestbedMain(String connectionName, int connectionPort, boolean showAllWindows, String controllerMode){
        this(connectionName, connectionPort, showAllWindows, controllerMode, false, "");
    }

    /**
     * Run method used for multiThreading purposes
     */
    @Override
    public void run() {
        try {
            this.testbedMainLoop();
        } catch (IOException | InterruptedException | ExecutionException e) {
            e.printStackTrace();
        }
    }

    /**
     * Initialize the testbed
     * Testbed is not initialized in the constructor because graphics need to be created
     * within one thread of control
     */
    private void initTestbed() throws IOException {

        // first generate the graphics
        this.generateGraphics();

        // then initialize the world
        this.initWorld();

        // the settings of the world influence the way the windows are rendered so
        // we may only initialize them after the world config is known
        this.initWindows();

        // set the timer for real time sync
        Time.initTime();
    }


    //Todo configure the autopilot in the autopilot main
    //open separate threads for both main loops, where the first one waits for the other one.
    public void testbedMainLoop() throws IOException, InterruptedException, ExecutionException {

        // first initialize the testbed itself
        this.initTestbed();
        // then initialize the connectivity of the testbed
        this.initTestbedServer();

        //write the configuration to the autopilot
        configAutopilot(this.getOutputStream());

        while(true) {
            //wait until the input is not null and advance if possible
            try {
                //set the timer for real frame rate
                Time.update();

                AutopilotOutputs output = AutopilotOutputsReader.read(this.getInputStream());
                AutopilotInputs_v2 autopilotInputs = this.testbedCycle(output);
                AutopilotInputs_v2Writer.write(this.getOutputStream(), autopilotInputs);
                //wait until frame is passed
                framerateControl();

            } catch(java.io.EOFException | SimulationEndedException ex ){
                //this exception means that the client socket has closed, close own sockets
                System.out.println("Closing down Testbed Server");
                terminateTestbedServer();
                break;

            } catch(AngleOfAttackException ex){
                //angle of attack exception has occurred close down the server
                System.out.println("Closing down Testbed Server");
                terminateTestbedServer();
                throw new AngleOfAttackException(ex.getCauseWing());
            }
        }
    }

    /**
     * Call this method to properly initialize the testbed server size
     * @throws IOException
     */
    private void initTestbedServer() throws IOException {
        // first setup the server
        ServerSocket testbedServer = new ServerSocket(this.getConnectionPort());
        // listen for connection
        Socket testbedClientSocket = testbedServer.accept();
        // initialize the streams
        DataInputStream inputStream = new DataInputStream(testbedClientSocket.getInputStream());
        DataOutputStream outputStream = new DataOutputStream(testbedClientSocket.getOutputStream());

        // set the sockets
        this.setTestbedServerSocket(testbedServer);
        this.setTestbedClientSocket(testbedClientSocket);

        // set the streams
        this.setInputStream(inputStream);
        this.setOutputStream(outputStream);

    }

    /**
     * Call this termination method to properly close down the server side of the system
     * @throws IOException
     */
    private void terminateTestbedServer() throws IOException {
        this.getInputStream().close();
        this.getOutputStream().close();
        this.getTestbedClientSocket().close();
        this.getTestbedServerSocket().close();
    }


    /**
     * Write the configuration data to the autopilot: first write the config, then the path and finally the inputs for
     * the first step
     * @param outputStream the stream containing the config and the input for the autopilot
     * @throws IOException
     * @throws InterruptedException
     */
    private void configAutopilot(DataOutputStream outputStream) throws IOException, InterruptedException {
        AutopilotConfigWriter.write(outputStream, this.getConfig());
        World world = this.getWorld();
        AutopilotInterfaces.Path path = world.getApproxPath();
        try{
            PathWriter.write(outputStream, path);
        //if we have a null pointer, this means that the world builder that we use does not have a path
        //todo uncomment if fixed
        }catch(NullPointerException e){
            PathWriter.write(outputStream, new Path() {
                @Override
                public float[] getX() {
                    return new float[0];
                }

                @Override
                public float[] getY() {
                    return new float[0];
                }

                @Override
                public float[] getZ() {
                    return new float[0];
                }
            });
        }

        //instert a null pointer, will be ignored on the first step of the testbed step method
        AutopilotInputs_v2 autopilotInputs = this.firstCycle();
        AutopilotInputs_v2Writer.write(outputStream, autopilotInputs);
    }

    /**
     * Handles the simulation output for the first simulation cycle (no time update, just render the image
     * @return the autopilot inputs for the first Cycle
     * @throws IOException
     */
    private AutopilotInputs_v2 firstCycle() throws IOException {
        byte[] image = this.generateImage();
        this.updateSimulationTime();
        return new MainAutopilotInputs(this.getDrone(), image, this.getSimulationTime());
    }


    /**
     * Simulates one step of the testbed
     *
     * @param autopilotOutputs the outputs of the autopilot
     * @return the inputs for the next step of the autopilot
     * @throws InterruptedException
     * @throws IOException
     */
    public AutopilotInputs_v2 testbedCycle(AutopilotOutputs autopilotOutputs) throws InterruptedException, IOException, ExecutionException {
        AutopilotInputs_v2 autopilotInputs;
        // update time


        //pass the outputs to the drone
        //try if the world can be advanced to the next state
        // it is the first run, just skip the output (frame is not yet rendered)
        this.getDrone().setAutopilotOutputs(autopilotOutputs);
        //System.out.println("all drones in world: " + this.getWorld().getDroneSet());
//        long startTime = System.currentTimeMillis();
        this.getWorld().advanceWorldState(TIME_STEP, STEPS_PER_CYCLE);
//        long deltaCalcTime = System.currentTimeMillis()-startTime;
//        System.out.println("physics time: " + deltaCalcTime);
        //update the simulation time for the outputs (elapsed time)
        updateSimulationTime();
        //generate the image for the autopilot
        byte[] cameraImage = generateImage();

        //create the output object
        autopilotInputs = new MainAutopilotInputs(this.getDrone(), cameraImage, this.getSimulationTime());
        //System.out.println("Dronepos: " + this.getDrone().getPosition() + "; elapsed_time: " + this.getSimulationTime());
        return autopilotInputs;
    }


    /**
     * generates an image represented by a 200x200x3 1D byte array (the 3 represents the RGB value)
     * based on the current camera view
     * @return an array containing the current camera input
     * @throws IOException
     */
    private byte[] generateImage() throws IOException {
        this.getGraphics().renderWindows();
        return droneCam.getCameraView();
    }

    /**
     * Updates the simulation time with the time added needed for one cycle in the simulation
     */
    private void updateSimulationTime() {
        float prevTime = this.getSimulationTime();
        float newTime = prevTime + TIME_STEP * STEPS_PER_CYCLE;
        this.setSimulationTime(newTime);
    }

    /**
     * Lets the thread sleep for as long as needed to give true frame rate
     * @throws InterruptedException
     */
    private void framerateControl() throws InterruptedException {
        long timeLeft = (long) (FRAME_MILLIS - Time.timeSinceLastUpdate());
        if (timeLeft > 0)
            Thread.sleep(timeLeft);
    }

    /**
     * Initializes the world according to the parameters set in the constructor
     * @throws IOException just java things
     */
    private void initWorld() throws IOException {
        WorldBuilder_v2 builder = new WorldBuilder_v2();
//        Map<Vector, Float> droneConfig = new HashMap<>();
//        droneConfig.put(new Vector(0, 10f,0), 0f); //drone standing on ground with tyre compression 0.05
        //World world =  builder.createWorld();
        //World world = builder.createWorld(droneConfig);
        //World world = builder.createFlightTestWorld();
        //World world = builder.createTakeoffWorld();
        //World world = builder.createTotalFlightWorld();
        World world = builder.createTotalFlightWorld();
        //World world  = builder.createLandingWorld();
        this.setWorld(world);
        Drone drone =(Drone)(world.getDroneSet().toArray())[0]; //only one drone is present
        //System.out.println("drone: " + drone);
        this.setDrone(drone);
        drone.addFlightRecorder(this.getFlightRecorder());

//        // drone builder covers all the stuff involving building the drone, adjust parameters there
//        WorldBuilder worldBuilder = new WorldBuilder();
//
//        // if the simulation is run in predefined mode
//        // generate the predefined world instead of the random one
//        if(this.isPredefMode()){
//            worldBuilder.setPredefWorld(true);
//            worldBuilder.setPredefDirectory(this.getPredefWorldDirect());
//        }
//
//        // configure the world note that the drone object is part of the world instance (and not a static as before)
//        this.setWorld(worldBuilder.createWorld(this.getControllerMode()));
//        this.setDrone(worldBuilder.getDrone());
//        // only in use for diagnostics
//        this.getDrone().addFlightRecorder(this.getFlightRecorder());
    }

    /**
     * Generates the graphics of the drone and adds all the windows to the graphics object
     */
    private void generateGraphics(){
        // initialize graphics capabilities
        this.setGraphics(new Graphics());

        // GraphicsObjects needs graphics to be able to initialize cubes
        Cube.setGraphics(this.getGraphics());
        Tile.setGraphics(this.getGraphics());
		Wheel.setGraphics(this.getGraphics());

        this.setDroneCam(new Window(200, 200, 0.5f, 0.4f, "bytestream window", new Vector3f(1.0f, 1.0f, 1.0f), false));

        GLFWVidMode vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());
        int monitorWidth = vidmode.width();
        int monitorHeight = vidmode.height();
        // if we only want to show part of the windows, this flag is set in the main loop
        if(this.getShowAllWindows()) {
            this.setDroneView(new Window(monitorWidth/2, monitorHeight/2 - 30, 0.0f, 0.05f, "Drone view", new Vector3f(1.0f, 1.0f, 1.0f), true));
            this.setTopDownView(new Window(monitorWidth/2, monitorHeight/3 - 30, 1f, 0.04f, "Top down view", new Vector3f(1.0f, 1.0f, 1.0f), true));
            this.setSideView(new Window(monitorWidth/2, monitorHeight/3 - 30, 1f, 0.52f, "Side view", new Vector3f(1.0f, 1.0f, 1.0f), true));
            this.setChaseView(new Window(monitorWidth/2, monitorHeight/2 - 30, 0f, 1f, "Chase view", new Vector3f(1.0f, 1.0f, 1.0f), true));
        } else {
            this.setDroneView(new Window(monitorWidth, monitorHeight, 1.f, 1.f, "Drone view", new Vector3f(1.0f, 1.0f, 1.0f), true));

        }


        // add the windows to graphics
        this.getGraphics().addWindow("bytestream window", this.getDroneCam());
        this.getGraphics().addWindow("Drone view", this.getDroneView());
        //only needed for demo
        if(this.getShowAllWindows()) {
            this.getGraphics().addWindow("Top down view", this.getTopDownView());
            this.getGraphics().addWindow("Side view", this.getSideView());
            this.getGraphics().addWindow("Chase view", this.getChaseView());
        }
    }

    /**
     * Initialize the windows used in the simulation
     */
    private void initWindows(){
    	this.getGraphics().setWorld(getWorld());
        // Initialize the windows
        this.getDroneCam().initWindow(Settings.DRONE_CAM);
        this.getDroneView().initWindow(Settings.DRONE_CAM);

        if(this.getShowAllWindows()) {
            this.getTopDownView().initWindow(Settings.DRONE_TOP_DOWN_CAM);
            this.getChaseView().initWindow(Settings.DRONE_CHASE_CAM);
            this.getSideView().initWindow(Settings.DRONE_SIDE_CAM);
        }

        GLFWVidMode vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());
        int monitorWidth = vidmode.width();
        int monitorHeight = vidmode.height();
        this.getGraphics().makeTextWindow("Stats", monitorWidth/2, monitorHeight/3, monitorWidth/2, monitorHeight*2/3);
        // create the switch when in single window mode
        if (!this.getShowAllWindows()) {

//            this.getGraphics().makeButtonWindow();
        }
    }



    /**
     * Class that contains the autopilotInputs implemented separately for cleaner code
     */
    private static class MainAutopilotInputs implements AutopilotInputs_v2 {

        /**
         * Constructor for the class
         * @param drone the drone of the testbed
         * @param cameraImage the byte array containing the image for the autopilot
         * @param elapsedTime the time that has elapsed
         */
        private MainAutopilotInputs(Drone drone, byte[] cameraImage, float elapsedTime) {
            this.drone = drone;
            this.cameraImage = cameraImage;
            this.elapsedTime = elapsedTime;
        }

        @Override
        public byte[] getImage() {
            return cameraImage;
        }

        @Override
        public float getX() {
            return drone.getPosition().getxValue();
        }

        @Override
        public float getY() {
            return drone.getPosition().getyValue();
        }

        @Override
        public float getZ() {
            return drone.getPosition().getzValue();
        }

        @Override
        public float getHeading() {
            return drone.getHeading();
        }

        @Override
        public float getPitch() {
            return drone.getPitch();
        }

        @Override
        public float getRoll() {
            return drone.getRoll();
        }

        @Override
        public float getElapsedTime() {
            return elapsedTime;
        }

        private Drone drone;

        private byte[] cameraImage;

        private float elapsedTime;
    }


    private World getWorld() {
        return world;
    }

    private void setWorld(World world) {
        this.world = world;
    }

    public AutopilotConfig getConfig() {
        return drone.getAutopilotConfig();
    }

    private Graphics getGraphics() {
        return graphics;
    }

    private void setGraphics(Graphics graphics) {
        this.graphics = graphics;
    }

    private Drone getDrone() {
        return drone;
    }

    private void setDrone(Drone drone) {
        this.drone = drone;
    }

    private String getConnectionName() {
        return connectionName;
    }

    private void setConnectionName(String connectionName) {
        this.connectionName = connectionName;
    }

    private int getConnectionPort() {
        return connectionPort;
    }

    private void setConnectionPort(int connectionPort) {
        this.connectionPort = connectionPort;
    }

    private Window getDroneCam() {
        return droneCam;
    }

    private void setDroneCam(Window droneCam) {
        this.droneCam = droneCam;
    }

    private Window getDroneView() {
        return droneView;
    }

    private void setDroneView(Window droneView) {
        this.droneView = droneView;
    }

    private Window getTopDownView() {
        return topDownView;
    }

    private void setTopDownView(Window topDownView) {
        this.topDownView = topDownView;
    }

    private Window getChaseView() {
        return chaseView;
    }

    private void setChaseView(Window chaseView) {
        this.chaseView = chaseView;
    }

    private Window getSideView() {
        return sideView;
    }

    private void setSideView(Window sideView) {
        this.sideView = sideView;
    }

    private boolean getShowAllWindows() {
        return showAllWindows;
    }

    private FlightRecorder getFlightRecorder() {
        return flightRecorder;
    }

    public void setFlightRecorder(FlightRecorder flightRecorder) {
        if(this.getFlightRecorder() != null)
            throw new IllegalStateException(ALREADY_RECORDING);
        this.flightRecorder = flightRecorder;
    }

    private String getControllerMode() {
        return controllerMode;
    }

    private void setControllerMode(String controllerMode) {
        this.controllerMode = controllerMode;
    }

    private boolean isPredefMode() {
        return predefMode;
    }

    private void setPredefMode(boolean predefMode) {
        this.predefMode = predefMode;
    }

    private String getPredefWorldDirect() {
        return predefWorldDirect;
    }

    private void setPredefWorldDirect(String predefWorldDirect) {
        this.predefWorldDirect = predefWorldDirect;
    }

    private float getSimulationTime() {
        return simulationTime;
    }

    private void setSimulationTime(float simulationTime) {
        this.simulationTime = simulationTime;
    }

    private ServerSocket getTestbedServerSocket() {
        return testbedServerSocket;
    }

    private void setTestbedServerSocket(ServerSocket testbedServerSocket) {
        this.testbedServerSocket = testbedServerSocket;
    }

    private Socket getTestbedClientSocket() {
        return testbedClientSocket;
    }

    private void setTestbedClientSocket(Socket testbedClientSocket) {
        this.testbedClientSocket = testbedClientSocket;
    }

    private DataInputStream getInputStream() {
        return inputStream;
    }

    private void setInputStream(DataInputStream inputStream) {
        this.inputStream = inputStream;
    }

    private DataOutputStream getOutputStream() {
        return outputStream;
    }

    private void setOutputStream(DataOutputStream outputStream) {
        this.outputStream = outputStream;
    }

    private World world;
    private Graphics graphics;
    private Drone drone;

    private Window droneCam;
    private Window droneView;
    private Window topDownView;
    private Window chaseView;
    private Window sideView;

    private String connectionName;
    private int connectionPort;
    private boolean showAllWindows;
    private FlightRecorder flightRecorder;
    private String controllerMode;
    private boolean predefMode;
    private String predefWorldDirect;

    private float simulationTime = 0.0f;
    private ServerSocket testbedServerSocket;
    private Socket testbedClientSocket;
    private DataInputStream inputStream;
    private DataOutputStream outputStream;


    // configuration for 20 fps
    private final static float TIME_STEP = 0.001f;
    private final static float FRAMERATE = 20.0f;
    private final static int STEPS_PER_CYCLE = Math.round((1 / FRAMERATE) / TIME_STEP);
    private final static long FRAME_MILLIS = 50;

    /*
    Error Messages
     */
    private final static String ALREADY_RECORDING = "there is already a flight recorder recording";

}
