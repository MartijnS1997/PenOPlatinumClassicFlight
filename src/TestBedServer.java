import gui.Cube;
import gui.Graphics;
import gui.Settings;
import gui.Window;
import internal.Exceptions.AngleOfAttackException;
import internal.Exceptions.SimulationEndedException;
import internal.Helper.Vector;
import internal.Testbed.Drone;
import internal.Testbed.World;
import internal.Testbed.WorldBuilder_v2;
import math.Vector3f;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.stream.Collectors;

/**
 * Created by Martijn on 23/02/2018.
 * A class to host many requests for testbed connections by autopilot (acts as a server with multiple connections
 */

// Todo implement the following scheme for the server side:
//1. build a new world (blocks, airports etc, no drones)
//2. add the drones to the world via drone builder
//3. start the GUI
//4. start the server and listen for connections
//5. for every new connection create a new thread by assigning each thread a drone
//6. simulate the first step (config and state are sent via the threads)
//7. wait for the threads to send their data and pass the commands of the autopilot to their corresponding drones
//8. if the threads are finished setting the parameters for the next iteration start a new loop in the simulation
//note on threads: at the start of the connection all the Testbed threads are created and a thread pool to accompany them
//they are used for communication only and are re-used each execution cycle
//note: the server operates on port 4242 (TCP) on "localhost" (IP)

public class TestBedServer implements Runnable {


    public TestBedServer(float timeStep, int stepsPerCycle, int maxNbConnections, int maxNbThreads) {
        this.timeStep = timeStep;
        this.stepsPerCycle = stepsPerCycle;
        this.maxNbOfConnections = maxNbConnections;
        this.maxNbOfThreads = maxNbThreads;
    }

    //Todo initialize the server threads
    //Todo implement the incrementation of the simulation time

    /**
     * Run method, gets called upon thread creation for this class (own main loop)
     */
    @Override
    public void run() {
        //first initialize the testbed server
        this.initTestbedServer();
        //then start simulating the server
        this.serverMainLoop();
    }

    /**
     * The main loop of the server
     */
    private void serverMainLoop(){
        boolean simulationActive = true;
        while(simulationActive){
            try{
                //simulate the next step
                this.simulateStep();
            }catch(java.io.EOFException | SimulationEndedException e) {
                //the connection at the other side has closed or the simulation has ended
                //terminate the testbed
                this.terminateServer();
                simulationActive = false;

            }catch(AngleOfAttackException angleOffAttackException){
                //angle of attack exception occurred
                this.terminateServer();
                throw angleOffAttackException;

            }catch(IOException | InterruptedException e){
                //no clue what happened
                e.printStackTrace();
            }

        }
    }

    /*
    Methods for the simulation cycle
     */
    //Todo implement the simulation cycle
    private void simulateStep() throws IOException, InterruptedException {

        //first open up the connection and communicate the state to the autopilot
        //wait for a response
        autopilotCommunication();

        //now that all the communication is done, we can simulate the next step
        //(all the drones have received their commands for the next step)
        advanceWorld();

        //once the world is advanced, we are finished here
    }

    /**
     * Sends the current state to all the connections which in turn send the state to the connected autopilots
     * Post: the commands needed to simulate the next step are in place for each drone (the autopilotOutputs)
     * @throws InterruptedException i have no idea why (something with interrupted threads, but we dit not put timers in place)
     */
    private void autopilotCommunication() throws InterruptedException {
        //first send all the connections to the thread pool
        ExecutorService threadPool = this.getThreadPool();
        Set<TestbedConnection> connections = this.getServerTestbedConnections();
        //invoke all the connections to get a response from their connected autopilots
        List<Future<Void>> busyConnectionList = threadPool.invokeAll(connections);

        //now we wait for all the connections to finish
        boolean allConnectionsFinished = false;
        while(!allConnectionsFinished){
            //wait for the first element to finish
            busyConnectionList.get(0);
            //then filter for futures that are not yet complete (keep em)
            busyConnectionList = busyConnectionList.stream()
                    .filter(connection -> !connection.isDone())
                    .collect(Collectors.toList());
            //if all the elements are finished then all the communication is done
            if(busyConnectionList.size() == 0){
                allConnectionsFinished = true;
            }
        }
        // if all communication is done, return
    }

    /**
     * Advances the world state n times for a given time step t
     */
    private void advanceWorld() throws InterruptedException, IOException {
        World world = this.getWorld();
        float timeStep = this.getTimeStep();
        int stepsPerCycles = this.getStepsPerCycle();
        //advance the state of the world for n steps of time delta t
        world.advanceWorldState(timeStep, stepsPerCycles);
        //increment the time that was simulated
        this.incrementElapsedTime();
    }

    /**
     * Called for proper termination of the server (closing all sockets and streams etc)
     */
    private void terminateServer(){
        for(TestbedConnection connection: this.getServerTestbedConnections()){
            connection.terminateConnection();
        }
        try {
            this.getServerSocket().close();
        } catch (IOException e) {
            //doesn't matter if we're closing down
            e.printStackTrace();
        }
    }

    /*
    Initialization methods for the testbed server
     */

    /**
     * Initializes the testbed (see implementations and comments along the lines for further explanation
     */
    private void initTestbedServer() {
        // initialize the graphics engine
        this.initGraphics();

        // initialize the world that needs to be simulated
        this.initWorld();

        // initialize the windows
        this.initWindows();

        // initialize the server
        this.initServer();
    }


    //Todo implement the graphics initializer (need for adapted graphics)
    //Todo check if the initializer can be placed after the generation of the world (or why not)
    /**
     * Initializer for the graphics of the server
     * note: no drone cam is supported here, may be added in the future (not part of the assignment)
     */
    private void initGraphics(){
        //create a new graphics object to associate with the server
        this.setGraphics(new Graphics());

        //TODO check if cubes are still needed, with the packet delivery no cubes need to be placed to fly trough
        //provide the graphics for generating cubes
        Cube.setGraphics(this.getGraphics());

        //construct the windows
        this.setDroneView(new Window(960, 510, 0.0f, 0.05f, "Drone view", new Vector3f(1.0f, 1.0f, 1.0f), true));
        this.setTopDownView(new Window(960, 510, 1f, 0.05f, "Top down view", new Vector3f(1.0f, 1.0f, 1.0f), true));
        this.setSideView(new Window(960, 510, 1f, 1f, "Side view", new Vector3f(1.0f, 1.0f, 1.0f), true));
        this.setChaseView(new Window(960, 510, 0f, 1f, "Chase view", new Vector3f(1.0f, 1.0f, 1.0f), true));

        //then add the windows to the graphics engine
        this.getGraphics().addWindow("Drone view", this.getDroneView());
        this.getGraphics().addWindow("Top down view", this.getTopDownView());
        this.getGraphics().addWindow("Side view", this.getSideView());
        this.getGraphics().addWindow("Chase view", this.getChaseView());
    }

    /**
     * Initializer for the world that needs to be simulated by the server
     */
    // Todo implement the world initializer, first cleanup the world and drone builder
    // note: the amount of created drones is equal to the number of connections we can make
    private void initWorld(){
        Map<Vector, Float> droneSates = new HashMap<>();
        droneSates.put(new Vector(0,20,0), 0f); // a drone facing forward
        droneSates.put(new Vector(0, 30f, 20f), (float) Math.PI); // a drone facing backward
        WorldBuilder_v2 builder = new WorldBuilder_v2();
        World world = builder.createWorld(null, this.getThreadPool());
        this.setWorld(world);
    }

    /**
     * Initializer for the windows
     */
    private void initWindows(){
    	this.getGraphics().setWorld(getWorld());
        this.getDroneView().initWindow(Settings.DRONE_CAM);
        this.getTopDownView().initWindow(Settings.DRONE_TOP_DOWN_CAM);
        this.getChaseView().initWindow(Settings.DRONE_CHASE_CAM);
        this.getSideView().initWindow(Settings.DRONE_SIDE_CAM);
    }

    /**
     * Initializes the server and creates the objects needed for the connections
     */
    private void initServer(){
        //create the thread pool
        ExecutorService threadPool = Executors.newFixedThreadPool(this.getMaxNbOfThreads());
        //set the thread pool
        this.setThreadPool(threadPool);

        //create the server socket
        ServerSocket serverSocket = null;
        try {
            serverSocket = new ServerSocket(TestBedServer.getTcpPort());
        } catch (IOException e) {
            //the socket could not be created... I have no idea why, so lets print the issue
            e.printStackTrace();
        }
        //we need not to worry about null references, if server socket could not be created, the program terminates
        //abnormally
        this.setServerSocket(serverSocket);
        initConnections();

    }

    /**
     * Creates the connections needed for communication with the autopilots using the drones that
     * are contained within the world (adds the connections to the connection set)
     */
    private void initConnections() {
        //start listening for connections, creating connections until all the available connection spots are filled
        //get the world that is simulated and grab all the drones in a set
        World world = this.getWorld();
        Set<Drone> droneSet = world.getDroneSet();
        for(Drone drone: droneSet){
            try {
                //listening is a thread blocking method, just listen
                Socket socket = this.getServerSocket().accept();
                //if the socket is created hook a drone up to the connection
                TestbedConnection connection = new TestbedConnection(socket, drone, this);
                //add the connection to the connection set
                this.getServerTestbedConnections().add(connection);
            } catch (IOException e) {
                // again not a clue what went wrong, better print some stack trace
                e.printStackTrace();
            }
        }
        //if all the drones are assigned a connection we may exit
    }

    /*
    Getter and setters for physics stuff #############################
     */

    /**
     * Getter for the world simulated by the server, containing all
     * the drones and cubes
     * @return the world that is simulated
     */
    private World getWorld() {
        return world;
    }

    /**
     * Setter for the world that is simuated by the server, contains all the drones and cubes
     * @param world the world to simulate
     */
    private void setWorld(World world) {
        this.world = world;
    }

    /**
     * Getter for the in-simulation flight duration
     * @return the elapsed time
     */
    public float getElapsedTime() {
        return elapsedTime;
    }

    /**
     * Increments the in-simulation elapsed time with the given time step
     * elapsed time += getTimeStep()
     */
    private void incrementElapsedTime(){
        this.elapsedTime += this.getTimeStep()*this.getStepsPerCycle();
    }

    /**
     * Getter for the in-simulation time step
     * @return the time step
     */
    private float getTimeStep() {
        return timeStep;
    }

    /**
     * Getter for the number of steps that are taken each simulation cycle (no communication with the autopilot in between)
     * @return the nb op steps per simulation cycle
     */
    private int getStepsPerCycle() {
        return stepsPerCycle;
    }

    /*
    Physics related instances ########################################
     */

    /**
     * Object that stores the world that will be simulated
     */
    private World world;

    /**
     * Variable that stores the in-simulation duration of the flight
     */
    private float elapsedTime = 0;

    /**
     * Variable that stores the in simulation time between the simulation steps
     */
    private float timeStep;

    /**
     * Variable that stores the number of steps that are simulated within as single execution cycle
     */
    private int stepsPerCycle;

   /*
   Server related getters and setters #################################
    */

    /**
     * Getter for the server socket, listens for connections with autopilots
     * @return a the server socket used by the testbed server for outside connections
     */
    private ServerSocket getServerSocket() {
        return serverSocket;
    }

    /**
     * Setter for the server socket used for connecting with the autopilot
     * @param serverSocket the server socket used for connections
     */
    private void setServerSocket(ServerSocket serverSocket) {
        if(!this.canHaveAsServerSocket(serverSocket))
            throw new IllegalArgumentException(INVALID_SERVER_SOCKET);
        this.serverSocket = serverSocket;
    }

    private boolean canHaveAsServerSocket(ServerSocket serverSocket){
        return serverSocket != null && this.getServerSocket() == null;
    }

    /**
     * Getter for the port to connect with
     * @return the TCP port used for connections
     */
    public static int getTcpPort() {
        return TCP_PORT;
    }

    /**
     * Getter for the number of connections maintained by the server
     * @return the number of connections
     */
    private int getMaxNbOfConnections() {
        return maxNbOfConnections;
    }

    /**
     * Getter for the list containing all the connections maintained by the testbed
     * @return the set of all current connections
     */
    private Set<TestbedConnection> getServerTestbedConnections() {
        return serverTestbedConnections;
    }

    /**
     * Getter for the number of threads maintained by the server for the simulation
     * @return the number of threads active in the current thread pool
     */
    private int getMaxNbOfThreads() {
        return maxNbOfThreads;
    }

    /**
     * Getter for the thread pool used by the server to maintain the connections
     * @return the thread pool
     */
    private ExecutorService getThreadPool() {
        return threadPool;
    }

    /**
     * Setter for the thread pool of the server
     * @param threadPool the thread pool for the server
     */
    private void setThreadPool(ExecutorService threadPool) {
        if(!canHaveAsThreadPool(threadPool))
            throw new IllegalArgumentException(INVALID_THREAD_POOL);
        this.threadPool = threadPool;
    }

    /**
     * Checks if the provided thread pool can be assigned as the thread pool of the server
     * @param threadPool the thread pool to be tested
     * @return true if and only if the provided thread pool is a non null reference and the
     *         server doesn't already have an initialized thread pool
     */
    private boolean canHaveAsThreadPool(ExecutorService threadPool){
        return threadPool != null && this.getThreadPool() != null;
    }

    /*
    Server related instances ##########################################
     */
    /**
     * Object that stores the socket of the server that listens for connections
     */
   private ServerSocket serverSocket;

    /**
     * Variable that stores the current used port to listen for TCP connections
     */
    private final static int TCP_PORT = 4242;

    /**
     * Variable that stores the number of connections maintained by the server,
     * is equal to the number of drones and autopilots supported by the server
     */
    private int maxNbOfConnections;

    /**
     * Object that contains a list that stores all the current server threads
     * choice for set so connections can easily be removed
     */
    private Set<TestbedConnection> serverTestbedConnections = new HashSet<>();


    //Todo maybe use one shared thread pool for the world simulation and
    //Todo and the connections since the execution doesn't overlap (resource sharing)
    //note: the nb of threads may be less than the nb of connections if better for performance (submit more tasks to
    //the thread pool than there are threads available)
    /**
     * Variable that stores the number of threads available in the
     * thread pool of the server
     */
    private int maxNbOfThreads;

    /**
     * Object that stores the thread pool needed for the simulation
     */
    private ExecutorService threadPool;


    /*
    Graphics getters and setters ######################################
     */

    /**
     * Getter for the graphics engine, used for generating the objects
     * @return the graphics engine
     */
    private Graphics getGraphics() {
        return graphics;
    }

    /**
     * Setter for the graphics engine
     * @param graphics the graphics engine to be set
     */
    private void setGraphics(Graphics graphics) {
        if(!canHaveAsGraphics(graphics))
            throw new IllegalArgumentException(INVALID_GRAPHICS);
        this.graphics = graphics;
    }

    /**
     * Checks if the provided graphics engine can be set as the graphics engine
     * for the testbed server
     * @param graphics the graphics engine to be tested
     * @return true if and only if graphics is not a null reference and the server
     * hasn't already a graphics engine associated with it
     */
    private boolean canHaveAsGraphics(Graphics graphics){
        return graphics != null && this.getGraphics() == null;
    }

    /**
     * Getter for the drone View
     * @return the drone view
     */
    private Window getDroneView() {
        return droneView;
    }

    /**
     * Setter for the drone view
     * @param droneView the drone view
     */
    private void setDroneView(Window droneView) {
        this.droneView = droneView;
    }

    /**
     * Getter for the top down view
     * @return the top down view
     */
    private Window getTopDownView() {
        return topDownView;
    }

    /**
     * Setter for the top down view
     * @param topDownView the top down view
     */
    private void setTopDownView(Window topDownView) {
        this.topDownView = topDownView;
    }

    /**
     * Getter for the chase view
     * @return the chase view
     */
    private Window getChaseView() {
        return chaseView;
    }

    /**
     * Setter for the chase view
     * @param chaseView the chase view
     */
    private void setChaseView(Window chaseView) {
        this.chaseView = chaseView;
    }

    /**
     * Getter for the side view
     * @return the side view
     */
    private Window getSideView() {
        return sideView;
    }

    /**
     * Setter for the side view
     * @param sideView the side view
     */
    private void setSideView(Window sideView) {
        this.sideView = sideView;
    }

    /*
    Graphics related instances #######################################
     */
    /**
     * Object that stores the graphics engine for the testbed
     */
    private Graphics graphics;

    /**
     * The windows used in the simulation, note no window for the drone camera input
     * because this one is not used for the packet service (may be added later)
     */
    private Window droneView;
    private Window topDownView;
    private Window chaseView;
    private Window sideView;

    /*
    Message strings
     */
    private static final String INVALID_SERVER_SOCKET = "Invalid server socket, provided socket is null reference or the server socket is already initialized";
    private static final String INVALID_THREAD_POOL = "Invalid thread pool, thread pool is already initialized or the provided pool is a null reference";
    private static final String INVALID_GRAPHICS = "Invalid graphics, graphics engine is already initialized or the provided engine is a null reference";


}


