import AutopilotInterfaces.*;
import internal.Autopilot.AutoPilot;
import internal.Testbed.FlightRecorder;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;

/**
 * Created by Martijn on 13/11/2017.
 * supervised by bart
 */
public class AutopilotMain implements Runnable {

    /**
     * Constructor for the autopilot mainloop
     * @param connectionName the name of the connection (in our case local thus 'localhost'
     * @param connectionPort the port which trough we connect
     * @param autopilot the autopilot that will be hooked up to the loop
     */
    public AutopilotMain(String connectionName, int connectionPort, AutoPilot autopilot) {
        this.setAutoPilot(autopilot);
        this.setConnectionName(connectionName);
        this.setConnectionPort(connectionPort);
    }

    /**
     * starts the main loop of the autopilot, (was needed to implement runnable)
     */
    @Override
    public void run() {
        try {
            autopilotMainLoop();
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * The autopilot mainloop
     * established connection from the autopilot to the testbed server
     * Also provides the autopilot outputs trough the streams
     * @throws IOException
     * @throws InterruptedException
     */
    private void autopilotMainLoop() throws IOException, InterruptedException {

        //set the flight recorder
        //this.getAutoPilot().setFlightRecorder(this.getFlightRecorder());

        this.initConnection(this.getConnectionName(), this.getConnectionPort());
        this.configAutopilot();

        // now enter the real main loop, also find way to terminate correctly
        while(true){

            try{
                //generate outputs based on the received inputs
                writeOutputs();
            }catch(java.io.EOFException e){
                System.out.println(CLOSING_DOWN);
                //the stream has stopped, termination routine
                this.terminate();
                //nothing to do anymore... leave the main loop
                break;
            }
        }
    }


    /**
     * Generates the output based on the inputs that were written to the stream
     * @throws IOException (just java stuff)
     */
    private void writeOutputs() throws IOException {
        DataInputStream inputStream = this.getInputStream();
        DataOutputStream outputStream = this.getOutputStream();

        AutopilotInputs_v2 inputs = AutopilotInputs_v2Reader.read(inputStream);
        AutopilotOutputs outputs = this.getAutoPilot().timePassed(inputs);
        //System.out.println("outputs: " + outputs);
        AutopilotOutputsWriter.write(outputStream, outputs);
    }

    /**
     * Initializes the connection between the autopilot and the testbed
     * @param connectionName the name of the connection (in our case 'localhost')
     * @param connectionPort the port where to connect from
     * @return the socket containing the connection
     */
    private void initConnection(String connectionName, int connectionPort) throws IOException {
        Socket clientSocket = null;
        boolean connected = false;
        // first try to connect
        while(!connected){
            try{
                clientSocket = new Socket(connectionName, connectionPort);
            }catch(java.net.ConnectException e){
                //if the connection was not available, try again
               continue;
            } catch (UnknownHostException e) {
                //host address is invalid, notify user
                System.out.println(INVALID_HOST_ADDRESS);
                continue;
            } catch (IOException e) {
                //idk what happened, better try again, notify the user
                System.out.println(IO_EXCEPION);
                continue;
            }
            connected = true;
        }

        // set the newly found socket
        this.setAutopilotSocket(clientSocket);

        // configure the streams:
        DataInputStream input = new DataInputStream(this.getAutopilotSocket().getInputStream());
        DataOutputStream output = new DataOutputStream(this.getAutopilotSocket().getOutputStream());

        // set the newly made streams
        this.setInputStream(input);
        this.setOutputStream(output);

    }

    /**
     * Configures the autopilot by reading and writing the appropriate data to the streams
     * first reads the config, then the path and then the inputs
     * @throws IOException
     */
    private void configAutopilot() throws IOException {

        //first get the streams
        DataInputStream inputStream = this.getInputStream();
        DataOutputStream outputStream = this.getOutputStream();

        //then read the configuration from the stream (sent first)
        AutopilotConfig config = AutopilotConfigReader.read(inputStream);
        //read the path
        AutopilotInterfaces.Path path = PathReader.read(inputStream);
        //read the inputs from the stream (provided by the testbed)
        AutopilotInputs_v2 inputs = AutopilotInputs_v2Reader.read(inputStream);
        //set the path of the autopilot
        this.getAutoPilot().setPath(path);
        //get the outputs from the autopilot
        AutopilotOutputs outputs = this.getAutoPilot().simulationStarted(config, inputs);
        //write the outputs generated by the autopilot to the stream
        AutopilotOutputsWriter.write(outputStream, outputs);

    }

    /**
     * implements the termination routine of the drone
     * @throws IOException just java things
     */
    private void terminate() throws IOException {
        DataInputStream inputStream = this.getInputStream();
        DataOutputStream outputStream = this.getOutputStream();
        Socket autopilotSocket = this.getAutopilotSocket();

        inputStream.close();
        outputStream.close();
        autopilotSocket.close();
    }

    /**
     * Getter for the autopilot used in the loop
     * @return the autopilot
     */
    private AutoPilot getAutoPilot() {
        return autoPilot;
    }

    /**
     * Setter for the autopilot used in the loop
     * @param autoPilot
     */
    private void setAutoPilot(AutoPilot autoPilot) {
        this.autoPilot = autoPilot;
    }

    /**
     * Getter for the connection name used for communication with the testbed
     * @return
     */
    private String getConnectionName() {
        return connectionName;
    }

    /**
     * Setter for the connection name used for communication with the testbed
     * @param connectionName
     */
    private void setConnectionName(String connectionName) {
        this.connectionName = connectionName;
    }

    /**
     * getter for the port nb of the connection used for communication with the testbed
     * @return
     */
    private int getConnectionPort() {
        return connectionPort;
    }

    /**
     * Setter for the connection port nb used for the connection with the testbed
     * @param connectionPort
     */
    private void setConnectionPort(int connectionPort) {
        this.connectionPort = connectionPort;
    }

    /**
     * Getter for the flight recorder (used for diagnosis)
     * @return
     */
    private FlightRecorder getFlightRecorder() {
        return flightRecorder;
    }

    /**
     * Setter for the flight recorder (used in diagnosis)
     * @param flightRecorder
     */
    public void setFlightRecorder(FlightRecorder flightRecorder) {
        this.flightRecorder = flightRecorder;
    }

    /**
     * Getter for the socket used for the communication with the testbed
     * This socket is used to generate the streams and establish the connection, for good practice
     * close upon completion
     * @return
     */
    private Socket getAutopilotSocket() {
        return autopilotSocket;
    }

    /**
     * Setter for the socket of the autopilot used for communication with the testbed
     * @param autopilotSocket
     */
    private void setAutopilotSocket(Socket autopilotSocket) {
        this.autopilotSocket = autopilotSocket;
    }

    /**
     * Getter for the output stream of the autopilot used for communication with the testbed
     * the output stream outputs the data from the drone to the testbed (where the testbed
     * receives the data on its input stream)
     * @return the output communication stream
     * note: close this when the loop is finished for good practice
     */
    private DataOutputStream getOutputStream() {
        return outputStream;
    }

    /**
     * Setter for the output stream of the autopilot used for communication with the testbed
     * @param outputStream
     */
    private void setOutputStream(DataOutputStream outputStream) {
        this.outputStream = outputStream;
    }

    /**
     * Getter for the input stream of the autopilot used for communication with the testbed
     * the input stream receives the results outputted from the testbed (the new location, orientation, elapsed time
     * and camera inputs)
     * @return the input communication stream
     * note: close this when loop is finished for good practices
     */
    private DataInputStream getInputStream() {
        return inputStream;
    }

    /**
     * Setter for the input stream of the autopilot used for communication with the testbed
     * @param inputStream
     */
    public void setInputStream(DataInputStream inputStream) {
        if(this.getInputStream() != null)
            throw new IllegalArgumentException();
        this.inputStream = inputStream;

    }

    private AutoPilot autoPilot;
    private FlightRecorder flightRecorder;
    private String connectionName;
    private int connectionPort;

    /**
     * stream configuration
     */
    private DataOutputStream outputStream;
    private DataInputStream inputStream;
    private Socket autopilotSocket;

    /**
     * Error & Diagnosis messages
     */

    private final static String ACTIVE_STREAM = "This stream is already active, please check if the socket" +
            "is still active or not.";
    private final static String IO_EXCEPION = "An IO exception was triggered, investigate the cause please.";
    private final static String INVALID_HOST_ADDRESS = "the host address is not valid, if intended use is to run" +
            "on same device use the 'localhost' as connection name";
    private final static String CLOSING_DOWN = "Closing down the client";
}