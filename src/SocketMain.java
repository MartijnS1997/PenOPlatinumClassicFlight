import internal.Autopilot.AutoPilot;
import internal.Exceptions.AngleOfAttackException;
import internal.Exceptions.SimulationEndedException;
import internal.Physics.PhysXEngine;
import internal.Testbed.FlightRecorder;

import java.io.*;


/**
 * Created by Martijn on 13/11/2017.
 * and supervised by Bart
 */
public class SocketMain {
    public static void main(String[] args) throws IOException, InterruptedException {
        //System.out.println(PhysXEngine.GAMMA_MODE);

        MAIN_TESTBED.setFlightRecorder(FLIGHT_RECORDER);
        MAIN_AUTOPILOT.setFlightRecorder(FLIGHT_RECORDER);

        //first create both threads
        //TestbedMain mainTestbed = new TestbedMain();
        Thread testbedThread = new Thread(MAIN_TESTBED);
        Thread autopilotThread = new Thread(MAIN_AUTOPILOT);

        testbedThread.setUncaughtExceptionHandler(HANDLER);

        testbedThread.setPriority(Thread.MAX_PRIORITY);

        testbedThread.start();
        Thread.sleep(1000);
        autopilotThread.start();

        //mainTestbed.testbedMainLoop();


    }

    static Thread.UncaughtExceptionHandler HANDLER =  new Thread.UncaughtExceptionHandler() {
        public void uncaughtException(Thread th, Throwable ex) {
            switch(ex.toString()){
                case SimulationEndedException.TAG:
                    System.out.println("Handling ended simulation");
                    return;
                // if it is an angle of attack exception, diagnose the issue
                case AngleOfAttackException.TAG:
                    try {
                        FLIGHT_RECORDER.saveDiagnosisWingIssues(0, "diagnosis.txt");
                    } catch (FileNotFoundException e) {
                        e.printStackTrace();
                    }
                    return;

                default:
                    System.out.println("GENERIC ERROR");
                    ex.printStackTrace();
                    try {
                        FLIGHT_RECORDER.saveDiagnosisWingIssues(20, "diagnosis.txt");
                    } catch (FileNotFoundException e) {
                        e.printStackTrace();
                    }
                    ex.printStackTrace();
                    return;
            }
        }
    };

    public final static String CONNECTION_NAME = "localhost";
    public final static int CONNECTION_PORT = 21212;
    public final static FlightRecorder FLIGHT_RECORDER = new FlightRecorder(5, true);
    public final static String MODE = PhysXEngine.GAMMA_MODE;
    //Todo add the recorder to the autopilot and the testbed: used for diagnis during flight;
    private static TestbedMain MAIN_TESTBED = new TestbedMain(CONNECTION_NAME, CONNECTION_PORT, false, MODE);
    private static AutopilotMain MAIN_AUTOPILOT = new AutopilotMain(CONNECTION_NAME, CONNECTION_PORT, new AutoPilot(MODE));



}
