import internal.Exceptions.AngleOfAttackException;
import internal.Autopilot.AutoPilot;
import internal.Physics.PhysXEngine;

import java.util.concurrent.*;

/**
 * Created by Martijn on 14/12/2017.
 */
public class DemoMain {
    private final static String CONNECTION_NAME = "localhost";
    private final static int CONNECTION_PORT = 21212;
    private final static String MODE = PhysXEngine.ALPHA_MODE;
    private final static int timeout = 60;
    private final static String[] testDirectories = {"src/internal/blockData.txt","src/blockDemoEasy.txt","src/blockDemoHard.txt","src/blockCoordinates.txt"};


    public static void main(String args[]) {
        for (int i = 0; i != testDirectories.length; i++) {
            TestbedMain testbed = new TestbedMain(CONNECTION_NAME, CONNECTION_PORT + i, false, MODE, true, testDirectories[i]);
            AutopilotMain autopilot = new AutopilotMain(CONNECTION_NAME, CONNECTION_PORT + i, new AutoPilot(MODE));
            ExecutorService executorTestbed = Executors.newSingleThreadExecutor();
            Future futureTest = executorTestbed.submit(testbed);
            ExecutorService executorAutopilot = Executors.newSingleThreadExecutor();
            Future futureAP = executorAutopilot.submit(autopilot);
            try {
                futureTest.get(timeout, TimeUnit.SECONDS);
                futureAP.get(timeout, TimeUnit.SECONDS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            } catch (ExecutionException e) {
                executorAutopilot.shutdown();
                executorTestbed.shutdown();
                continue;
            } catch (TimeoutException e) {
                futureTest.cancel(true);
                futureAP.cancel(true);
                executorAutopilot.shutdown();
                executorTestbed.shutdown();
                continue;

            } catch (AngleOfAttackException e) {
                executorAutopilot.shutdown();
                executorTestbed.shutdown();
                continue;
            }

            executorAutopilot.shutdown();
            executorTestbed.shutdown();

            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}