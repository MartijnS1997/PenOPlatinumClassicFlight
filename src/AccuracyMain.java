import internal.Exceptions.AngleOfAttackException;
import internal.Autopilot.AutoPilot;
import internal.Physics.PhysXEngine;

import java.util.concurrent.*;

/**
 * Created by Martijn on 11/12/2017.
 */
public class AccuracyMain {
    public final static String CONNECTION_NAME = "localhost";
    public final static int CONNECTION_PORT = 21212;
    public final static String MODE = PhysXEngine.BETA_MODE;
    public final static int nbTests = 100;
    public final static int timeout = 40;


    public static void main(String args[]){
        int nbSuccesses = 0;
        for(int i = 0; i !=nbTests; i++) {
            TestbedMain testbed = new TestbedMain(CONNECTION_NAME, CONNECTION_PORT+i, true, MODE);
            AutopilotMain autopilot = new AutopilotMain(CONNECTION_NAME, CONNECTION_PORT+i, new AutoPilot(MODE));
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

                    // let it go
                System.out.println("terminated!");
                continue;
            } catch (AngleOfAttackException e){
                executorAutopilot.shutdown();
                executorTestbed.shutdown();
                continue;
            }

            executorAutopilot.shutdown();
            executorTestbed.shutdown();

            nbSuccesses += 1;

            System.out.println("Nb of Succesful Runs: " + nbSuccesses + "/" + (i+1));

        }
    }
}
