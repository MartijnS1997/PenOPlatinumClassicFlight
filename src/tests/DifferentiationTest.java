package tests;

import AutopilotInterfaces.AutopilotOutputs;
import internal.Testbed.Drone;
import internal.Helper.Vector;
import internal.Testbed.WorldBuilder;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;
import java.io.PrintWriter;

/**
 * Created by Martijn on 16/11/2017.
 * created to test the different differentiation techniques
 */
public class DifferentiationTest {

    public static Drone drone;
    public static float INTERVAL1 = 1E-3f;
    public static float INTERVAL2 = 1E-4f;
    public static int NB_ITERATIONS1 = 10000;
    public static int NB_ITERATIONS2 = 100000;


    @Before
    public void setupMutableFixture(){
        WorldBuilder builder = new WorldBuilder();
        drone = builder.getDrone();
    }

    @Test
    public void differentiationTest() throws IOException {
        String firstTestString = "";
        drone.setAutopilotOutputs(new AutopilotOutputs() {
            @Override
            public float getThrust() {
                return 32.857273f;
            }

            @Override
            public float getLeftWingInclination() {
                return (float) Math.PI / 12;
            }

            @Override
            public float getRightWingInclination() {
                return (float) Math.PI / 12;
            }

            @Override
            public float getHorStabInclination() {
                return 0;
            }

            @Override
            public float getVerStabInclination() {
                return 0;
            }

            @Override
            public float getFrontBrakeForce() {
                return 0;
            }

            @Override
            public float getLeftBrakeForce() {
                return 0;
            }

            @Override
            public float getRightBrakeForce() {
                return 0;
            }
        });
        for (int i = 0; i != NB_ITERATIONS1; i++) {
            drone.toNextState(INTERVAL1);
            //every hundred ms add an entry to the string
            if (i % 100 == 0) {
                Vector dronePos = drone.getPosition();
                firstTestString += Float.toString(dronePos.getxValue()) + ";" +
                        Float.toString(dronePos.getyValue()) + ";" + Float.toString(dronePos.getzValue()) + ";";
            }
        }

        PrintWriter printerOutput = new PrintWriter("differentiationTest1.txt");
        printerOutput.print(firstTestString);
        printerOutput.close();
    }
    @Test
    public void testDifference2() throws IOException {

        drone.setAutopilotOutputs(new AutopilotOutputs() {
            @Override
            public float getThrust() {
                return 32.857273f;
            }

            @Override
            public float getLeftWingInclination() {
                return (float) Math.PI / 12;
            }

            @Override
            public float getRightWingInclination() {
                return (float) Math.PI / 12;
            }

            @Override
            public float getHorStabInclination() {
                return 0;
            }

            @Override
            public float getVerStabInclination() {
                return 0;
            }

            @Override
            public float getFrontBrakeForce() {
                return 0;
            }

            @Override
            public float getLeftBrakeForce() {
                return 0;
            }

            @Override
            public float getRightBrakeForce() {
                return 0;
            }
        });

        drone.setPosition(new Vector());
        drone.setVelocity(new Vector());
        drone.setOrientation(new Vector());
        drone.setRotationVector(new Vector());
        String secondTestString = "";
        for(int i = 0; i != NB_ITERATIONS2; i++) {
            drone.toNextState(INTERVAL2);
            //every hundred ms add an entry to the string
            if(i%1000 == 0){
                Vector dronePos = drone.getPosition();
                secondTestString += Float.toString(dronePos.getxValue()) + ";" +
                        Float.toString(dronePos.getyValue()) + ";" + Float.toString(dronePos.getzValue())+";";
            }
        }

        PrintWriter printerOutput = new PrintWriter("differentiationTest2.txt");
        printerOutput.print(secondTestString);
        printerOutput.close();


    }



}
