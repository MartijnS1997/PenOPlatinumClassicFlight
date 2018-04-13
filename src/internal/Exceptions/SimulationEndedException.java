package internal.Exceptions;

/**
 * Created by Martijn on 25/10/2017.
 * An exception class for finishing the simulation (used in the main loop)
 */
public class SimulationEndedException extends IllegalArgumentException {

    public SimulationEndedException(){
        //System.out.println("Hurray the drone has reached the cube!");
    }

    @Override
    public String toString() {
        return TAG;
    }

    public final static String TAG = "SIMULATION_ENDED";
}
