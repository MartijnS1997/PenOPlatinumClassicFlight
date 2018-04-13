package internal.Autopilot;

import internal.Helper.Vector;

/**
 * Created by Martijn on 21/03/2018.
 * an interface for a path generator
 */
public interface AutopilotPathGenerator {

    Vector getNextWayPoint(Vector position, Vector velocity, Vector destination);

    Vector getNextWayPointMissed(Vector position, Vector velocity, Vector destination);

    Vector getNextWayPointSucces(Vector position, Vector velocity, Vector destination);
}
