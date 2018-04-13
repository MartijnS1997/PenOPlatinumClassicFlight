package internal.Autopilot;

/**
 * Created by Martijn on 13/03/2018.
 * an enumeration of flight states for an autopilot
 * TAKEOFF: the autopilot objective is to takeoff
 * FLIGHT: the autopilot objective is passing by all the cubes in the world
 * WAY_POINT: the autopilot objective is following way points for flying back to the takeoff point
 * LANDING: the autopilot objective is getting the drone safely to ground again
 * TAXIINg: the autopilot objective is taxiing to a specified point on the ground
 */
public enum FlightState {
    TAKEOFF, FLIGHT, WAY_POINT, LANDING, TAXIING
}
