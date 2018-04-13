package internal.Autopilot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import internal.Helper.Vector;

/**
 * Class for generating (landing) paths
 * @author Anthony Rathe
 *
 */
public class PathGenerator {

	public PathGenerator() {
		pathUnlock();
	}

	public List<Vector> getPath(){
		return this.path;
	}

	private void setPath(List<Vector> path) {
		this.path = path;
	}


	private void pathLock() {
		pathLock = true;
	}

	public void pathUnlock() {
		pathLock = false;
	}

	public boolean hasPathLocked() {
		return pathLock;
	}

	public void generateLandingPath(Vector position, Vector velocity, Vector destination){

		Vector landingVector = getLandingVector(position, velocity, destination);

		List<Vector> blockCoordinates = new ArrayList<Vector>();

		if (canMakeTurn(position, velocity, destination)) {


			// generate landing path
			Vector lastBlock = destination;
			blockCoordinates.add(lastBlock);
			while(lastBlock.getyValue() < (position.getyValue()-0.01)) {

				lastBlock = lastBlock.vectorSum(landingVector);
				if (lastBlock.getyValue() >= position.getyValue()) {
					lastBlock = new Vector(lastBlock.getxValue(),position.getyValue(),lastBlock.getzValue());
				}
				blockCoordinates.add(lastBlock);
			}

			// generate additional path
			float additionalDistance = forwardDistanceBetween(position, velocity, lastBlock) + RUN_DISTANCE;
			float prolongedDistance = 0;
			Vector prolongVector = velocity.makeHorizontal().normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS);
			if (additionalDistance > 0) {
				// path between exit of semi-circle and start of descent should be prolonged
				while (prolongedDistance < additionalDistance) {
					prolongedDistance += DISTANCE_BETWEEN_LANDING_BLOCKS;
					lastBlock = lastBlock.vectorSum(prolongVector);
					blockCoordinates.add(lastBlock);
				}
			}

			// generate semi-circle
			float circleDiameter = parallelDistanceBetween(position, velocity, destination);
			int numberOfSemiCircleWaypoints = (int)Math.floor((double)(Math.PI*circleDiameter/2f/DISTANCE_BETWEEN_LANDING_BLOCKS))-1;
			if (lastBlock.toTheRightOf(position, velocity)) {
				Vector circleCenter = lastBlock.vectorSum(prolongVector.rotateAroundYAxis((float)Math.PI/2f).normalizeToLength(circleDiameter/2f));
				Vector radius = prolongVector.rotateAroundYAxis((float)Math.PI/2f).normalizeToLength(circleDiameter/2f).scalarMult(-1f);

				for (int i = 0; i < numberOfSemiCircleWaypoints; i++) {
					radius = radius.rotateAroundYAxis((float)(Math.PI)/numberOfSemiCircleWaypoints);
					lastBlock = circleCenter.vectorSum(radius);
					blockCoordinates.add(lastBlock);
				}
			}else {
				Vector circleCenter = lastBlock.vectorSum(prolongVector.rotateAroundYAxis(-(float)Math.PI/2f).normalizeToLength(circleDiameter/2f));
				Vector radius = prolongVector.rotateAroundYAxis(-(float)Math.PI/2f).normalizeToLength(circleDiameter/2f).scalarMult(-1f);

				for (int i = 0; i < numberOfSemiCircleWaypoints; i++) {
					radius = radius.rotateAroundYAxis(-(float)(Math.PI)/numberOfSemiCircleWaypoints);
					lastBlock = circleCenter.vectorSum(radius);
					blockCoordinates.add(lastBlock);
				}
			}


			// generate additional path
			// path before start of semi-circle should be prolonged
			while (prolongedDistance < Math.max(additionalDistance - DISTANCE_BETWEEN_LANDING_BLOCKS + RUN_DISTANCE, RUN_DISTANCE - DISTANCE_BETWEEN_LANDING_BLOCKS)) {
				prolongedDistance += DISTANCE_BETWEEN_LANDING_BLOCKS;
				lastBlock = lastBlock.vectorSum(prolongVector.scalarMult(-1));
				blockCoordinates.add(lastBlock);
			}
			
			Collections.reverse(blockCoordinates);
    		
    		pathLock();
    		
    	}else {
    		// get the drone further away from the destination in order to be able to turn
    		
    		if (position.makeHorizontal().distanceBetween(getLandingStartPosition(position, parallelHorizontalVector(position, destination), destination).makeHorizontal()) >= STEEPEST_TURN_DIAMETER) {
    			// if distance between drone and destination is large enough to make a turn, start aligning
        		if (getLandingStartPosition(position, parallelHorizontalVector(position, destination), destination).makeHorizontal().toTheRightOf(position, velocity)) {
        			// Turn right
        			blockCoordinates.add(position.vectorSum(velocity.makeHorizontal().rotateAroundYAxis(-ALIGNMENT_TURN_ANGLE).normalizeToLength(velocity.getSize()*ALIGNMENT_TURN_FACTOR)));
        		}else {
        			// Turn left
        			blockCoordinates.add(position.vectorSum(velocity.makeHorizontal().rotateAroundYAxis(ALIGNMENT_TURN_ANGLE).normalizeToLength(velocity.getSize()*ALIGNMENT_TURN_FACTOR)));
        		}
    		}else {
    			// if distance between drone and destination is too small to allow for a proper turn, keep on flying away (horizontally) from destination
    	    	blockCoordinates.add(position.vectorSum(velocity.makeHorizontal().scalarMult(ALIGNMENT_TURN_FACTOR)));
    		}
    		
    		pathLock();
    	}
    	
    	setPath(blockCoordinates);
        
	}

	private Vector getLandingVector(Vector position, Vector direction, Vector destination) {

	    	Vector horizontalDirection = direction.makeHorizontal();
	    	// Formula: y = sqrt(x^2+z^2)*tan(landingAngle)
	    	return new Vector(horizontalDirection.getxValue(),(float)(Math.sqrt(Math.pow(horizontalDirection.getxValue(),2)+Math.pow(horizontalDirection.getzValue(),2))*Math.tan(LANDING_ANGLE)),horizontalDirection.getzValue()).normalizeToLength(DISTANCE_BETWEEN_LANDING_BLOCKS);
	    }
	    
    private boolean canMakeTurn(Vector position, Vector direction, Vector destination) {
    	return parallelDistanceBetween(position, direction, getLandingStartPosition(position, direction, destination)) >= STEEPEST_TURN_DIAMETER;
    }

    /**
     * Returns the shortest distance between a vector with direction "direction", placed in
     * position, and a vector with direction "direction", placed in destination.
     * @param position
     * @param direction
     * @param destination
     * @author Anthony Rathe
     */
    private static float parallelDistanceBetween(Vector position, Vector direction, Vector destination) {
    	Vector horizontalDirection = direction.makeHorizontal();
    	Vector horizontalPosition = position.makeHorizontal();
    	Vector horizontalDestination = destination.makeHorizontal();
    	Vector dMinusP = horizontalDestination.vectorDifference(horizontalPosition);

    	// Formula: d = (destinationVector - positionVector).size()*cos(angleBetween(destinationVector - positionVector,directionVector)-PI/2)
    	return (float)(dMinusP.getSize()*Math.cos(dMinusP.getAngleBetween(horizontalDirection)-Math.PI/2f));
    }

    private Vector getLandingStartPosition(Vector position, Vector direction, Vector destination) {
    	Vector landingVector = getLandingVector(position, direction, destination);

    	Vector lastBlock = destination;

		while(lastBlock.getyValue() < position.getyValue()) {
			lastBlock = lastBlock.vectorSum(landingVector);
			if (lastBlock.getyValue() >= position.getyValue()) {
				lastBlock = new Vector(lastBlock.getxValue(),position.getyValue(),lastBlock.getzValue());
			}
		}
		return lastBlock;
	}

	/**
	 * Return the distance a point moving in the direction "direction" should travel horizontally, starting from "destination", to arrive
	 * in a parallel position, next to "position".
	 * @param position
	 * @param direction
	 * @param destination
	 * @author Anthony Rathe
	 */
	private static float forwardDistanceBetween(Vector position, Vector direction, Vector destination) {
		Vector horizontalDirection = direction.makeHorizontal();
		Vector horizontalPosition = position.makeHorizontal();
		Vector horizontalDestination = destination.makeHorizontal();
		Vector dMinusP = horizontalDestination.vectorDifference(horizontalPosition);

		// Formula: d = (destinationVector - positionVector).size()*cos(angleBetween(destinationVector - positionVector,directionVector)-PI/2)
		float alfa = (float)(dMinusP.getAngleBetween(horizontalDirection)-Math.PI/2f);
		float d = (float)(dMinusP.getSize()*Math.sin(alfa));
		return Math.signum(alfa)*d;

	}

	/**
	 * Generates a vector that is oriented perpendicularly on the horizontal cord between the two given points.
	 * Points to the left when standing in position2, looking towards position1
	 * @param position1
	 * @param position2
	 * @return
	 */
	private static Vector parallelHorizontalVector(Vector position1, Vector position2) {
		position1 = position1.makeHorizontal();
		position2 = position2.makeHorizontal();

		Vector distanceBetween = position1.vectorDifference(position2);
		return distanceBetween.rotateAroundYAxis((float)Math.PI/2);

	}

	/**
	 * Method for retrieving the next waypoint we should reach
	 * @param position
	 * @param velocity
	 * @param destination
	 * @return
	 */
	public Vector getNextWaypoint(Vector position, Vector velocity, Vector destination) {
		if (!hasPathLocked())generateLandingPath(position, velocity, destination);
		return getPath().get(0);
	}

	/**
	 * Method for retrieving the next waypoint we should reach, after having just reached one
	 * @param position
	 * @param velocity
	 * @param destination
	 * @return
	 */
	public Vector getNextWaypointSuccess(Vector position, Vector velocity, Vector destination) {
		if (!hasPathLocked()) {
			// Generate a path if none was generated
			generateLandingPath(position, velocity, destination);
		}else {
			if (getPath().size() <= 1) {
				// If path contains only one element, generate a new path
				generateLandingPath(position, velocity, destination);
			}else {
				// If path is long enough, erase first element
				List<Vector> newPath = getPath();
				newPath.remove(0);
				setPath(newPath);
			}
		}
		return getPath().get(0);

	}

	/**
	 * Method for retrieving the next waypoint we should reach, after having missed one.
	 * Always regenerates the path.
	 * @param position
	 * @param velocity
	 * @param destination
	 * @return
	 */
	public Vector getNextWaypointMissed(Vector position, Vector velocity, Vector destination) {
		// Always generate a new path
		generateLandingPath(position, velocity, destination);
		return getPath().get(0);
	}


	private static final float DISTANCE_BETWEEN_LANDING_BLOCKS = 20f;
	private static final float LANDING_ANGLE = (float)Math.PI/12;
	private static final float STEEPEST_TURN_DIAMETER = 20f;
	private static final float ALIGNMENT_TURN_ANGLE = (float)Math.PI/8;
	private static final float ALIGNMENT_TURN_FACTOR = 2.5f;
	private static final float RUN_DISTANCE = 40f;

	private boolean pathLock;
	private List<Vector> path;
}
