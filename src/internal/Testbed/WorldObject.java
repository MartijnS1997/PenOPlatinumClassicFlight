package internal.Testbed;

import java.io.IOException;
import java.util.Set;

import gui.Cube;
import gui.GraphicsObject;
import internal.Helper.Vector;

/**
 * 
 * @author anthonyrathe
 *
 */
public interface WorldObject {
	// public abstract void evolve(float duration) throws IOException;

	/**
	 * Advances the world object for a given delta time
	 * @param deltaTime the size of the time step
	 * @throws IOException 
	 */
	void toNextState(float deltaTime) throws IOException;

	/**
	 * Checks if the time difference is valid
	 * @param deltaTime the time step to be tested
	 * @return true if and only if deltaTime > 0.0f
	 */
	static boolean isValidTimeStep(float deltaTime){
		return deltaTime > 0.0f;
	}

	static boolean canHaveAsWorld(World world){
		return true;
	};

	/**
	 * Getter for the position of the world object
	 * @return the position of the world object in vector format
	 */
	Vector getPosition();

	Set<GraphicsObject> getAssociatedGraphicsObjects();
}
