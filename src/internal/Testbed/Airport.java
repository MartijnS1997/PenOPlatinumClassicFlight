package internal.Testbed;

import java.io.IOException;
import java.util.HashSet;
import java.util.Set;

import gui.GraphicsObject;
import gui.Tile;
import internal.Helper.Vector;

public class Airport implements WorldObject {

	private Vector position;
	
	private Set<GraphicsObject> airportTiles = new HashSet<>();

	public Airport(Vector position) {
		this.position = position;
		createAirport(15, 280);
	}
	
	@Override
	public void toNextState(float deltaTime) throws IOException {
		// do nothing
	}

	@Override
	public Vector getPosition() {
		return this.position;
	}

	@Override
	public Set<GraphicsObject> getAssociatedGraphicsObjects() {
		return this.airportTiles;
	}

	public void setAssociatedGraphicsObject(GraphicsObject tile) {
		this.airportTiles.add(tile);
		
	}
	
	public void createAirport(int W, int L) {
		createLandStrip(W, L, 0);
		createLandStrip(W, L, 1);
		createGate(W, 0);
		createGate(W, 1);
	}
	
	public void createLandStrip(float W, float L, float number) {
		
		Vector position = this.position.vectorSum(new Vector(0, 0, -(L+W)/2*(float)Math.pow(-1, number)));
		Vector color = new Vector(0, 0, 0.25f*number);
		Tile tile = new Tile(position.convertToVector3f(), color.convertToVector3f());
	    tile.setSize(new Vector(2*W, 1, L));
	    this.setAssociatedGraphicsObject(tile);
	}
	
	public void createGate(float W, float number) {
		Vector position = this.position.vectorSum(new Vector(-W/2*(float)Math.pow(-1, number), 0, 0));
		Vector color = new Vector(0, 0, 0.45f + number/2);
		Tile tile = new Tile(position.convertToVector3f(), color.convertToVector3f());
	    tile.setSize(new Vector(W, 1, W));
	    this.setAssociatedGraphicsObject(tile);
	}
}