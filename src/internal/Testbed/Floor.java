package internal.Testbed;

import java.io.IOException;
import java.util.HashSet;
import java.util.Set;

import gui.GraphicsObject;
import gui.Tile;
import internal.Helper.Vector;

public class Floor implements WorldObject {

	private Vector position;
	
	private Set<GraphicsObject> floorTiles = new HashSet<>();

	public Floor(Vector position) {
		int n = 41; // n moet oneven zijn
        float nx = 20f;
        float nz = 20f;
		this.position = new Vector(-n*nx/2, 0, -n*nz);
		createFloor(n, nx, nz);
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
		return this.floorTiles;
	}

	public void setAssociatedGraphicsObject(GraphicsObject tile) {
		this.floorTiles.add(tile);
		
	}
	
	public void createFloor(int n, float nx, float nz) {
		
        for (int i = 0; i < 2*n*n; i++) {
        	Vector delta = new Vector(nx*(i%n), 0, nz*(i/n));
        	Vector position = delta.vectorSum(getPosition());
            Vector color = new Vector((60.0f+(i%2)*60), 1, 0.6f);
        	Tile tile = new Tile(position.convertToVector3f(), color.convertToVector3f());
        	tile.setSize(new Vector(nx, 1, nz));
        	this.setAssociatedGraphicsObject(tile);
        }
	}
}