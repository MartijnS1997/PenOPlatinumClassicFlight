package gui;

import math.Matrix3f;
import math.Vector3f;

public interface Polygon extends GraphicsObject{
	
	Vector3f relativePosition = new Vector3f();
	Vector3f orientation = new Vector3f();
	
	public void update(Vector3f displacement, Vector3f orientation);
	
	public default Vector3f getRelPos() {
		Vector3f pos = this.position;
		Matrix3f transformation = Matrix3f.transformationMatrix(this.orientation.negate()).transpose();
		Vector3f difference = transformation.multiply(relativePosition);
		pos = pos.add(difference);
		return pos;
	}
}
