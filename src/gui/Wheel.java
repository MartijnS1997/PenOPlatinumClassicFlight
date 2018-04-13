package gui;

import static org.lwjgl.glfw.GLFW.glfwMakeContextCurrent;
import static org.lwjgl.system.MemoryUtil.NULL;

import java.util.Arrays;

import internal.Helper.HSVconverter;
import internal.Helper.Vector;
import math.Matrix3f;
import math.Vector3f;
import tests.VectorTest;

public class Wheel implements Polygon{

	private float[] positions;
	private int[] indices;
	private float[] colours;
	
	private Vector3f relativePosition = new Vector3f();
	private Vector3f orientation = new Vector3f();

	static Graphics g;
	
	private Mesh mesh;
	private Vector3f position = new Vector3f();
	private Vector3f size = new Vector3f(1f, 1f, 1f);
	
	static public void setGraphics(Graphics graphics) {
		g = graphics;
	}
	
	public Wheel(Vector3f colour, int nPolygon) {
		setData(colour, nPolygon);

		for (String key: g.windows.keySet()) {
			glfwMakeContextCurrent(g.windows.get(key).getHandler());
			this.mesh = new Mesh();
			mesh.init(positions, colours, indices);
			glfwMakeContextCurrent(NULL);
		}
	}
	
	public Wheel(Vector3f position, Vector3f colour, int nbOfSides) {
		this(colour, nbOfSides);
		
		this.position = position;
		
	}

	public Wheel(Vector3f relativePosition, Vector3f colour, Cube attachedCube, int nbOfSides) {
		this(attachedCube.getPos(), colour, nbOfSides);
		
		this.relativePosition = relativePosition;
	}

	public void update(Vector3f displacement, Vector3f orientation) {
		this.orientation  = orientation.negate();
		position = position.add(displacement);
	}
	
	public Vector3f getRelPos() {
		Vector3f pos = this.position;
		Matrix3f transformation = Matrix3f.transformationMatrix(this.orientation.negate()).transpose();
		Vector3f difference = transformation.multiply(relativePosition);
		pos = pos.add(difference);
		return pos;
	}
	
	public void render() {
		this.mesh.render();
	}
	
	public void delete() {
		this.mesh.delete();
	}

	public Vector getPosition(){
		return Vector.vector3fToVector(this.getPos());
	}
	
	public Vector3f getSize() {
		return this.size;
	}
	
	public Vector3f getPos() {
		return this.position;
	}
	
	public void setSize(float size) {
		this.size = new Vector3f(size, size, size);
	}
	
	public void setSize(Vector3f size) {
		this.size = size;
	}
	
	public void setSize(Vector vector) {
		this.setSize(vector.convertToVector3f());
	}
	
	public void setData(Vector3f colour, int nPolygon) {	
		Vector3f rgbColour = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		
		this.colours = new float[3*2*(nPolygon+1)];
		for (int i = 0; i<3*2*(nPolygon+1); i++) {
			if (i%3 == 0)
				this.colours[i] = rgbColour.x;
			if (i%3 == 1)
				this.colours[i] = rgbColour.y;
			if (i%3 == 2)
				this.colours[i] = rgbColour.z;
		}
		
		this.positions = new float[3*2*(nPolygon+1)];
		
		this.positions[0] = 0.5f;
		this.positions[1] = 0.0f;
		this.positions[2] = 0.0f;
		
		this.positions[3] = -0.5f;
		this.positions[4] = 0.0f;
		this.positions[5] = 0.0f;
		
		for (int i = 6; i< 3*(nPolygon+1)+3; i++) {
			if (i%3 == 0)
				this.positions[i] = 0.5f;
			if (i%3 == 1)
				this.positions[i] = (float) Math.cos(((i-6)/3)*2*Math.PI/nPolygon);
			if (i%3 == 2)
				this.positions[i] = (float) Math.sin(((i-6)/3)*2*Math.PI/nPolygon);
		}
		
		
		
		for (int i = 3*(nPolygon+1)+3; i< 6*(nPolygon+1); i++) {
			if (i%3 == 0)
				this.positions[i] = -0.5f;
			if (i%3 == 1)
				this.positions[i] = (float) Math.cos(((i-6)/3)*2*Math.PI/nPolygon);
			if (i%3 == 2)
				this.positions[i] = (float) Math.sin(((i-6)/3)*2*Math.PI/nPolygon);
		}
		
		this.indices = new int[4*3*nPolygon];
		for (int i = 0; i < nPolygon; i++) {
			this.indices[3*i] = 0;
			this.indices[3*i+1] = 2+(i+1)%nPolygon;
			this.indices[3*i+2] = 2+i;
		}
		for (int i = 0; i < nPolygon; i++) {
			this.indices[3*nPolygon+3*i] = 1;	
			this.indices[3*nPolygon+3*i+1] = 2+nPolygon+i;
			this.indices[3*nPolygon+3*i+2] = 2+nPolygon+(i+1)%nPolygon;
		}
		for (int i = 0; i < nPolygon; i++) {
			this.indices[6*nPolygon+6*i] = 2+i;	
			this.indices[6*nPolygon+6*i+1] = 2+(i+1)%nPolygon;
			this.indices[6*nPolygon+6*i+2] = 2+nPolygon+i;
			this.indices[6*nPolygon+6*i+3] = 2+nPolygon+(i+1)%nPolygon;
			this.indices[6*nPolygon+6*i+4] = 2+nPolygon+i;
			this.indices[6*nPolygon+6*i+5] = 2+(i+1)%nPolygon;
		}
		
	}
}
