package gui;

import static org.lwjgl.glfw.GLFW.glfwMakeContextCurrent;
import static org.lwjgl.system.MemoryUtil.NULL;

import java.util.Arrays;


import internal.Helper.HSVconverter;
import internal.Helper.Vector;
import math.Vector3f;

public class Tile implements GraphicsObject {

	static float[] positions = new float[]{

			-0.5f,  0.0f, -0.5f,
			-0.5f,  0.0f,  0.5f,
			0.5f,  0.0f,  0.5f,
			0.5f,  0.0f, -0.5f,
	};
	static int[] indices = new int[]{
			0, 1, 2,
			0, 2, 3,
	};
	
	private float[] colours;

	static Graphics g;
	
	private Mesh mesh;
	private Vector3f position = new Vector3f();
	private Vector3f size = new Vector3f(1f, 1f, 1f);
	
	static Vector3f positionOffset = new Vector3f();
	
	static public void setGraphics(Graphics graphics) {
		g = graphics;
	}
	
	public Tile(Vector3f colour) {
		setColours(colour);
		
		for (String key: g.windows.keySet()) {
			glfwMakeContextCurrent(g.windows.get(key).getHandler());
			this.mesh = new Mesh();
			mesh.init(positions, colours, indices);
			glfwMakeContextCurrent(NULL);
		}
	}
	
	public Tile(Vector3f position, Vector3f colour) {
		this(colour);
		
		this.position = position;
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
	
	public Vector3f getPosWithOffset() {
		return this.position.add(new Vector3f(2*size.x * (int) (positionOffset.x/(2*size.x)), 2*size.y * (int) (positionOffset.y/(2*size.y)), 2*size.z * (int) (positionOffset.z/(2*size.z))));
	}
	
	public static void setPosOffset(Vector3f offset) {
		positionOffset = offset;
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
	
	public void setColours(Vector3f colour) {	
		Vector3f posY = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		Vector3f negY = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		Vector3f posX = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		Vector3f negX = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		Vector3f posZ = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		Vector3f negZ = Vector3f.ArrayToVector3f(HSVconverter.HSVtoRGB2(colour.x, colour.y, colour.z));
		
		this.colours = new float[]{
				posZ.x, posZ.y, posZ.z,
				posZ.x, posZ.y, posZ.z,
				posZ.x, posZ.y, posZ.z,
				posZ.x, posZ.y, posZ.z,
				
				posY.x, posY.y, posY.z,
				posY.x, posY.y, posY.z,
				posY.x, posY.y, posY.z,
				posY.x, posY.y, posY.z,
				
				posX.x, posX.y, posX.z,
				posX.x, posX.y, posX.z,
				posX.x, posX.y, posX.z,
				posX.x, posX.y, posX.z,
				
				negX.x, negX.y, negX.z,
				negX.x, negX.y, negX.z,
				negX.x, negX.y, negX.z,
				negX.x, negX.y, negX.z,
				
				negY.x, negY.y, negY.z,
				negY.x, negY.y, negY.z,
				negY.x, negY.y, negY.z,
				negY.x, negY.y, negY.z,
				
				negZ.x, negZ.y, negZ.z,
				negZ.x, negZ.y, negZ.z,
				negZ.x, negZ.y, negZ.z,
				negZ.x, negZ.y, negZ.z,
		};
	}

	@Override
	public String toString() {
		return "Cube{" +
				"colours=" + Arrays.toString(colours) +
				'}';
	}
}