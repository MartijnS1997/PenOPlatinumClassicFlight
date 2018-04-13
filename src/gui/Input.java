package gui;

import static org.lwjgl.glfw.GLFW.GLFW_KEY_A;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_D;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_DOWN;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_LEFT;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_LEFT_ALT;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_RIGHT;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_S;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_SPACE;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_UP;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_W;
import static org.lwjgl.glfw.GLFW.GLFW_PRESS;
import static org.lwjgl.glfw.GLFW.glfwGetKey;

import org.lwjgl.glfw.GLFW;

import math.Matrix4f;
import math.Vector3f;

public class Input {
	
    private Vector3f position;
	private Mouse mouse;

	private float yaw = 0;
	private float pitch = 0;
	
    private static final float SPEED = 0.01f;
    private static final float TURN_SPEED = 0.00003f;
    private static final float camDistance = 130f;
    
    private Vector3f right = new Vector3f(1, 0, 0);
    private Vector3f up = new Vector3f(0, 1, 0);
    private Vector3f look = new Vector3f(0, 0, -1);
	private Vector3f hop = new Vector3f();

    Input(Settings setting) {
    	switch (setting) {
    	case DRONE_TOP_DOWN_CAM: 
    		position = new Vector3f(hop.x, hop.y + camDistance, -120f + hop.z);
			yaw = (float) Math.PI/2;
			pitch = (float) -Math.PI/2;
			break;
		case DRONE_SIDE_CAM: 
			position = new Vector3f(hop.x + camDistance, hop.y, -120f + hop.z);
    		yaw = (float) Math.PI/2;
    		pitch = 0;
    		break;
    	default: position = new Vector3f();
    		break;
    	}
    	
    	right = new Vector3f((float) Math.cos(yaw), 0, (float) -Math.sin(yaw));
		up = new Vector3f((float) (Math.sin(pitch)*Math.sin(yaw)), (float) Math.cos(pitch), (float) (Math.sin(pitch)*Math.cos(yaw)));
		look = up.cross(right);
    	
    	mouse = new Mouse(GLFW.glfwGetCurrentContext());
    	
    	
    	
    }

    /**
     * Processes input.
     */
	public void processInput() {
		
		double delta = Time.getDelta();
		
		mouse.update();
		yaw = yaw - mouse.dx() * TURN_SPEED * (float)delta;
		pitch = pitch + mouse.dy() * TURN_SPEED * (float)delta;

		right = new Vector3f((float) Math.cos(yaw), 0, (float) -Math.sin(yaw));
		up = new Vector3f((float) (Math.sin(pitch)*Math.sin(yaw)), (float) Math.cos(pitch), (float) (Math.sin(pitch)*Math.cos(yaw)));
		look = up.cross(right);
		
		Vector3f vec = new Vector3f();
        if (isKeyPressed(GLFW_KEY_UP) || isKeyPressed(GLFW_KEY_W)) {
            vec = vec.add(look);
        } 
        if (isKeyPressed(GLFW_KEY_DOWN) || isKeyPressed(GLFW_KEY_S)) {
        	vec = vec.add(look.negate());
        }
        if (isKeyPressed(GLFW_KEY_LEFT) || isKeyPressed(GLFW_KEY_A)) {
        	vec = vec.add(right.negate());
        }
        if (isKeyPressed(GLFW_KEY_RIGHT) || isKeyPressed(GLFW_KEY_D)) {
        	vec = vec.add(right);
        }
        if (isKeyPressed(GLFW_KEY_SPACE)) {
        	vec = vec.add(up);
        } 
        if (isKeyPressed(GLFW_KEY_LEFT_ALT)) {
        	vec = vec.add(up.negate());
        }
        
        position = position.add(vec.scale(SPEED * (float)delta));
	}
	
	public Matrix4f getViewMatrix() {
		return Matrix4f.viewMatrix(right, up, look, position);
	}
	
	public Matrix4f getViewMatrix(Vector3f position, float yaw, float pitch) {
		
		this.position = position;
		this.yaw = yaw;
		this.pitch = pitch;
		
		right = new Vector3f((float) Math.cos(yaw), 0, (float) -Math.sin(yaw));
		up = new Vector3f((float) (Math.sin(pitch)*Math.sin(yaw)), (float) Math.cos(pitch), (float) (Math.sin(pitch)*Math.cos(yaw)));
		look = up.cross(right);
		
		return Matrix4f.viewMatrix(right, up, look, position);
	}
	
public Matrix4f getViewMatrix(Settings setting) {
		
		switch (setting) {
		case DRONE_TOP_DOWN_CAM: 
			position = new Vector3f(hop.x, hop.y + camDistance, hop.z - getWidthAtCamDistance()/4);
			yaw = (float) Math.PI/2;
			pitch = (float) -Math.PI/2;
			break;
		case DRONE_SIDE_CAM: 
			position = new Vector3f(hop.x + camDistance, hop.y, hop.z - getWidthAtCamDistance()/4);
			yaw = (float) Math.PI/2;	
			pitch = 0;
			break;
		default: position = new Vector3f();
			break;
		}
		
		right = new Vector3f((float) Math.cos(yaw), 0, (float) -Math.sin(yaw));
		up = new Vector3f((float) (Math.sin(pitch)*Math.sin(yaw)), (float) Math.cos(pitch), (float) (Math.sin(pitch)*Math.cos(yaw)));
		look = up.cross(right);
		
		return Matrix4f.viewMatrix(right, up, look, position);
	}

	public Vector3f getPosition() {
		return this.position;
	}
	
	static boolean isKeyPressed(int keyCode) {
		return glfwGetKey(GLFW.glfwGetCurrentContext(), keyCode) == GLFW_PRESS;
	}

	public void nextPosition(Vector3f position) {
		this.hop = new Vector3f(position.x, position.y, roundPos(position.z, 0.5f));
		Tile.setPosOffset(new Vector3f(roundPos(position.x, 0.1f), 0, roundPos(position.z, 0.1f)));
	}
	
	public float roundPos(float dir) {
		return roundPos(dir, 1);
	}
	
	public float roundPos(float dir, float scale) {
		return getWidthAtCamDistance()*scale * (int) (dir/(scale*getWidthAtCamDistance()));
	}
	
	public float getWidthAtCamDistance() {
		return (float) (camDistance * Math.tan(Math.PI/3));
	}
}
