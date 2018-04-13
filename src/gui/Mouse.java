package gui;

import static org.lwjgl.glfw.GLFW.glfwGetCursorPos;

import java.nio.DoubleBuffer;

import org.lwjgl.BufferUtils;

public class Mouse {
	
	private float x;
	private float y;
	private float prevX;
	private float prevY;
	private long window;
	
	public Mouse(long window) {
		this.window = window;
		DoubleBuffer b1 = BufferUtils.createDoubleBuffer(1);
        DoubleBuffer b2 = BufferUtils.createDoubleBuffer(1);
        glfwGetCursorPos(window, b1, b2);
        prevX = (float) b1.get(0);
        prevY = -(float) b2.get(0);
        x = prevX;
        y = prevY;
	}
	
	public void update() {
		prevX = x;
		prevY = y;
		DoubleBuffer b1 = BufferUtils.createDoubleBuffer(1);
        DoubleBuffer b2 = BufferUtils.createDoubleBuffer(1);
        glfwGetCursorPos(window, b1, b2);
        x = (float) b1.get(0);
        y = -(float) b2.get(0);
	}
	
	public float x() {
		return this.x;
	}
	
	public float dx() {
		return this.x - this.prevX;
	}
	
	public float y() {
		return this.y;
	}
	
	public float dy() {
		return this.y - this.prevY;
	}
}
