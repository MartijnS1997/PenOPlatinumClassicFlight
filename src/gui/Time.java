package gui;


import static org.lwjgl.glfw.GLFW.glfwGetTime;

public class Time {
	
	// the starting time
	private static double firstTime;
	
	// the time of the previous update
	private static double lastTime;
	
	// updates per second
	private static int ups;
	
    // counter for the ups calculation.
    private static int upsCount;
    
    // time for the ups calculation.
    private static double timeCount;
	
	// time passed since last update
	private static double delta;
	

	public static void initTime() {
		lastTime = getTime();
		firstTime = getTime();
	}
	
	public static void update() {
		double time = getTime();
        delta = (time - lastTime);
        lastTime = time;
        
        timeCount += delta;
        if (timeCount > 1000f) {
            ups = upsCount;
            upsCount = 0;

            timeCount -= 1000f;
        }
    }

	public static double getTime() {
		return 1000. * glfwGetTime();
	}
	
	public static double getLastTime() {
		return lastTime;
	}
	
	public static double timeSinceLastUpdate() {
		return getTime() - getLastTime();
	}
	
	public static double getDelta() {
		return delta;
	}
	
	// time passed since this timer was made
	public static double getTimePassed() {
		return getTime() - firstTime;
	}
	
	public static int getUps() {
		if (ups > 0) {
			return ups;
		} else {
			return upsCount;
		}
	}
}
