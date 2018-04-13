package gui;

import internal.Testbed.Airport;
import internal.Testbed.Block;
import internal.Testbed.Drone;
import internal.Testbed.Floor;
import internal.Testbed.World;
import internal.Testbed.WorldObject;
import math.Matrix3f;
import math.Matrix4f;
import math.Vector3f;

import org.lwjgl.BufferUtils;
import org.lwjgl.glfw.*;
import org.lwjgl.opengl.*;
import org.lwjgl.system.*;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.*;

import static org.lwjgl.glfw.Callbacks.*;
import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL30.GL_INVALID_FRAMEBUFFER_OPERATION;
import static org.lwjgl.system.MemoryStack.*;
import static org.lwjgl.system.MemoryUtil.*;

public class Window {

	// The window handle
	private long windowHandle;
	
	GLCapabilities capabilities;
	
	private int WIDTH;
	private int HEIGHT;

	private Input input;
	private ShaderProgram program;
	
    private float FOV;
	private static final float NEAR = 0.01f;
	private static final float FAR = 1000.f;
	
	private String title;

	private Matrix4f viewMatrix;

	private Matrix4f projectionMatrix;

	private boolean terminated = false;

	private Settings setting;
	
	private Vector3f cameraposition;

	/**
	 * Creates a window.
     * Initializes the OpenGL state. Creating programs and sets 
     * appropriate state. 
	 * @param visible 
     */
	public Window(int width, int height, float xOffset, float yOffset, String title, Vector3f color, boolean visible) {
		WIDTH = width;
		HEIGHT = height;
		this.title = title;
		
		// Create the window
		windowHandle = glfwCreateWindow(width, height, title, NULL, NULL);
		if ( getHandler() == NULL )
			throw new RuntimeException("Failed to create the GLFW window");

		// Setup a key callback. It will be called every time a key is pressed, repeated or released.
		glfwSetKeyCallback(getHandler(), (handle, key, scancode, action, mods) -> {
			if ( key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE )
				glfwSetWindowShouldClose(handle, true); // We will detect this in the rendering loop
		});
		
		glfwSetInputMode(getHandler(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);

		// Get the thread stack and push a new frame
		try ( MemoryStack stack = stackPush() ) {
			IntBuffer pWidth = stack.mallocInt(1); // int*
			IntBuffer pHeight = stack.mallocInt(1); // int*

			// Get the window size passed to glfwCreateWindow
			glfwGetWindowSize(getHandler(), pWidth, pHeight);

			// Get the resolution of the primary monitor
			GLFWVidMode vidmode = glfwGetVideoMode(glfwGetPrimaryMonitor());

			// Center the window
			glfwSetWindowPos(
					getHandler(),
				(int) ((vidmode.width() - pWidth.get(0)) * xOffset),
				(int) ((vidmode.height() - pHeight.get(0)) * yOffset)
			);
		} // the stack frame is popped automatically

		// Make the OpenGL context current
		glfwMakeContextCurrent(getHandler());
				
		// Enable v-sync
		glfwSwapInterval(1);

		// Make the window visible
		if (visible)
			glfwShowWindow(getHandler());
		
		if (visible)
			this.FOV = (float) Math.toRadians(60.0f);
		else
			this.FOV = (float) Math.toRadians(120.0f);
		
		// This line is critical for LWJGL's interoperation with GLFW's
		// OpenGL context, or any context that is managed externally.
		// LWJGL detects the context that is current in the current thread,
		// creates the GLCapabilities instance and makes the OpenGL
		// bindings available for use.
		capabilities = GL.createCapabilities();

		// Set the clear color
		glClearColor(color.x, color.y, color.z, 1.0f);
        
		glfwMakeContextCurrent(NULL);
	}
	
	public void initWindow(Settings setting) {
		this.setting = setting;
//		if (setting == Settings.DRONE_CAM)
//			this.FOV = (float) Math.toRadians(120.0f);
//		else
//			this.FOV = (float) Math.toRadians(60.0f);
		
		glfwMakeContextCurrent(getHandler());
		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LESS);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		
		program = new ShaderProgram(false, "resources/3dWorld.vert", "resources/3dWorld.frag");	

        program.init();
        
        input = new Input(setting);
        
        try {
			program.createUniform("projectionMatrix");
			program.createUniform("viewMatrix");
	        program.createUniform("modelMatrix");
		} catch (Exception e) {
			e.printStackTrace();
		}
        
        float ratio;
		try (MemoryStack stack = MemoryStack.stackPush()) {
			long window = GLFW.glfwGetCurrentContext();
			IntBuffer width = stack.mallocInt(1);
			IntBuffer height = stack.mallocInt(1);
			GLFW.glfwGetFramebufferSize(window, width, height);
			ratio = (float) width.get() / (float) height.get();
		}
        projectionMatrix = Matrix4f.perspective(FOV, ratio, NEAR, FAR);
        
		if (setting == Settings.INDEPENDENT_CAM)
			glfwSetInputMode(getHandler(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		glfwMakeContextCurrent(NULL);
	}

	public void render(World world) {
		renderFrame(world);

		// Return false if the user has attempted to close
		// the window or has pressed the ESCAPE key.
		if (glfwWindowShouldClose(getHandler())) {
			terminate(world);
		}
//		checkError();
	}
	
	private void renderFrame(World world) {
		GL.setCapabilities(capabilities);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear the buffers

		updateMatrices(world);
	
		program.bind();
        program.setUniform("projectionMatrix", projectionMatrix);
        program.setUniform("viewMatrix", viewMatrix);
        
        float viewingDistance = 500;

        for (WorldObject object: world.getObjectSet()) {
        	if (object.getClass() == Block.class) {
        		for (GraphicsObject cube: object.getAssociatedGraphicsObjects()) {
        			program.setUniform("modelMatrix", getModelMatrix(((Cube) cube).getRelPos(), cube.getSize()));
//        			if (cube.getPos().subtract(this.cameraposition).length() < viewingDistance)
					cube.render();
        		}
        	}
        	else if(object.getClass() == Drone.class) {
        		for (GraphicsObject polygon: object.getAssociatedGraphicsObjects()) {
        			program.setUniform("modelMatrix", getModelMatrix(((Drone) object).getOrientation().convertToVector3f(), ((Polygon) polygon).getRelPos(), polygon.getSize()));
        			input.nextPosition(polygon.getPos());
        			polygon.render();
        		}
        	}
        	else if(object.getClass() == Floor.class) {
        		for (GraphicsObject tile: object.getAssociatedGraphicsObjects()) {
        			program.setUniform("modelMatrix", getModelMatrix(((Tile) tile).getPosWithOffset(), tile.getSize()));
        			if (((Tile) tile).getPosWithOffset().subtract(this.cameraposition).length() < viewingDistance)
						tile.render();
        		}
        	}
        	else if(object.getClass() == Airport.class) {
        		for (GraphicsObject tile: object.getAssociatedGraphicsObjects()) {
        			program.setUniform("modelMatrix", getModelMatrix(tile.getPos(), tile.getSize()));
					tile.render();
        		}
        	}
    	}
		
		program.unbind();
		
		glfwSwapBuffers(getHandler()); // swap the  buffers
	}
	
	// Free the window callbacks and destroy the window
	public void terminate(World world) {
		program.delete();
		
		glfwFreeCallbacks(getHandler());
		glfwDestroyWindow(getHandler());
		
		/**
	     * Releases in use OpenGL resources.
	     */
		for (WorldObject object: world.getObjectSet()) {
			for (GraphicsObject graphicsObject: object.getAssociatedGraphicsObjects())
				graphicsObject.delete();
    	}
		
		terminated = true;
	}
	
	public boolean isTerminated() {
		return terminated ;
	}

	public long getHandler() {
		return windowHandle;
	}
	
	public void updateMatrices(World world) {
		if (Input.isKeyPressed(GLFW_KEY_R))
			this.setting = Settings.DRONE_CAM;
		else if (Input.isKeyPressed(GLFW_KEY_T))
			this.setting = Settings.DRONE_TOP_DOWN_CAM;
		else if (Input.isKeyPressed(GLFW_KEY_Y))
			this.setting = Settings.DRONE_SIDE_CAM;
		else if (Input.isKeyPressed(GLFW_KEY_U))
			this.setting = Settings.DRONE_CHASE_CAM;
		else if (Input.isKeyPressed(GLFW_KEY_I))
			this.setting = Settings.INDEPENDENT_CAM;
		else if (Input.isKeyPressed(GLFW_KEY_TAB))
			this.setting = getSetting().next();
		
		if (getSetting() == Settings.INDEPENDENT_CAM)
			input.processInput();
		updateViewMatrix(setting, world);
		
		projectionMatrix = getProjectionMatrix();
	}
	
	public Matrix4f getModelMatrix(Vector3f position, Vector3f size) {
		return Matrix4f.translate(position.x, position.y, position.z).multiply(Matrix4f.scale(size));
	}
	
	public Matrix4f getModelMatrix(Vector3f orientation, Vector3f position, Vector3f size) {
		return Matrix4f.translate(position).multiply(Matrix4f.rotate(orientation)).multiply(Matrix4f.scale(size));
	}
	
	public void updateViewMatrix(World world) {
		switch (getSetting()) {
		case DRONE_CAM: 
			this.viewMatrix = getView(world, new Vector3f(1f, 1f, 1f), new Vector3f(1f, 1f, 1f).scale(3f));
			break;
		case DRONE_CHASE_CAM: 
			this.viewMatrix = getView(world, new Vector3f(1f, 0f, 0f), new Vector3f(-1f, 0f, -1f).scale(10f));
			break;
		default: 
			this.viewMatrix = input.getViewMatrix(getSetting());
			this.cameraposition = input.getPosition();
			break;
		}
	}
	
	public void updateViewMatrix(Settings setting, World world) {
		switch (setting) {
		case DRONE_CAM: 
			this.viewMatrix = getView(world, new Vector3f(1f, 1f, 1f), new Vector3f(1f, 1f, 1f).scale(3f));
			break;
		case DRONE_CHASE_CAM: 
			this.viewMatrix = getView(world, new Vector3f(1f, 0f, 0f), new Vector3f(-1f, 0f, -1f).scale(13f));
			break;
		default: 
			this.viewMatrix = input.getViewMatrix(setting);
			this.cameraposition = input.getPosition();
			break;
		}
	}
	
	public Matrix4f getView(World world, Vector3f camOrientation, Vector3f camPosition) {   
		Vector3f orientation = new Vector3f();
		Vector3f dronePosition = new Vector3f();
        for (Drone drone: world.getDroneSet()) {
        	orientation = drone.getOrientation().convertToVector3f().scale(camOrientation);
        	dronePosition = drone.getPosition().convertToVector3f();
        }
        
        Matrix3f transformationMatrix = Matrix3f.transformationMatrix(orientation).transpose();
        
        Vector3f right = transformationMatrix.multiply(new Vector3f(1,0,0));
        Vector3f up = transformationMatrix.multiply(new Vector3f(0,1,0));
        Vector3f look = transformationMatrix.multiply(new Vector3f(0,0,-1));
        
        
        Vector3f position = dronePosition.add(look.scale(camPosition));
        this.cameraposition = position;
		return Matrix4f.viewMatrix(right, up, look, position);
	}
	
	public Matrix4f getProjectionMatrix() {
        float ratio;
		try (MemoryStack stack = MemoryStack.stackPush()) {
			long window = GLFW.glfwGetCurrentContext();
			IntBuffer width = stack.mallocInt(1);
			IntBuffer height = stack.mallocInt(1);
			GLFW.glfwGetFramebufferSize(window, width, height);
			ratio = (float) width.get() / (float) height.get();
		}
        return Matrix4f.perspective(FOV, ratio, NEAR, FAR);
	}
	
	public String getTitle() {
		return this.title;
	}
	
	public void setSetting(Settings setting) {
		this.setting = setting;
	}
	
	public Settings getSetting() {
		return this.setting;
	}
	
	/**
     * Utility method which checks for an OpenGL error, throwing an exception if
     * one is found.
     */
    public static void checkError() {
        int err = glGetError();
        switch(err) {
            case GL_NO_ERROR: return;
            case GL_INVALID_OPERATION: throw new RuntimeException("Invalid Operation");
            case GL_INVALID_ENUM: throw new RuntimeException("Invalid Enum");
            case GL_INVALID_VALUE: throw new RuntimeException("Invalid Value");
            case GL_INVALID_FRAMEBUFFER_OPERATION: throw new RuntimeException("Invalid Framebuffer Operation");
            case GL_OUT_OF_MEMORY: throw new RuntimeException("Out of Memory");
        }
    }

	/**
	 * Reads the pixels on the screen and returns them as an array of bytes
	 * @return an byte array containing the image
	 * @author Martijn
	 */
	public byte[] getCameraView() throws IOException {
		glfwMakeContextCurrent(getHandler());

		GL11.glReadBuffer(GL11.GL_FRONT);
		int bpp = 4; // Assuming a 32-bit display with a byte each for red, green, blue, and alpha.
		ByteBuffer buffer = BufferUtils.createByteBuffer(WIDTH * HEIGHT * bpp);
		GL11.glReadPixels(0, 0, WIDTH, HEIGHT, GL11.GL_RGBA, GL11.GL_UNSIGNED_BYTE, buffer );
		byte[] imageByteArray = new byte[WIDTH*HEIGHT*bpp];
		buffer.get(imageByteArray);

		BufferedImage image = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_INT_RGB);

		
		/*
		String format = "PNG";
    	File file = new File("orginalTestImage.png");
    	BufferedImage testImage = new BufferedImage(WIDTH, HEIGHT, 1);
    	
		for(int x = 0; x < WIDTH; x++)
		{
			for(int y = 0; y < HEIGHT; y++)
			{
				//System.out.println(x+" : "+y);
				int i = (x + (WIDTH * y)) * bpp;
				//System.out.println("byte: "+imageByteArray[i]);
				int r = (imageByteArray[i])& 0xFF;
				int g = (imageByteArray[i + 1]) & 0xFF;
				int b =(imageByteArray[i+2])& 0xFF;
				
				image.setRGB(x, HEIGHT - (y + 1), (0xFF << 24) | (r << 16) | (g << 8) | b);
			}
		}
		try {
			ImageIO.write(testImage, format, file);
		} catch (IOException e) { e.printStackTrace(); }
		*/
		
		/*
		step += 1;
		String diagnosis = "";
		int prevRow = 0;
		if(step >= 100 && step <= 110) {
			for (int i = 0; i < WIDTH * HEIGHT * bpp; i += 4) {
				if (imageByteArray[i] != -1 || imageByteArray[i + 1] != -1 || imageByteArray[i + 1] != -1) {
					if (prevRow != i / WIDTH)
						diagnosis += "\n\n";
					diagnosis += imageByteArray[i] + "; " + imageByteArray[i + 1] + "; " + imageByteArray[i + 2] + "   ";
				}

				prevRow = i / WIDTH;
			}

			PrintWriter printerOutput = new PrintWriter("frame" + (step-100) + ".txt");
			printerOutput.print(diagnosis);
			printerOutput.close();
		}

		System.out.println(imageByteArray[0] + ";" + imageByteArray[1] +";" +  imageByteArray[2] + ";" + imageByteArray[3]);*/

		byte[] rescaledArray = rescale(imageByteArray, WIDTH, CAMERA_WIDTH, HEIGHT, CAMERA_HEIGHT, bpp);
		return rescaledArray;
	}

	/**
	 * Rescales the given image to an image of size (newNbRows*newNbColumns)
	 * @param newNbRows the nb of rows the rescaled image contains
	 * @param newNbColumns the nb of columns the rescaled image contains
	 * @return a new byte array containing the rescaled image
	 * note: algorithm is modified version from the one used in:
	 * http://tech-algorithm.com/articles/nearest-neighbor-image-scaling/
	 */
	public static byte[] rescale(byte[] imageArray, int oldNbRows, int newNbRows,int oldNbColumns, int newNbColumns, int bpp){


		byte[] temp = new byte[newNbRows*newNbColumns*3];
		float xRatio = oldNbColumns/(float)newNbColumns;
		float yRatio = oldNbRows/(float)newNbRows;
		float px, py;

		for(int i = 0; i!= newNbRows; i++){
			for(int j = 0; j!= newNbColumns; j++){
				px = (float) Math.floor(j*xRatio);
				py = (float) Math.floor(i*yRatio);

				for(int k = 0; k != 3; k++) {
					temp[((i * newNbColumns) + j) * 3 + k] = (imageArray[((int) (py * oldNbColumns + px)) * bpp+ k]);
				}

			}
		}

		return temp;
	}



	private static int CAMERA_WIDTH = 200;
	private static int CAMERA_HEIGHT = 200;
	private static boolean firstRun = true;
	private int step = 0;

}

/*
		step += 1;
		String diagnosis = "";
		int prevRow = 0;
		if(step >= 100 && step <= 110) {
			for (int i = 0; i < WIDTH * HEIGHT * bpp; i += 4) {
				if (imageByteArray[i] != -1 || imageByteArray[i + 1] != -1 || imageByteArray[i + 1] != -1) {
					if (prevRow != i / WIDTH)
						diagnosis += "\n\n";
					diagnosis += imageByteArray[i] + "; " + imageByteArray[i + 1] + "; " + imageByteArray[i + 2] + "   ";
				}

				prevRow = i / WIDTH;
			}

			PrintWriter printerOutput = new PrintWriter("frame" + (step-100) + ".txt");
			printerOutput.print(diagnosis);
			printerOutput.close();
		}

		System.out.println(imageByteArray[0] + ";" + imageByteArray[1] +";" +  imageByteArray[2] + ";" + imageByteArray[3]);*/

//		ByteBuffer buffer = BufferUtils.createByteBuffer(HEIGHT *WIDTH*4);
//		//the array used for storing the pixels
//		byte[] pixelArray = new byte[HEIGHT *WIDTH*4];
//		int[] imageData = new int[CAMERA_WIDTH * CAMERA_HEIGHT];
//		GL11.glPixelStorei(GL11.GL_PACK_ALIGNMENT, 1);
//		GL11.glPixelStorei(GL11.GL_UNPACK_ALIGNMENT, 1);
//		//prepare the buffer for filling
//		buffer.clear();
//		//read the pixels on screen in the given window
//		GL11.glReadPixels(0,0, WIDTH, HEIGHT, GL11.GL_RGBA, GL11.GL_UNSIGNED_BYTE, buffer);
//
//
//		//prepare buffer for reading
//		//transfer al the buffered data to the array
//		buffer.get(pixelArray);
//
//		//rescale the image to the appropriate size
//		byte[] rescaledPixelArray = rescale(pixelArray, WIDTH, CAMERA_WIDTH, HEIGHT, CAMERA_HEIGHT);
//		if(firstRun){
//			for (int l = 0; l < CAMERA_WIDTH; l++) {
//				for (int i1 = 0; i1 < CAMERA_HEIGHT; i1++) {
//					int j1 = l + (CAMERA_HEIGHT - i1 - 1) * CAMERA_WIDTH;
//					int k1 = rescaledPixelArray[j1 * 3 + 0] & 0xff;
//					int l1 = rescaledPixelArray[j1 * 3 + 1] & 0xff;
//					int i2 = rescaledPixelArray[j1 * 3 + 2] & 0xff;
//					int j2 = 0xff000000 | k1 << 16 | l1 << 8 | i2;
//					imageData[l + i1 * CAMERA_WIDTH] = j2;
//				}
//			}
//			BufferedImage bufferedImage = new BufferedImage(CAMERA_WIDTH, CAMERA_HEIGHT, 1);
//			bufferedImage.setRGB(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT, imageData, 0, CAMERA_WIDTH);
//			int index = 0;
//			for(byte bytes: pixelArray){
//				System.out.println(bytes);
//				if(index == 200){
//					break;
//				}
//				index++;
//
//			}
//
//			firstRun = false;
//		}
//
//
//
//
//		return rescaledPixelArray;

//		String format = "PNG";
//		File file = new File("image.png");
//
//		for(int x = 0; x < WIDTH; x++)
//		{
//			for(int y = 0; y < HEIGHT; y++)
//			{
//				int i = (x + (WIDTH * y)) * bpp;
//				int r = (imageByteArray[i])& 0xFF;
//				int g = (imageByteArray[i + 1]) & 0xFF;
//				int b =(imageByteArray[i+2])& 0xFF;
//				image.setRGB(x, HEIGHT - (y + 1), (0xFF << 24) | (r << 16) | (g << 8) | b);
//			}
//		}
//
//		try {
//			ImageIO.write(image, format, file);
//		} catch (IOException e) { e.printStackTrace(); }
