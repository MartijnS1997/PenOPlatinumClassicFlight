package internal.Autopilot;

/**
 * Class for creating AutoPilotInputs objects, through which the inputs to the AutopilotInterfaces will be passed
 * @author Anthony Rathe
 *
 */
public class AutoPilotInputs implements AutopilotInterfaces.AutopilotInputs{
	public AutoPilotInputs(byte[] image, float xPosition, float yPosition, float zPosition, float heading, float pitch, float roll, float elapsedTime) {
		this.image = image;
		this.xPostion = xPosition;
		this.yPosition = yPosition;
		this.zPosition = zPosition;
		this.heading = heading;
		this.pitch = pitch;
		this.roll = roll;
		this.elapsedTime = elapsedTime;
	}
	
	/**
	 * Returns the image as a byte array.
	 */
	public byte[] getImage() {
		return this.image;
	}
	
	/**
	 * Returns the x-co�rdinate of the drone
	 */
	public float getX() {
		return this.xPostion;
	}
	
	/**
	 * Returns the y-co�rdinate of the drone
	 */
	public float getY() {
		return this.yPosition;
	}
	
	/**
	 * Returns the z-co�rdinate of the drone
	 */
	public float getZ() {
		return this.zPosition;
	}
	
	/**
	 * Returns the heading of the drone
	 */
	public float getHeading() {
		return this.heading;
	}
	
	/**
	 * Returns the pitch of the drone
	 */
	public float getPitch() {
		return this.pitch;
	}
	
	/**
	 * Returns the roll of the drone
	 */
	public float getRoll() {
		return this.roll;
	}
	
	/**
	 * Returns the elapsed time
	 */
	public float getElapsedTime() {
		return this.elapsedTime;
	}
	
	
	private final byte[] image;
	private final float xPostion;
	private final float yPosition;
	private final float zPosition;
	private final float heading;
	private final float pitch;
	private final float roll;
	private final float elapsedTime;
}
