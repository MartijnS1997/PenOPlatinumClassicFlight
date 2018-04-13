package AutopilotInterfaces;

public interface AutopilotInputs {
    byte[] getImage();
    float getX();
    float getY();
    float getZ();
    float getHeading();
    float getPitch();
    float getRoll();
    float getElapsedTime();
}
