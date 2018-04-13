package AutopilotInterfaces;

public interface AutopilotInputs_v2 {
    byte[] getImage();
    float getX();
    float getY();
    float getZ();
    float getHeading();
    float getPitch();
    float getRoll();
    float getElapsedTime();
}
