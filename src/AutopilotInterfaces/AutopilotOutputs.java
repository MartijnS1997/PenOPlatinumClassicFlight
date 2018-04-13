package AutopilotInterfaces;

public interface AutopilotOutputs {
    float getThrust();
    float getLeftWingInclination();
    float getRightWingInclination();
    float getHorStabInclination();
    float getVerStabInclination();
    float getFrontBrakeForce();
    float getLeftBrakeForce();
    float getRightBrakeForce();
}
