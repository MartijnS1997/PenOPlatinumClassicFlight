package AutopilotInterfaces;

public interface Autopilot_v2 {
    AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs_v2 inputs);
    AutopilotOutputs timePassed(AutopilotInputs_v2 inputs);
    void setPath(Path path);
    void simulationEnded();
}