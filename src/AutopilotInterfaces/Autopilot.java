package AutopilotInterfaces;

import java.io.IOException;

public interface Autopilot {
    
	AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs inputs) throws IOException;
	
    AutopilotOutputs timePassed(AutopilotInputs inputs) throws IOException;
    void simulationEnded();
}
