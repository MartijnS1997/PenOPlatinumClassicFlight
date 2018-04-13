package internal.Autopilot;

import AutopilotInterfaces.*;
import AutopilotInterfaces.Path;

/**
 * Created by Martijn on 8/03/2018.
 */
public class Autopilot_v2Implementation implements Autopilot_v2 {
    @Override
    public AutopilotOutputs simulationStarted(AutopilotConfig config, AutopilotInputs_v2 inputs) {
        return null;
    }

    @Override
    public AutopilotOutputs timePassed(AutopilotInputs_v2 inputs) {
        return null;
    }

    @Override
    public void setPath(Path path) {

    }

    @Override
    public void simulationEnded() {

    }

}
