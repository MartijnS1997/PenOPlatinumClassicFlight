package gui;

public enum Settings {
	DRONE_CAM,
	DRONE_CHASE_CAM,
	DRONE_TOP_DOWN_CAM,
	DRONE_SIDE_CAM,
	INDEPENDENT_CAM;
	
	private static Settings[] vals = values();
    public Settings next()
    {
    	if (this.ordinal() == vals.length-3)
    		return vals[0];
        return vals[(this.ordinal()+1) % vals.length];
    }
}
