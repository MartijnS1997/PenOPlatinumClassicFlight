package internal.Autopilot;

import java.util.ArrayList;

import internal.Helper.Vector;

public class Path implements AutopilotInterfaces.Path {

	/**
	 * Path that has to be followed by the drone in the given order
	 * @param x All x positions of all cubes in the world
	 * @param y All y positions of all cubes in the world
	 * @param z All z positions of all cubes in the world
	 */
	public Path(float[] x, float[] y, float[] z){
		this.x = x;
		this.y = y;
		this.z = z;
	}



	/**
	 * Converts a list of cube positions to the arrays of x,y,z coordinates (in the same order)
	 * Updates the x,y,z variables to contain their new values
	 * @param cubePos A list of cube positions
	 */
	public void convertCubeLocsToPath(ArrayList<Vector> cubePos){
		int n = cubePos.size();

		ArrayList<Float> allX = new ArrayList<Float>();
		ArrayList<Float> allY = new ArrayList<Float>();
		ArrayList<Float> allZ = new ArrayList<Float>();

		for(int i = 0; i < n; i++){
			allX.add(cubePos.get(i).getxValue());
			allY.add(cubePos.get(i).getyValue());
			allZ.add(cubePos.get(i).getzValue());
		}

		Object[] xx = allX.toArray();
		Object[] yy = allY.toArray();
		Object[] zz = allZ.toArray();

		int size = xx.length;

		float[] xxx = new float[size];
		float[] yyy = new float[size];
		float[] zzz = new float[size];

		int j = 0;
		for(Object x : xx){
			xxx[j++] = (float) x;
		}

		int k = 0;
		for(Object y : yy){
			yyy[k++] = (float) y;
		}

		int l = 0;
		for(Object z : zz){
			zzz[l++] = (float) z;
		}


		setX(xxx);
		setY(yyy);
		setZ(zzz);

	}



	/**
	 * Getters and setters for the x,y,z arrays of the cube coordinates
	 */

	@Override
	public float[] getX() {
		return x;
	}

	@Override
	public float[] getY() {
		return y;
	}

	@Override
	public float[] getZ() {
		return z;
	}



	public void setX(float[] x) {
		this.x = x;
	}


	public void setY(float[] y) {
		this.y = y;
	}


	public void setZ(float[] z) {
		this.z = z;
	}


	/**
	 * Arrays of coordinates of cube positions
	 */
	private float[] x;
	private float[] y;
	private float[] z;
}
