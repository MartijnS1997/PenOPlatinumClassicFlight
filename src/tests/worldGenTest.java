package tests;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import internal.Testbed.WorldGenerator;

public class worldGenTest {

    private static int nbOfBlocks = 3;
    private static WorldGenerator wg = new WorldGenerator(nbOfBlocks);

    public static void main(String[] args) {
        wg.testFunction(wg.allPositionsGenerator());
    }





//
//	private static int nbOfBlocks = 100000;
//	private static WorldGenerator wg = new WorldGenerator(nbOfBlocks);
//
//
//	public static void main(String[] args) {
//		ArrayList<Vector> poss = generatePositions(nbOfBlocks);
//
//		for (int i = 1; i <= 3; i++){
//			ArrayList<ArrayList<Float>> xx = checkPositions(poss, nbOfBlocks,i);
//		}
//	}
//
//
//
//	public static ArrayList<Vector> generatePositions(int n){
//		ArrayList<Vector> pos = new ArrayList<Vector>();
//		for (int i = 0; i < n; i++){
//			Vector temp = wg.positionGenerator();
//			pos.add(temp);
//		}
//		return pos;
//	}
//
//	public static ArrayList<ArrayList<Float>> checkPositions(ArrayList<Vector> positions, int nb, int dir){
//		ArrayList<Float> stdDev1 = new ArrayList<>();
//		ArrayList<Float> stdDev2 = new ArrayList<>();
//		ArrayList<Float> stdDev3 = new ArrayList<>();
//
//		String direct;
//
//		if (dir == 1)
//			direct = "x";
//		else if (dir == 2)
//			direct = "y";
//		else if (dir == 3)
//			direct = "z";
//		else
//			direct = "ERROR";
//
//		for (int i = 0; i < nb; i++){
//			if (dir == 1){
//				if ((positions.get(i).getxValue() <= wg.getStdDevX() + wg.getXMean()) &&
//						(positions.get(i).getxValue() >= -wg.getStdDevX() + wg.getXMean())){
//					stdDev1.add(positions.get(i).getxValue());
//				}
//				if ((positions.get(i).getxValue() <= wg.getStdDevX() * 2 + wg.getXMean()) &&
//						(positions.get(i).getxValue() >= -2 * wg.getStdDevX() + wg.getXMean())){
//					stdDev2.add(positions.get(i).getxValue());
//				}
//				if ((positions.get(i).getxValue() <= wg.getStdDevX() * 3 + wg.getXMean()) &&
//						(positions.get(i).getxValue() >= -3 * wg.getStdDevX() + wg.getXMean())){
//					stdDev3.add(positions.get(i).getxValue());
//				}
//			}
//			if (dir == 2){
//				if ((positions.get(i).getyValue() <= wg.getStdDevY() + wg.getYMean()) &&
//						(positions.get(i).getyValue() >= -wg.getStdDevY() + wg.getYMean())){
//					stdDev1.add(positions.get(i).getyValue());
//				}
//				if ((positions.get(i).getyValue() <= wg.getStdDevY() * 2 + wg.getYMean()) &&
//						(positions.get(i).getyValue() >= -2 * wg.getStdDevY() + wg.getYMean())){
//					stdDev2.add(positions.get(i).getyValue());
//				}
//				if ((positions.get(i).getyValue() <= wg.getStdDevY() * 3 + wg.getYMean()) &&
//						(positions.get(i).getyValue() >= -3 * wg.getStdDevY() + wg.getYMean())){
//					stdDev3.add(positions.get(i).getyValue());
//				}
//			}
//			if (dir == 3){
//				if ((positions.get(i).getzValue() <= wg.getStdDevZ() + wg.getZMean()) &&
//						(positions.get(i).getzValue() >= -wg.getStdDevZ() + wg.getZMean())){
//					stdDev1.add(positions.get(i).getzValue());
//				}
//				if ((positions.get(i).getzValue() <= wg.getStdDevZ() * 2 + wg.getZMean()) &&
//						(positions.get(i).getzValue() >= -2 * wg.getStdDevZ() + wg.getZMean())){
//					stdDev2.add(positions.get(i).getzValue());
//				}
//				if ((positions.get(i).getzValue() <= wg.getStdDevZ() * 3 + wg.getZMean()) &&
//						(positions.get(i).getzValue() >= -3 * wg.getStdDevZ() + wg.getZMean())){
//					stdDev3.add(positions.get(i).getzValue());
//				}
//			}
//
//
//		}
//
//		float pc1 = (float) stdDev1.size() / nb * 100;
//		float pc2 = (float) stdDev2.size() / nb * 100;
//		float pc3 = (float) stdDev3.size() / nb * 100;
//
//		System.out.println(pc1+"% of the "+direct+"-positions have a value within the 1st Gaussian interval");
//		System.out.println(pc2+"% of the "+direct+"-positions have a value within the 2nd Gaussian interval");
//		System.out.println(pc3+"% of the "+direct+"-positions have a value within the 3rd Gaussian interval");
//
//		ArrayList<ArrayList<Float>> res = new ArrayList<ArrayList<Float>>(3);
//		res.add(stdDev1);
//		res.add(stdDev2);
//		res.add(stdDev3);
//		return res;
//	}
//
//
}
