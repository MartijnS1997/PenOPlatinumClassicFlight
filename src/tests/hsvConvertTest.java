package tests;



import internal.Helper.HSVconverter;

public class hsvConvertTest {

	public static void main(String[] args) {
		for (float[] color:all){
			float[] temp = (HSVconverter.HSVtoRGB2(color[0], color[1], color[2]));
			System.out.print(" r = " + Float.toString(temp[0]*255) + " g = " + Float.toString(temp[1]*255) + 
					" b = " + Float.toString(temp[2]*255)+'\n');
		}
		
//		for (float[] color:rall){
//			float[] temp = (HSVconverter.RGBtoHSV(color[0]/255, color[1]/255, color[2]/255));
//			System.out.print(" h = " + Float.toString(temp[0]) + " v = " + Float.toString(temp[1]) + 
//					" s = " + Float.toString(temp[2])+'\n');
//		}

	}

	
	private static float[] RED = {360,1,1}; //255 0 0
	private static float[] BLUE = {240,1,1}; // 0 0 255
	private static float[] GREEN = {120,1,1}; //0 255 0 
	private static float[] PURPLE = {300,1,1}; //255 0 255
	private static float[] YELLOW = {60,1,1}; //255 255 0
	private static float[] CYAN = {180,1,1}; //0 255 255
	private static float[] rand1 = {147,0.59f,1}; //104.55 255 172.25
	private static float[] rand2 = {80,0.55f,1}; //208.25 255 114.75
	
	private static float[][] all = {RED, BLUE, GREEN, PURPLE, YELLOW, CYAN, rand1, rand2};
	
	
	
	private static float[] rRED = {255, 0, 0}; 
	private static float[] rBLUE = {0 ,0 ,255}; 
	private static float[] rGREEN = {0, 255, 0 }; 
	private static float[] rPURPLE = {255, 0 ,255}; 
	private static float[] rYELLOW = {255 ,255 ,0}; 
	private static float[] rCYAN = {0, 255, 255}; 
	private static float[] rrand1 = {104.55f, 255 ,172.25f}; 
	private static float[] rrand2 = {208.25f, 255 ,114.75f}; 
	
	private static float[][] rall = {rRED, rBLUE, rGREEN, rPURPLE, rYELLOW, rCYAN, rrand1, rrand2};
	
}
