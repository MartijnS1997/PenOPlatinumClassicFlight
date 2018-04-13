package tests;
import internal.Helper.HSVconverter;
import internal.Helper.Vector;
import org.junit.Test;

import org.junit.Before;

import java.util.Arrays;

import static org.junit.Assert.assertEquals;


public class VectorTest {
	
	public static Vector vector1, vector2, vector3;
	
	public final static float EPSILON = (float) 0.0001;
	
	@Before
	public void setUpMutableFixture(){
		vector1 = new Vector(1,0,0);
		vector2 = new Vector(0,1,0);
		vector3 = new Vector(3,4,5);
	}
	
	@Test
	public final void vectorSum(){
		Vector tempVector = vector1.vectorSum(vector2);
		assert(tempVector.equals(new Vector(1,1,0)));
	}
	
	@Test
	public final void vectorMult(){
		Vector tempVector = vector3.scalarMult(5);
		assert(tempVector.equals(new Vector(15,20,25)));
	}
	
	@Test
	public final void scalarProduct(){
		float scalar1 = vector1.scalarProduct(vector2);
		float scalar2 = vector1.scalarProduct(vector3);
		float scalar3 = vector2.scalarProduct(vector3);
		
		assert(scalar1 == 0);
		assert(scalar2 == vector3.getxValue());
		assert(scalar3 == vector3.getyValue());
	}
	
	@Test
	public final void getSize(){
		float size1 = vector1.getSize();
		float size2 = vector3.getSize(); // size = Math.sqrt(50)
		
		assert(size1 == 1);
		assertEquals(size2, Math.sqrt(50.0), EPSILON);
	}
	
	@Test
	public final void getNormalized(){
		Vector tempVector = vector3.normalizeVector();
		float tempVal = (float) Math.sqrt(50.0);
		
		assertEquals(tempVector.getxValue(), vector3.getxValue()/tempVal, EPSILON);
		assertEquals(tempVector.getyValue(), vector3.getyValue()/tempVal, EPSILON);
		assertEquals(tempVector.getzValue(), vector3.getzValue()/tempVal, EPSILON);
	}
	
	@Test
	public final void getAngle(){
		float angle = vector1.getAngleBetween(vector2);
		assertEquals(angle, Math.PI/2.0, EPSILON);
	}

	@Test
	public final void HSVtester(){

		float[] converted = HSVconverter.RGBtoHSV(1.0f, 1.0f, 0.0f);
		System.out.println(converted[0] + ", " + converted[1] + ", " + converted[2]);
	}

	@Test
	public final void HSVToRGBTester(){
		float[] converted = HSVconverter.RGBtoHSV(1.0f,0,0);
		System.out.println(Arrays.toString(converted));
		converted = HSVconverter.HSVtoRGB2(0,1.0f,0.6f);
		System.out.println(Arrays.toString(converted));
	}

}
