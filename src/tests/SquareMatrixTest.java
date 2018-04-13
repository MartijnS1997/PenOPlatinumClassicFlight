package tests;

import internal.Helper.SquareMatrix;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.assertEquals;

/**
 * Created by Martijn on 14/10/2017.
 * @author Martijn Sauwens
 */
public class SquareMatrixTest {


    public static SquareMatrix matrix1, matrix2, matrix3, matrix4;
    public static float EPSILON = 10E-8f;
    public static int matrixSize = 9;

    @Before
    public void setupMutableFixture(){
        float[] matrixArray1 = new float[] {1.0f, 1.0f, 1.0f,
                                            0.0f, 0.0f, 0.0f,
                                            0.0f, 0.0f, 0.0f};
        float[] matrixArray2 = new float[] {0.0f, 0.0f, 0.0f,
                                            1.0f, 1.0f, 1.0f,
                                            0.0f, 0.0f, 0.0f};
        float[] matrixArray3 = new float[] {0.0f, 0.0f, 0.0f,
                                            0.0f, 0.0f, 0.0f,
                                            1.0f, 1.0f, 1.0f};
        float[] matrixArray4 = new float[] {0.0f, 0.0f, 1.0f,
                                            0.0f, 0.0f, 1.0f,
                                            0.0f, 0.0f, 1.0f};

        matrix1 = new SquareMatrix(matrixArray1);
        matrix2 = new SquareMatrix(matrixArray2);
        matrix3 = new SquareMatrix(matrixArray3);
        matrix4 = new SquareMatrix(matrixArray4);
    }

    @Test
    public final void testGetRow(){
        float[] rowArray1 = matrix1.getRow(0);
        float[] rowArray2 = matrix2.getRow(0);

        for(float f: rowArray1){
            assertEquals(f, 1.0f, EPSILON);
        }

        for(float f: rowArray2){
            assertEquals(f,0.0f, EPSILON);
        }
    }

    @Test
    public final void testGetColumn(){
        float[] columnArray1 = matrix4.getColumn(2);
        float[] columnArray2 = matrix4.getColumn(0);

        for(float f: columnArray1){
            assertEquals(f, 1.0f, EPSILON);
        }

        for(float f: columnArray2){
            assertEquals(f, 0.0f, EPSILON);
        }
    }

    @Test
    public final void testSum(){
        SquareMatrix testMat = matrix1.matrixSum(matrix2).matrixSum(matrix3);
        assertEquals(testMat, new SquareMatrix(new float[]{1.0f, 1.0f, 1.0f,1.0f, 1.0f, 1.0f,1.0f, 1.0f, 1.0f}));
    }

    @Test
    public final void testScalarMult(){
        SquareMatrix testMatrix = matrix1.scalarMult(2.5f);
        float[] controlArray = new float[] {2.5f, 2.5f, 2.5f,
                                            0.0f, 0.0f, 0.0f,
                                            0.0f, 0.0f, 0.0f};
        assertEquals(testMatrix, new SquareMatrix(controlArray));
    }

    @Test
    public final void testTranspose(){
        SquareMatrix testMatrix = matrix3.transpose();
        assertEquals(testMatrix, matrix4);
    }

    @Test
    public final void matrixMult(){
        SquareMatrix testMatrix = matrix1.matrixProduct(matrix2);
        assertEquals(testMatrix, matrix1);
    }
    /**
     * @author Jonathan 
     */
    @Test
    public final void getHeadingTransformMatTest(){
    	float angle = (float) Math.PI;
    	float cos = (float) Math.cos(angle);
    	float sin = (float) Math.sin(angle);
    	SquareMatrix a = SquareMatrix.getHeadingTransformMatrix((float)Math.PI);
    	float[] r = new float[] {cos,0.0f,sin,
    							0.0f,1.0f,0.0f,
    							-sin,0.0f,cos};
    	assertEquals(a, new SquareMatrix(r));
    	assertEquals(cos, -1.0f,0);
    	
    }
    
    /**
     * @author Jonathan 
     */
    @Test
    public final void getPitchTransformMatTest(){
    	float angle = (float) Math.PI;
    	float cos = (float) Math.cos(angle);
    	float sin = (float) Math.sin(angle);
    	SquareMatrix a = SquareMatrix.getPitchTransformMatrix((float)Math.PI);
    	float[] r = new float[] {1.0f,0.0f,0.0f,
    							0.0f,cos,-sin,
    							0.0f,sin,cos};
    	assertEquals(a, new SquareMatrix(r));    	
    }
    
    /**
     * @author Jonathan 
     */
    @Test
    public final void getRollTransformMatTest(){
    	float angle = (float) Math.PI;
    	float cos = (float) Math.cos(angle);
    	float sin = (float) Math.sin(angle);
    	SquareMatrix a = SquareMatrix.getRollTransformMatrix((float)Math.PI);
    	float[] r = new float[] {cos,-sin,0.0f,
    							sin,cos,0f,
    							0.0f,0f,1f};
    	assertEquals(a, new SquareMatrix(r));    	
    }
}
