package tests;

import internal.Helper.Array2D;
import org.junit.Before;
import org.junit.Test;

/**
 * Created by Martijn on 26/10/2017.
 */
public class Array2Dtests {

    public static Array2D<Integer> array;

    @Before
    public final void setupMutableFixture(){
        Integer[] intArray = {0,1,2,3,4,5,6,7,8};
        array = new Array2D<>(intArray, 3,3);
    }

    @Test
    public final void testSlice(){
        System.out.print(array.getSlice(1,3,0,1));
    }
}
