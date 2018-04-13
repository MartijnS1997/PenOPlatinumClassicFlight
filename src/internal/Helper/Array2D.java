package internal.Helper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A class of immutable 2D arrays
 * Created by Martijn on 15/10/2017.
 */
public class Array2D<T> {

    /**
     * Constructor for a 2D array
     * @param inputArray the 1D input array containing the elements of the 2d array
     * @param nbRows the number of rows of the 2D array
     * @param nbColumns the number of columns of the 2D array
     * @throws IllegalArgumentException
     */
    public Array2D(T[] inputArray, int nbRows, int nbColumns) throws IllegalArgumentException{
        
    	if(!isValid2DArray(inputArray, nbRows, nbColumns))
            throw new IllegalArgumentException(INVALID_SIZE);

        this.array2DArray = inputArray;
        this.nbRows = nbRows;
        this.nbColumns = nbColumns;
    }

    /**
     * Gets a slice of the 2D array for the given indices
     * @param startRow the start of the 2D array row
     * @param endRow the end of the 2D array row (exclusive)
     * @param startColumn the start of the 2D array column
     * @param endColumn the end of the 2D array column (exclusive)
     * @return a new 2D array containing a 2D slice of the provided 2D array
     */
    public Array2D<T> getSlice(int startRow, int endRow, int startColumn, int endColumn){
        ArrayList<T> tempArray = new ArrayList<>();
        for(int rowIndex = startRow; rowIndex != endRow; rowIndex++){
            T[] row = this.getRow(rowIndex);
            ArrayList<T> tempRow = new ArrayList<>(Arrays.asList(row));
            ArrayList<T> columnSlice = new ArrayList<>(tempRow.subList(startColumn, endColumn));
            tempArray.addAll(columnSlice);
        }

        T[] elemArray = (T[]) tempArray.toArray();
        return new Array2D<T>(elemArray, endRow-startRow,endColumn-startColumn );
    }


    /**
     * Selects a row from the 2D array
     * @param rowIndex the index of the row in the 2d array
     * @return an array containing the row at the given index
     * @throws IndexOutOfBoundsException
     */
    public T[] getRow(int rowIndex) throws IndexOutOfBoundsException{
        int nbColumns = this.getNbColumns();
        ArrayList<T> tempRow = new ArrayList<>(nbColumns);
        int base = rowIndex*nbColumns;
        for(int columnIndex = 0; columnIndex != nbColumns; columnIndex++){
            T elem = this.getElementAtIndex(base + columnIndex);
            tempRow.add(elem);
        }

        return (T[])tempRow.toArray();
    }

    /**
     * Selects a column from the 2D array
     * @param columnIndex the index of the desired column
     * @return an array containing the column at the given index
     * @throws IndexOutOfBoundsException
     */
    public T[] getColumn(int columnIndex) throws IndexOutOfBoundsException{
        int nbColumns = this.getNbColumns();
        int nbRows = this.getNbRows();
        ArrayList<T> tempColumn = new ArrayList<>(nbRows);
        for(int rowIndex = 0; rowIndex != nbRows; rowIndex++){
            T elem = this.getElementAtIndex(nbColumns*rowIndex + columnIndex);
            tempColumn.add(elem);
        }
        return (T[]) tempColumn.toArray();

    }

    /**
     * Gets the element at the given index in the 1d array containing the 2d array values
     * @param index the index of the required element
     * @return the element at the given index of the 1d array
     * @throws IndexOutOfBoundsException
     */
    public T getElementAtIndex(int index)throws IndexOutOfBoundsException{
        return this.getArray2DArray()[index];
    }

    /**
     * Gets the element at the given 2d Index in the 2d array
     * @param rowIndex the index of the row
     * @param columnIndex the index of the column
     * @return the value located at (rowIndex, columnIndex) in the 2D array
     */
    public T getElementAtIndex(int rowIndex, int columnIndex) throws IndexOutOfBoundsException{
        int trueRowIndex = rowIndex*this.getNbColumns();

        return this.getElementAtIndex(trueRowIndex + columnIndex);
    }

    /**
     * Getter for the number of rows of the 2D array
     * @return an integer containing the number of rows
     */
    public int getNbRows() {
        return this.nbRows;
    }

    /**
     * Getter for the number of columns of the 2D array
     * @return an integer containing the number of columns
     */
    public int getNbColumns() {
        return this.nbColumns;
    }

    /**
     * Getter for the 1d array version of the 2d array
     * @return an array of type T containing the 1d array
     */
    private T[] getArray2DArray(){
        return this.array2DArray;
    }

    /**
     * checks if the given array and dimensions can be converted in a 2D array of
     * size nbRows x nbColumns
     * @param array2DArray the array containing the 2D array elements
     * @param nbRows the number of rows in the array
     * @param nbColumns the number of columns in the  array
     * @return true if and only if the size of the array is equal to the product of
     *         the number of rows and the number of columns
     */
    private boolean isValid2DArray(T[] array2DArray, int nbRows, int nbColumns){
        int length = array2DArray.length;
        return length == nbColumns*nbRows;
    }

    @Override
    public String toString() {
        return "Array2D{" +
                "nbRows=" + nbRows +
                ", nbColumns=" + nbColumns +
                ", array2DArray=" + Arrays.toString(array2DArray) +
                '}';
    }

    /**
     * gets a list version of the array containing the 2D array list
     * @return  a list containing the 1D array containing the 2D array elements
     */
    public List<T> getArray2DList(){
        return Arrays.asList(this.getArray2DArray());
    }
    

    /*
    Instance Variables
     */

    /**
     * Variable containing the rows of the 2D array
     */
    private int nbRows;
    /**
     * Variable containing the columns of the 2D array
     */
    private int nbColumns;

    /**
     * Array containing the values of the 2D array
     */
    private T[] array2DArray;

    /*
    Error Messages
     */
    private final static String INVALID_SIZE = "The size of the array is not equal to nbRows*nbColumns";

}
