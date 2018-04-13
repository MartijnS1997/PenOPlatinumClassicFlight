package internal.Helper;

/**
 * Created by Martijn on 11/10/2017.
 * A class of immutable 3by3Matrices used for transformations
 */
public class SquareMatrix {

    /**
     * Constructor for a SquareMatrix object
     * @param matrixArray a array of length 9 contianing the elements of the array
     */
    public SquareMatrix(float[] matrixArray){
        if(this.canHaveAsMatrixArray(matrixArray)){
            for(int index = 0; index != arraySize; index++){
                this.setElementAt(index, matrixArray[index]);
            }
        }else
            throw new IllegalArgumentException(OVERSIZED);
    }


    /**
     * Multiplies a 3x3 matrix with a 3x1 vector
     * @param vector the vector to multiply with
     * @return a new 3x1 vector containing the matrix vector product
     */
    public Vector matrixVectorProduct(Vector vector){
        float[] vectorArray = new float[3];

        for(int rowIndex = 0; rowIndex != nbRows; rowIndex++){
            float[] tempRow = this.getRow(rowIndex);
            Vector tempVector = new Vector(tempRow);
            vectorArray[rowIndex] = tempVector.scalarProduct(vector);
        }

        return new Vector(vectorArray);
    }

    /**
     * Calculates the matrixProduct of two matrices
     * @param other the other matrix
     * @return a new 3x3 matrix containing the matrix product
     */
    public SquareMatrix matrixProduct(SquareMatrix other){

        float[] newMatrixArray = new float[arraySize];

        for( int rowIndex = 0; rowIndex !=nbRows; rowIndex++){
            float[] tempRow = this.getRow(rowIndex);
            for( int columnIndex = 0; columnIndex != nbColumns; columnIndex++){
                float[] tempColumn = other.getColumn(columnIndex);
                float scalarProduct = 0;
                for( int scalarIndex = 0; scalarIndex != nbRows; scalarIndex ++){
                    scalarProduct += tempRow[scalarIndex]*tempColumn[scalarIndex];
                }
                newMatrixArray[rowIndex*nbRows + columnIndex] = scalarProduct;
            }
        }

        return new SquareMatrix(newMatrixArray);
    }

    /**
     * Calculates the transpose of the given matrix
     * @return a new matrix containing the transpose of the given matrix
     */
    public SquareMatrix transpose(){

        float[] tempRow;
        float[] newMatrixArray = new float[arraySize];

        for(int i = 0; i != nbColumns; i++){
            tempRow = this.getColumn(i);

            for(int j = 0; j != nbRows; j++){
                newMatrixArray[i*nbRows + j] = tempRow[j];
            }
        }

        return new SquareMatrix(newMatrixArray);
    }

    /**
     * Calculates the multiplication of a matrix with a scalar
     * @param scalar the scalar to multiply with
     * @return a new matrix containing multiplication of the given matrix and the scalar
     */
    public SquareMatrix scalarMult(float scalar){

        float[] newMatrixArray = new float[arraySize];

        for(int i = 0; i != arraySize; i++){
            newMatrixArray[i] =  this.getElementAt(i) * scalar;
        }

        return new SquareMatrix(newMatrixArray);
    }

    /**
     * Calculates the sum two matrices
     * @param other the other matrix
     * @return a new matrix containing the sum of the two provided matrices
     */
    public SquareMatrix matrixSum(SquareMatrix other){

        float[] newMatrixArray = new float[arraySize];

        for(int index = 0; index != arraySize; index++){
            newMatrixArray[index] = this.getElementAt(index) + other.getElementAt(index);
        }

        return new SquareMatrix(newMatrixArray);
    }

    /**
     * Calculates the difference of two matrices
     * @param other the other matrix
     * @return a new matrix containing the difference of the two provided matrices
     */
    public SquareMatrix matrixDiff(SquareMatrix other){
        float[] newMatrixArray = new float[arraySize];

        for(int index = 0; index != arraySize; index++){
            newMatrixArray[index] = this.getElementAt(index) - other.getElementAt(index);
        }

        return new SquareMatrix(newMatrixArray);
    }

    /**
     * Calculates the inverse of a diagonal matrix
     * @return a new matrix containing the inverse of the diagonal matrix
     * note: the inverse of a diagonal matrix can be calculated by raising the
     * diagonal elements to the power of -1
     */
    public SquareMatrix invertDiagonal(){
        if(!this.isDiagonal())
            throw new IllegalArgumentException(NOT_DIAGONAL);
        int jumpSize = nbRows + 1;
        float[] newMatrixArray = new float[arraySize];

        for(int index = 0; index != nbRows; index++){
            newMatrixArray[index*(nbRows + 1)] = 1/this.getElementAt(index*(nbRows + 1));
        }

        return new SquareMatrix(newMatrixArray);
    }

    /**
     * checks if the given matrix is diagonal
     * @return true if and only if the matrix is diagonal
     * note an element is on the diagonal if the index is i*(nbRows + 1) with i the current row
     * so in this case if the index is a multiple of 4, the element is one on the diagonal and thus
     * modulus 4 of the index is 0
     */
    public boolean isDiagonal(){
        for(int index = 0; index != arraySize; index++){
            float elem = this.getElementAt(index);
            if(elem != 0 && index%(nbRows+1) != 0){
                return false;
            }
        }

        return true;
    }

    /*
    Static methods
     */

    /**
     * The roll transformation matrix, roll is a rotation around the z-axis
     * @param  angle the angle of rotation around the z-axis
     * @return a matrix containing the roll transformation for a given angle
     */
    public static SquareMatrix getRollTransformMatrix(float angle){
        float[] matrixArray = new float[]{(float) Math.cos(angle), -  (float)Math.sin(angle), 0f,
                (float)Math.sin(angle),  (float) Math.cos(angle),   0f,
                0f,                      0f,                        1f};

        return new SquareMatrix(matrixArray);
    }

    /**
     * The pitch transformation matrix, pitch is a rotation around the x-axis
     * @param angle the angle of rotation around the x-axis
     * @return a matrix containing the pitch transformation for a given angle
     */
    public static SquareMatrix getPitchTransformMatrix(float angle){
        float[] matrixArray = new float[]{1f, 0f,                      0f,
                0f, (float) Math.cos(angle), - (float)Math.sin(angle),
                0f, (float)Math.sin(angle),  (float) Math.cos(angle)};

        return new SquareMatrix(matrixArray);
    }

    /**
     * The heading transformation matrix, heading is a rotation around the y-axis
     * @param angle the angle of rotation around the y-axis
     * @return a matrix containing the heading transformation for a given angle
     */
    public static SquareMatrix getHeadingTransformMatrix(float angle){
        float[] matrixArray = new float[]{(float) Math.cos(angle),   0f, (float)Math.sin(angle),
                0f,                       1f,  0f,
                - (float)Math.sin(angle), 0f, (float) Math.cos(angle)};
        return new SquareMatrix(matrixArray);
    }

    /*
    getters and setters
     */

    /**
     * Selects a row at the given index of the 3x3 matrix
     * @param index the index of the row
     * @return a new array containing the elements of a row in a 3x3 matrix at a given index
     */
    public float[] getRow(int index){

        float[] tempArray = new float[3];

        for(int i = 0; i != nbRows; i++){
            tempArray[i] = this.matrixArray[index*nbRows + i];
        }

        return tempArray;
    }

    /**
     * Selects a column at the given index of the 3x3 matrix
     * with a given index
     * @param index the index of the column
     * @return a new array containing the elements of a column in a 3x3 matrix at a given index
     */
    public float[] getColumn(int index){
        float[] tempArray = new float[3];

        for(int j = 0; j != nbColumns; j++){
            tempArray[j] = this.matrixArray[index + j*nbColumns];
        }

        return tempArray;
    }

    /**
     * checker if the matrix array is Valid
     * @param matrixArray the array containing the elements of the matrix
     * @return true if and only if the array has length 9
     */
    private boolean canHaveAsMatrixArray(float[] matrixArray){
        return matrixArray.length == arraySize;
    }

    /**
     * gets the element at the given index
     * @param index the index of the array
     * @return the element at a given index in the matrix array
     */
    private float getElementAt(int index){
        if(index < this.getArraySize() && index >= 0)
            return this.matrixArray[index];
        else
            throw new IndexOutOfBoundsException();
    }

    /**
     * Sets the element at the given index to the given value
     * @param index the index of the element in the matrixArray
     * @param elem the desired value of the element
     */
    private void setElementAt(int index, float elem){
        if(index < this.getArraySize() && index >= 0)
            this.matrixArray[index] = elem;
        else
            throw new IndexOutOfBoundsException();
    }

    @Override
    public boolean equals(Object obj) {
        if(obj == this)
            return true;
        if(obj == null)
            return false;

        for(int index = 0; index != arraySize; index++){
            if(((SquareMatrix)obj).getElementAt(index) != this.getElementAt(index))
                return false;
        }

        return true;
    }

    @Override
    public String toString(){
        String total = "";
        for(int index = 0; index != nbRows; index++){
            for(float elem: this.getRow(index)){
                total += " "+ Float.toString(elem);
            }
            total += "\n";
        }

        return total;
    }

    /**
     * Returns the size of the array
     * @return the size of the array containing the elements of the matrix
     */
    private int getArraySize(){
        return arraySize;
    }

    /**
     * array that holds the elements of the 3x3 matrix
     */
    float[] matrixArray = new float[arraySize];

    /**
     * contant that holds the size of the matrix array
     */
    final static int arraySize = 9;

    /**
     * constant that holds the number of rows
     */
    final static int nbRows = 3;

    /**
     * constant that holds the number of columns
     */
    final static int nbColumns = 3;

    /*
    Constants
     */
    public final static SquareMatrix IDENTITY = new SquareMatrix(new float[] {1.0f, 0.0f, 0.0f,
                                                                              0.0f, 1.0f, 0.0f,
                                                                              0.0f, 0.0f, 1.0f});

    /*
    Error messages
     */
    public final static String OVERSIZED = "The array is not lenght 9";
    public final static String NOT_DIAGONAL = "The provided matrix is not a diagonal matrix";
}
