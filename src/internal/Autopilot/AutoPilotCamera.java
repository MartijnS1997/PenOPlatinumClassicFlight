package internal.Autopilot;

import internal.Helper.CameraImage;
import internal.Exceptions.NoCubeException;
import internal.Helper.Pixel;

import java.util.*;


/**
 * Created by Martijn on 14/10/2017.
 * @author Martijn Sauwens
 *
 * note:
 * _____________________________________> (0, n: columns or x values)
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |
 * |/
 * (0, m : the rows or y values)
 */
public class AutoPilotCamera {

    public AutoPilotCamera(byte[] image, float horizontalAngleOfView, float verticalAngleOfView, int nbRows, int nbColumns) throws IllegalArgumentException{

        if(!isValidAngleOfView(horizontalAngleOfView) || ! isValidAngleOfView(verticalAngleOfView))
            throw new IllegalArgumentException(VIEWINGANGLE_EXCEPTION);

        this.nbRows = nbRows;
        this.nbColumns = nbColumns;
        this.horizAngleOfView = horizontalAngleOfView;
        this.verticalAngleOfView = verticalAngleOfView;
        this.setCameraImage(this.convertToCameraImage(image, nbRows, nbColumns));
    }


    /**
     * Loads the new image and searches for different colored cubes
     * @param imageByteArray the byte array containing the information for the cubes
     */
    public void loadNewImage(byte[] imageByteArray){

        // check if the new image is possible
        if(!canHaveAsImageArray(imageByteArray))
            throw new IllegalArgumentException(INCOMPATIBLE_SIZE);
        CameraImage newImage = this.convertToCameraImage(imageByteArray, this.getNbRows(), this.getNbColumns());
        //System.out.println("looking for cubes");
        this.setCameraImage(newImage);
        List<internal.Helper.Vector> cubesInPicture = this.getAllCubeCenters();
        //System.out.println(cubesInPicture);
        this.setCubesInPicture(cubesInPicture);
    }

    /**
     * Method that based on the camera input locates all the centers of different colored cubes ordered by
     * on screen size
     * @return a ordered list containing the on screen location of the cube (y and x value) and the size
     * (nb pixels) of the cube on screen, ordered by on screen size
     */
    public List<internal.Helper.Vector> getAllCubeCenters(){
        CameraImage currentImage = this.getCameraImage();
        // first get the center coordinates
        Map<Coordinates, internal.Helper.Vector> cubeCenterMap = getDifferentCubeMap(currentImage);

        // then get the ordered list, small to large
        List<internal.Helper.Vector> orderedList = new ArrayList<>(cubeCenterMap.values());
        orderedList.sort(new Comparator<internal.Helper.Vector>() {
            @Override
            public int compare(internal.Helper.Vector o1, internal.Helper.Vector o2) {
                return -(int) Math.signum(o1.getzValue() - o2.getzValue());
            }
        });

        return orderedList;

    }

    /**
     * method that returns a map containing the different cubes (HS values) and their respective center coordinates
     * @param currentImage the image to process
     * @return a map containing different HS values (representing different cubes) with a corresponding vector
     * (y and x location, z size)
     */
    private Map<Coordinates, internal.Helper.Vector> getDifferentCubeMap(CameraImage currentImage) {
        // the map containing all the different cubes (keys are HS values, vector contains the mean position
        // and the onscreen size (z value)
        Map<Coordinates, internal.Helper.Vector> colorCenter =  new HashMap<>();

        for(int i = 0; i !=this.getNbRows(); i++){
            for(int j = 0; j!= this.getNbColumns(); j++) {
                // first convert the current pixel
                Pixel currentPixel = currentImage.getElementAtIndex(i, j);
                float[] HSVPixel = currentPixel.convertToHSV();
                float H = HSVPixel[0];
                float S = HSVPixel[1];
                float V = HSVPixel[2];


                // H can be NaN
                if (Float.isNaN(H)) {
                    H = 0.0f;
                }
                //if it is a white pixel ignore it
                if(V >= 0.5f)
                    continue;

                //generate a coordinates object
                Coordinates HS = new Coordinates(H, S);

                // get the vector if there was already a valid entry
                internal.Helper.Vector entry = colorCenter.get(HS);

                // if the HS value is not yet part of the map, add it
                if (entry == null) {
                    colorCenter.put(HS, new internal.Helper.Vector(i, j, 1.0f));
                    //if it is already present, calculate the new mean value for the position
                }else{
                    // first load the old values for the entry
                    // the i mean is the mean of the rows, corresponding to the y values
                    float oldIMean = entry.getxValue();
                    // the j mean is the mean of the columns, corresponding to the x values
                    float oldJMean = entry.getyValue();
                    float oldSize = entry.getzValue();

                    //calculate the new mean
                    float iNumerator = oldIMean*oldSize + i;
                    float jNumerator = oldJMean*oldSize + j;

                    // add one for the new mean size
                    float denominator = oldSize + 1;

                    internal.Helper.Vector newMean = new internal.Helper.Vector(iNumerator/denominator, jNumerator/denominator, denominator);

                    colorCenter.put(HS, newMean);
                }
            }
        }
        return colorCenter;
    }

    /**
     * Get the mean of the center coordinates of the N biggest on screen cubes, if there are less than N cubes
     * on screen, return all onscreen coordinates
     * @param nbOfCubes the amount of cubes involved in the mean
     * @return the mean coordinates of the N closest cubes, if there are less than N cubes on screen
     * return the mean of all cubes on screen. Vector layout: (x-coord, y-coord, size)
     * @throws NoCubeException thrown if no cube was found
     */
    public internal.Helper.Vector getCenterOfNCubes(int nbOfCubes) throws NoCubeException {
        List<internal.Helper.Vector> cubesInPicture = this.getCubesInPicture();

        /*if(cubesInPicture.size() == 0){
            throw new NoCubeException();
        }*/

        float xOffset = this.getNbColumns()/2.0f;
        float yOffset = this.getNbRows()/2.0f;

        int cubeCounter = 0;
        internal.Helper.Vector centerVector = new internal.Helper.Vector();
        // the x value is the x pos of the cube, the y value is the y pos if the cube
        // and the z value is the size of the cube
        for(internal.Helper.Vector cube: cubesInPicture) {
            if(cubeCounter == nbOfCubes){
                return centerVector;
            }
            float currentPixels = cube.getzValue();
            float prevTotalPixels = centerVector.getzValue();
            float newTotalPixels = currentPixels + prevTotalPixels;

            float xCenter = (centerVector.getxValue() * prevTotalPixels + cube.getyValue()*currentPixels)/(newTotalPixels);
            float yCenter = (centerVector.getyValue() * prevTotalPixels + cube.getxValue()*currentPixels)/(newTotalPixels);

            centerVector = new internal.Helper.Vector(xCenter, yCenter, newTotalPixels);
        }

        // account for the offsets
        return new internal.Helper.Vector(centerVector.getxValue() - xOffset, - centerVector.getyValue() + yOffset, centerVector.getzValue());

    }


    /**
     * A private class of immutable Coordinates
     * @author Martijn Sauwens
     */
    private class Coordinates{

        Coordinates(float x, float y){
            xCoordinate = x;
            yCoordinate = y;
        }

        /**
         * calculates the sum of two coordinates
         * @param other the other coordinates to sum
         * @return a new coordinates object containing the sum of both coordinates
         */
        public Coordinates sum(Coordinates other){
            float x = this.xCoordinate + other.xCoordinate;
            float y = this.yCoordinate + other.yCoordinate;

            return new Coordinates(x, y);
        }

        /**
         * Method that rescales the coordinates with a given scalar
         * @param scalar the scalar to multiply the coordinates with
         * @return  a new coordinates object containing the rescaled coordinates
         *          Coordinates( scalar * x, scalar * y)
         */
        private Coordinates scalarMult(float scalar){
            float x = this.getXCoordinate();
            float y = this.getYCoordinate();

            return new Coordinates(x * scalar, y*scalar);
        }

        private float getXCoordinate() {
            return xCoordinate;
        }

        private float getYCoordinate(){
            return yCoordinate;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof Coordinates)) return false;

            Coordinates that = (Coordinates) o;
            float errorMargin = 0.1f;
            return Pixel.isEqualFloat(this.getXCoordinate(), that.getXCoordinate(), errorMargin)&&
                    Pixel.isEqualFloat(this.getYCoordinate(), that.getYCoordinate(), errorMargin);

        }

        @Override
        public int hashCode() {
            int result = (getXCoordinate() != +0.0f ? Float.floatToIntBits(getXCoordinate()) : 0);
            result = 31 * result + (getYCoordinate() != +0.0f ? Float.floatToIntBits(getYCoordinate()) : 0);
            return result;
        }

        @Override
        public String toString() {
            return "(" +  xCoordinate +
                    " ; "+ yCoordinate +
                    ")";
        }

        private float xCoordinate;
        private float yCoordinate;
    }

    /**
     * Converts an array of bytes containing the RGB color scheme to an array consisting of pixels
     * @param image the byte array containing the data for the pixels
     * @return an array containing the pixels of the image
     * byte array ={ R1, G1, B1, R2, G2, B2, R3, G3, B3} then
     * pixel array = { P1(RGB), P2(RGB), P3(RGB)}
     */
    public Pixel[] convertToPixelArray(byte[] image){
        int imageArraySize = image.length;
        int pixelArraySize = imageArraySize/NB_OF_BYTES_IN_PIXEL;
        Pixel[] pixelArray = new Pixel[pixelArraySize];

        //loop through the indices of the pixel array
        for(int pixelIndex = 0; pixelIndex != pixelArraySize; pixelIndex++){
            byte[] onePixel = new byte[NB_OF_BYTES_IN_PIXEL];

            // the byte array is three times longer than the pixel array, place every three bytes in one pixel
            for(int byteExtraIndex = 0; byteExtraIndex != NB_OF_BYTES_IN_PIXEL; byteExtraIndex++){

                onePixel[byteExtraIndex] = image[pixelIndex*NB_OF_BYTES_IN_PIXEL + byteExtraIndex];
            }
            pixelArray[pixelIndex] = new Pixel(onePixel);
        }

        return pixelArray;
    }


    /**
     * Converts an image byte array to a 2D array containing the pixels of the image
     * @param byteImage the bytes containing the pixels of the image
     * @param nbRows the number of rows of the pixel array
     * @param nbColumns the number cf columns of the pixel array
     * @return a 2D array containing the pixels of the image
     */
    public CameraImage convertToCameraImage(byte[] byteImage, int nbRows, int nbColumns){
        Pixel[] pixelArray = this.convertToPixelArray(byteImage);
        return new CameraImage(pixelArray, nbRows, nbColumns);
    }

    /**
     * Checks if the angle of view is valid
     * @param angle the angle to check
     * @return true if and only if the view is within range (0°, 180°]
     */
    public boolean isValidAngleOfView(float angle){
        //System.out.println("viewing Angle: " + angle);
        return angle > 0.0f && angle <= Math.PI;
    }

    public CameraImage getCameraImage(){
        return this.cameraImage;
    }

    public void setCameraImage(CameraImage cameraImage){
        this.cameraImage = cameraImage;
    }

    /**
     * Checks if the given image array can be set as as image array
     * @param imageArray the cameraImage to be checked
     * @return true if and only if the cameraImage is the right size
     */
    public boolean canHaveAsImageArray(byte[] imageArray){
       return this.getNbRows()*this.getNbColumns() == imageArray.length/NB_OF_BYTES_IN_PIXEL;
    }

    /**
     * Getter for the number of rows in the camera image
     * @return the number of rows in a picture
     */
    public int getNbRows() {
        return nbRows;
    }

    /**
     * Getter for the number of columns in the camera image
     * @return the number of columns in a picture
     */
    public int getNbColumns() {
        return nbColumns;
    }

    /**
     * @author anthonyrathe
     * @return
     */
    public internal.Helper.Vector getDestination(){
        return this.destination;
    }

    /**
     * @author anthonyrathe
     * @return
     */
    public void setDestination(internal.Helper.Vector destination){
        this.destination = destination;
    }


    /**
     * get pixels that qualify the recognition criteria
     * @return the total qualified pixels
     */
    public int getTotalQualifiedPixels() {
        return totalQualifiedPixels;
    }

    /**
     * the total amount of qualified pixels
     * @param totalQualifiedPixels an integer containing the total amount of qualified pixels
     */
    public void setTotalQualifiedPixels(int totalQualifiedPixels) {
        this.totalQualifiedPixels = totalQualifiedPixels;
    }

    /**
     * Getter for the list that contains the cubes of different color that could be located in the picture
     * @return a list containing the on screen center coordinates of the cubes and the nb of pixels
     */
    public List<internal.Helper.Vector> getCubesInPicture() {
        return cubesInPicture;
    }

    /**
     * Setter for the list tht contains the cubes of different color
     * @param cubesInPicture the list containing vectors with the first two being the onscreen coordinates of the cubes
     *                       and the final element being their size
     */
    public void setCubesInPicture(List<internal.Helper.Vector> cubesInPicture) {
        this.cubesInPicture = cubesInPicture;
    }

    /*
                Variables
                 */
    private CameraImage cameraImage;

    /**
     * The number of pixel rows the pixel image contains (immutable)
     */
    private int nbRows;
    /**
     * The number of pixel columns the image contains (immutable)
     */
    private int nbColumns;
    /**
     * horizontal viewing angle of the drone (immutable)
     */
    private float horizAngleOfView;
    /**
     * vertical viewing angle of the drone (immutable)
     */
    private float verticalAngleOfView;

    private internal.Helper.Vector destination;
    private int totalQualifiedPixels;
    private List<internal.Helper.Vector> cubesInPicture = new ArrayList<>();


    /*
    Constant tags
     */
    public final static String RED_PIXEL = "red";
    public final static String BLUE_PIXEL = "blue";
    public final static String GREEN_PIXEL = "green";
    /*
    Constants
     */
    private final static int NB_OF_BYTES_IN_PIXEL = 3;
    private final static float RED_H_VALUE = 0.0f;
    private final static float RED_S_VALUE = 1.0f;
    private final static float Z_AXIS_V_VALUE = 0.7f;
    private final static float EPSILON  = 1E-2f;


    /*
    Error Messages
     */
    public final static String IO_EXCEPTION = "Something went wrong reading the image";
    public final static String VIEWINGANGLE_EXCEPTION = "the viewing angle is out of range (0, PI]";
    public final static String ILLEGAL_SIZE = "The byte array cannot be converted to a 2D pixel array";
    public final static String INCOMPATIBLE_SIZE = "the sample size is not a multiple of the dimensions of the array";


}
