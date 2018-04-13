package internal.Helper;


/**
 * a class of Immutable pixels, represented by RGB values in bytes.
 * the RGB values in bytes are represented in two's complement
 * Created by Martijn on 14/10/2017.
 * @author Martijn
 */
public class Pixel {

    /**
     * Constructor for a pixel class object
     * @param red the red value of the pixel (two's complement)
     * @param green the green value of the pixel (two's complement)
     * @param blue the blue value of the pixel (two's complement)
     */
    public Pixel(byte red, byte green, byte blue){
        this.setRed(red);
        this.setGreen(green);
        this.setBlue(blue);
    }


    /**
     * Constructor for a pixel class object
     * @param pixelArray array containing the RGB values (red, green, blue) in that specific order
     */
    public Pixel(byte[] pixelArray){
        if(!canBeConvertedToPixel(pixelArray))
            throw new IllegalArgumentException(INVALID_ARRAY_SIZE);

        byte red = pixelArray[0];
        byte green = pixelArray[1];
        byte blue = pixelArray[2];

        this.red = red;
        this.green = green;
        this.blue = blue;
    }

    private boolean canBeConvertedToPixel(byte[] byteArray){
        return byteArray.length == NB_OF_BYTES_IN_PIXEL;
    }


    /**
     * Converts the pixel's RGB value to HSV values
     * @return an array Containing the (H, S, V);
     */
    public float[] convertToHSV(){
        float R = this.getRedFloat();
        float G = this.getGreenFloat();
        float B = this.getBlueFloat();
        return HSVconverter.RGBtoHSV(R, G, B);
    }

    /**
     * checks if a floating point value is equal to the other within the margin of error
     * @param value1 the first value
     * @param value2 the second value
     * @param errorMargin the margin of error for the floating point values
     * @return returns true if and only if.
     *         value1 >= value2 - errorMargin && value1 <= value2 + errorMargin
     */
    public static boolean isEqualFloat(float value1, float value2, float errorMargin){
        return value1 >= value2 - errorMargin && value1 <= value2 + errorMargin;
    }

    /**
     * Getter for the red value of the pixel
     * @return a byte value, range (-128, 127)
     */
    public byte getRed() {
        return this.red;
    }

    /**
     * Getter for the red value of the pixel in integer value
     * @return an integer value, range (0, 255)
     */
    public int getRedInt(){
        return this.getRed()&0xFF;
    }

    /**
     * getter for the red value of the pixel in float value
     * @return an floating point value, range (0.0f, 1.0f)
     */
    public float getRedFloat(){
        return this.getRedInt()/255.0f;
    }
    /**
     * Setter for the red value of the pixel
     * @param red the desired red value between -128 and 127
     */
    public void setRed(byte red) {
        this.red = red;
    }

    /**
     * Getter for the green value of the pixel (two's complement)
     * @return a byte value, range (-128, 127)
     */
    public byte getGreen() {
        return this.green;
    }

    /**
     * Getter for the green value of the pixel
     * @return an integer value, range (0, 255)
     */
    public int getGreenInt(){
        return this.getGreen()&0xFF;
    }

    /**
     * Getter for the green floating point value of the pixel
     * @return an floating point value, range (0.0f, 1.0f)
     */
    public float getGreenFloat(){
        return this.getGreenInt()/255.0f;
    }

    /**
     * Setter for the green value of the pixel
     * @param green the desired green value between -128 and 127
     */
    public void setGreen(byte green) {
        this.green = green;
    }


    /**
     * Getter for the blue value of the pixel (two's complement)
     * @return a byte value, range(-128, 127)
     */
    public byte getBlue() {
        return blue;
    }

    /**
     * getter for the blue value of the pixel
     * @return an integer value, range(0,255)
     */
    public int getBlueInt(){
        return this.getBlue()&0xFF;
    }

    /**
     * Getter for the blue value of he pixel in floating point
     * @return a floating point value, range (0.0f, 1.0f)
     */
    public float getBlueFloat(){
        return this.getBlueInt()/255.0f;
    }

    /**
     * Setter for the blue value of the pixel
     * @param blue the desired blue value between -128 and 127
     */
    public void setBlue(byte blue) {
        this.blue = blue;
    }


    @Override
    public String toString() {
        return "Pixel{" +
                "red=" + (red) +
                ", green=" + (green) +
                ", blue=" + (blue) +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Pixel)) return false;

        Pixel pixel = (Pixel) o;

        if (getRed() != pixel.getRed()) return false;
        if (getGreen() != pixel.getGreen()) return false;
        return getBlue() == pixel.getBlue();
    }

    @Override
    public int hashCode() {
        int result = (int) getRed();
        result = 31 * result + (int) getGreen();
        result = 31 * result + (int) getBlue();
        return result;
    }

    /*
    Instance Variables
     */
    /**
     * A variable containing the red value of the pixel
     */
    byte red;

    /**
     * A variable containing the green value of the pixel
     */
    byte green;

    /**
     * A variable containing the blue value of the pixel
     */
    byte blue;

    /*
    Constants
     */
    public final static int NB_OF_BYTES_IN_PIXEL = 3;


    /*
    Error Messages
     */
    private final static String INVALID_ARRAY_SIZE = "The supplied array is not size 3";
}
