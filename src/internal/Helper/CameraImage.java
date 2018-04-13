package internal.Helper;


import java.util.List;

/**
 * Created by Martijn on 28/10/2017.
 * a class of immutable images
 */
public class CameraImage extends Array2D<Pixel> {

    /**
     * Constructor for an image object
     * @param imageArray the array containing the image
     * @param nbRows the nb of rows in the image
     * @param nbColumns the nb of columns in the image
     */
    public CameraImage(Pixel[] imageArray, int nbRows, int nbColumns){
        super(imageArray, nbRows, nbColumns);
    }


    /**
     * Rescales the given image to an image of size (newNbRows*newNbColumns)
     * @param newNbRows the nb of rows the rescaled image contains
     * @param newNbColumns the nb of columns the rescaled image contains
     * @return a new image rescaled to the parameters newNbRows and newNbColumns
     * note: algorithm is modified version from the one used in:
     * http://tech-algorithm.com/articles/nearest-neighbor-image-scaling/
     */
    public CameraImage rescale(int newNbRows,int newNbColumns){

        int oldNbRows = this.getNbRows();
        int oldNbColumns = this.getNbColumns();

        Pixel[] temp = new Pixel[newNbRows*newNbColumns];
        float xRatio = oldNbColumns/(float)newNbColumns;
        float yRatio = oldNbRows/(float)newNbRows;
        float px, py;
        List<Pixel> image = this.getArray2DList();

        for(int i = 0; i!= newNbRows; i++){
            for(int j = 0; j!= newNbColumns; j++){
                px = (float) Math.floor(j*xRatio);
                py = (float) Math.floor(i*yRatio);
                temp[(i*newNbColumns) + j] = image.get((int)(py*oldNbColumns + px));

            }
        }

        return new CameraImage(temp, newNbRows, newNbColumns);
    }
}
