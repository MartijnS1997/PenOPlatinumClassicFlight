package internal.Helper;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.NoSuchFileException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

/**
 * Parser class, used for reading block-co�rdinates in order to pass them to the worldBuilder
 * @author Anthony Rathe
 */
public class Parser {
	
	/**
	 * Initialize a Parser and link it to a given file.
	 * @param path The path to the file containing the co�rdinates
	 */
	public Parser(String path) {
	
		this.path = Paths.get(path);
		
	}
	
	/**
	 * Getter for retrieving the path to the co�rdinate-file
	 * @return The path to the file containing the co�rdinates
	 */
	public Path getPath() {
		return this.path;
	}
	
	/**
	 * Method that reads the co�rdinate-file and returns a list of vectors
	 * @return List of vectors containing the co�rdinates of the blocks to be added
	 * @throws IOException
	 */
	public List<Vector> getCoordinates() throws IOException {
		List<Vector> coordinates = new ArrayList<Vector>();
		List<String> coordinateStrings = new ArrayList<String>();
		try {
			Stream<String> coordinateStream = Files.lines(getPath(), StandardCharsets.UTF_8);
			coordinateStream.forEach(coordinateStrings::add);
			try {
				for (String coordinateString : coordinateStrings) {
					String[] splitString = coordinateString.split("\\s+");
					float xValue = Float.parseFloat(splitString[0]);
					float yValue = Float.parseFloat(splitString[1]);
					float zValue = Float.parseFloat(splitString[2]);
					coordinates.add(new Vector(xValue, yValue, zValue));
				}
			}catch(Exception exception) {
				coordinateStream.close();
				throw new IOException("Something was wrong with the given coordinates file: ", exception);
			}
			coordinateStream.close();
			return coordinates;
		}catch (NoSuchFileException exception){
			throw new IllegalArgumentException("The given file path does not exist.", exception);
		}
		
		
	}
	
	public List<Vector> getBlockData() throws IOException {
		List<Vector> blockData = new ArrayList<Vector>();
		List<String> dataStrings = new ArrayList<String>();
		try {
			Stream<String> dataStream = Files.lines(getPath(), StandardCharsets.UTF_8);
			dataStream.forEach(dataStrings::add);
			try {
				for (String dataString : dataStrings) {
					String[] splitString = dataString.split("\\s+");
					float xValue = Float.parseFloat(splitString[0]);
					float yValue = Float.parseFloat(splitString[1]);
					float zValue = Float.parseFloat(splitString[2]);
					blockData.add(new Vector(xValue, yValue, zValue));
					
					float hValue = Float.parseFloat(splitString[3]);
					float sValue = Float.parseFloat(splitString[4]);
					float vValue = Float.parseFloat(splitString[5]);
					blockData.add(new Vector(hValue, sValue, vValue));
				}
			}catch(Exception exception) {
				dataStream.close();
				throw new IOException("Something was wrong with the given coordinates file: ", exception);
			}
			dataStream.close();
			return blockData;
		}catch (NoSuchFileException exception){
			throw new IllegalArgumentException("The given file path does not exist.", exception);
		}
		
		
	}
	
	/**
	 * Variable storing the path to the text file containing the co�rdinates
	 */
	private final Path path;
}
