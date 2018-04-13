package tests;
import org.junit.Test;

import internal.Helper.Parser;

import java.io.IOException;

import org.junit.Before;
/**
 * 
 * @author Anthony Rathe
 *
 */
public class ParserTests {
	
	Parser parser;
	String path;
	
	@Before
	public void setup() {
		path = "src/internal/blockCoordinates.txt";
		parser = new Parser(path);
	}
	
	@Test
	public void multipleLinesOfFloatsTest() throws IOException {
		parser.getCoordinates();
	}
	
}
