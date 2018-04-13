package gui;

import static org.lwjgl.opengl.GL11.GL_TRUE;
import static org.lwjgl.opengl.GL20.GL_COMPILE_STATUS;
import static org.lwjgl.opengl.GL20.GL_FRAGMENT_SHADER;
import static org.lwjgl.opengl.GL20.GL_LINK_STATUS;
import static org.lwjgl.opengl.GL20.GL_VERTEX_SHADER;
import static org.lwjgl.opengl.GL20.glAttachShader;
import static org.lwjgl.opengl.GL20.glCompileShader;
import static org.lwjgl.opengl.GL20.glCreateProgram;
import static org.lwjgl.opengl.GL20.glCreateShader;
import static org.lwjgl.opengl.GL20.glDeleteProgram;
import static org.lwjgl.opengl.GL20.glGetProgrami;
import static org.lwjgl.opengl.GL20.glGetShaderi;
import static org.lwjgl.opengl.GL20.glGetUniformLocation;
import static org.lwjgl.opengl.GL20.glLinkProgram;
import static org.lwjgl.opengl.GL20.glShaderSource;
import static org.lwjgl.opengl.GL20.glUniformMatrix4fv;
import static org.lwjgl.opengl.GL20.glUseProgram;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.FloatBuffer;
import java.util.HashMap;
import java.util.Map;

import org.lwjgl.system.MemoryStack;

import math.Matrix4f;

public class ShaderProgram {
	private int programID;
	private CharSequence vertexShader;
	private CharSequence fragmentShader;
	private final Map<String, Integer> uniforms;
	
	public ShaderProgram(boolean compiled, CharSequence vertexsrc, CharSequence fragmentsrc) {
		if (compiled) {
			this.vertexShader = vertexsrc;
			this.fragmentShader = fragmentsrc;
		} else {
			this.vertexShader = loadShader((String) vertexsrc);
			this.fragmentShader = loadShader((String) fragmentsrc);
		}
		uniforms = new HashMap<>();
	}

	public void init() {
		programID = glCreateProgram();
        int vertexId = glCreateShader(GL_VERTEX_SHADER);
        glShaderSource(vertexId, vertexShader);
        glCompileShader(vertexId);
        if(glGetShaderi(vertexId, GL_COMPILE_STATUS) != GL_TRUE) {
            //System.out.println(glGetShaderInfoLog(vertexId, Integer.MAX_VALUE));
            throw new RuntimeException();
        }
        
        int fragmentId = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fragmentId, fragmentShader);
        glCompileShader(fragmentId);
        if(glGetShaderi(fragmentId, GL_COMPILE_STATUS) != GL_TRUE) {
            //System.out.println(glGetShaderInfoLog(fragmentId, Integer.MAX_VALUE));
            throw new RuntimeException();
        }
        
        glAttachShader(programID, vertexId);
        glAttachShader(programID, fragmentId);
        glLinkProgram(programID);
        if(glGetProgrami(programID, GL_LINK_STATUS) != GL_TRUE) {
            //System.out.println(glGetProgramInfoLog(programID, Integer.MAX_VALUE));
            throw new RuntimeException();
        }
	}
	
	public void createUniform(String uniformName) throws Exception {
		int uniformLocation = glGetUniformLocation(programID, uniformName);
        if (uniformLocation < 0) {
            throw new Exception("Could not find uniform:" + uniformName);
        }
        uniforms.put(uniformName, uniformLocation);
	}
	
	public void setUniform(String uniformName, Matrix4f matrix) {
		try (MemoryStack stack = MemoryStack.stackPush()) {
            // Dump the matrix into a float buffer
            FloatBuffer fb = stack.mallocFloat(16);
            matrix.toBuffer(fb);
            glUniformMatrix4fv(uniforms.get(uniformName), false, fb);
        }
	}
	
	public void bind() {
		//System.out.println(programID);
        glUseProgram(programID);
    }

    public void unbind() {
        glUseProgram(0);
    }

	public void delete() {
		if (programID != 0) {
            glDeleteProgram(programID);
        }
	}
	
	/**
     * Loads a shader from a file.
     *
     * @param type Type of the shader
     * @param path File path of the shader
     *
     * @return Compiled Shader from specified file
     */
    public static CharSequence loadShader(String path) {
        StringBuilder builder = new StringBuilder();

        try (InputStream in = new FileInputStream(path);
             BufferedReader reader = new BufferedReader(new InputStreamReader(in))) {
            String line;
            while ((line = reader.readLine()) != null) {
                builder.append(line).append("\n");
            }
        } catch (IOException ex) {
            throw new RuntimeException("Failed to load a shader file!"
                                       + System.lineSeparator() + ex.getMessage());
        }
        CharSequence source = builder.toString();

        return source;
    }
}
