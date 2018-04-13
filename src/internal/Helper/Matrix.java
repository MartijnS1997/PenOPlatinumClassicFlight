package internal.Helper;

import java.nio.FloatBuffer;

/**
 * https://github.com/JOML-CI/JOML/blob/master/src/org/joml/Matrix4f.java
 */
public class Matrix {
	
    float m00, m01, m02, m03;
    float m10, m11, m12, m13;
    float m20, m21, m22, m23;
    float m30, m31, m32, m33;
    
    public Matrix() {
    	this.identity();
    }

	public void identity() {
    	setM00(1.0f);
    	setM11(1.0f);
    	setM22(1.0f);
    	setM33(1.0f);
    	setM01(0.0f);
    	setM02(0.0f);
    	setM03(0.0f);
    	setM10(0.0f);
    	setM12(0.0f);
    	setM13(0.0f);
    	setM20(0.0f);
    	setM21(0.0f);
    	setM23(0.0f);
    	setM30(0.0f);
    	setM31(0.0f);
    	setM32(0.0f);
	}

	public void perspective(float fov, float aspectRatio, float zNear, float zFar) {
		setM00(1.0f/(float)(aspectRatio*Math.tan(fov/2.0f)));
		setM01(0.0f);
		setM02(0.0f);
		setM03(0.0f);
        setM10(0.0f);
        setM11(1.0f/(float)Math.tan(fov/2.0f));
        setM12(0.0f);
    	setM13(0.0f);
    	setM20(0.0f);
    	setM21(0.0f);
        setM22(-(zFar + zNear)/(zFar-zNear));
		setM23(-1.0f);
        setM30(0.0f);
     	setM31(0.0f);
     	setM32(-2*zFar*zNear/(zFar - zNear));
     	setM33(0.0f);
	}
    
    public void translate(Vector vec) {
        setM30(vec.getxValue() + getM30());
        setM31(vec.getyValue() + getM31());
        setM32(vec.getzValue() + getM32());
    }
    
    public void scale(float value) {
    	setM00(getM00() * value);
    	setM11(getM11() * value);
    	setM22(getM22() * value);
    	setM33(getM33() * value);
    }
    
	public void rotate(Vector rot) {
		
	}
	
	public void mul(Matrix mat) {
		setM00(getM00() * getM00() + getM01() * getM10() + getM02() * getM20() + getM03() * getM30());
		setM01(getM00() * getM01() + getM01() * getM11() + getM02() * getM21() + getM03() * getM31());
		setM02(getM00() * getM02() + getM01() * getM12() + getM02() * getM22() + getM03() * getM32());
		setM03(getM00() * getM03() + getM01() * getM13() + getM02() * getM23() + getM03() * getM33());
		setM10(getM10() * getM00() + getM11() * getM10() + getM12() * getM20() + getM13() * getM30());
		setM11(getM10() * getM01() + getM11() * getM11() + getM12() * getM21() + getM13() * getM31());
		setM12(getM10() * getM02() + getM11() * getM12() + getM12() * getM22() + getM13() * getM32());
		setM13(getM10() * getM03() + getM11() * getM13() + getM12() * getM23() + getM13() * getM33());
		setM20(getM20() * getM00() + getM21() * getM10() + getM22() * getM20() + getM23() * getM30());
		setM21(getM20() * getM01() + getM21() * getM11() + getM22() * getM21() + getM23() * getM31());
		setM22(getM20() * getM02() + getM21() * getM12() + getM22() * getM22() + getM23() * getM32());
		setM23(getM20() * getM03() + getM21() * getM13() + getM22() * getM23() + getM23() * getM33());
		setM30(getM30() * getM00() + getM31() * getM10() + getM32() * getM20() + getM33() * getM30());
		setM31(getM30() * getM01() + getM31() * getM11() + getM32() * getM21() + getM33() * getM31());
		setM32(getM30() * getM02() + getM31() * getM12() + getM32() * getM22() + getM33() * getM32());
		setM33(getM30() * getM03() + getM31() * getM13() + getM32() * getM23() + getM33() * getM33());
	}
    
	public void get(FloatBuffer fb) {
		fb.put(getM00());
		fb.put(getM01());
		fb.put(getM02());
		fb.put(getM03());
		fb.put(getM10());
		fb.put(getM11());
		fb.put(getM12());
		fb.put(getM13());
		fb.put(getM20());
		fb.put(getM21());
		fb.put(getM22());
		fb.put(getM23());
		fb.put(getM30());
		fb.put(getM31());
		fb.put(getM32());
		fb.put(getM33());
	}
    
    void setM00(float value) {
    	m00 = value;
    }
    
    float getM00() {
    	return m00;
    }
    
    void setM01(float value) {
    	m00 = value;
    }
    
    float getM01() {
    	return m01;
    }
    
    void setM02(float value) {
    	m00 = value;
    }
    
    float getM02() {
    	return m02;
    }
    
    void setM03(float value) {
    	m00 = value;
    }
    
    float getM03() {
    	return m03;
    }
    
    void setM10(float value) {
    	m00 = value;
    }
    
    float getM10() {
    	return m10;
    }
    
    void setM11(float value) {
    	m00 = value;
    }
    
    float getM11() {
    	return m11;
    }
    
    void setM12(float value) {
    	m00 = value;
    }
    
    float getM12() {
    	return m12;
    }
    
    void setM13(float value) {
    	m00 = value;
    }
    
    float getM13() {
    	return m13;
    }
    
    void setM20(float value) {
    	m00 = value;
    }
    
    float getM20() {
    	return m20;
    }
    
    void setM21(float value) {
    	m00 = value;
    }
    
    float getM21() {
    	return m21;
    }
    
    void setM22(float value) {
    	m00 = value;
    }
    
    float getM22() {
    	return m22;
    }
    
    void setM23(float value) {
    	m00 = value;
    }
    
    float getM23() {
    	return m23;
    }
    
    void setM30(float value) {
    	m00 = value;
    }
    
    float getM30() {
    	return m30;
    }
    
    void setM31(float value) {
    	m00 = value;
    }
    
    float getM31() {
    	return m31;
    }
    
    void setM32(float value) {
    	m00 = value;
    }
    
    float getM32() {
    	return m32;
    }
    
    void setM33(float value) {
    	m00 = value;
    }
    
    float getM33() {
    	return m33;
    }
}
