package internal.Helper;


import static java.lang.Math.max;
import static java.lang.Math.min;

public class HSVconverter {
	//http://www.java2s.com/Code/Java/2D-Graphics-GUI/HSVtoRGB.htm
	public static float[] HSVtoRGB(float h, float s, float v){

	    
		 	float m, n, f;
	        int i;

	        float[] hsv = new float[3];
	        float[] rgb = new float[3];

	        hsv[0] = h;
	        hsv[1] = s;
	        hsv[2] = v;

	        if (hsv[0] == -1)
	        {
	            rgb[0] = rgb[1] = rgb[2] = hsv[2];
	            return rgb;
	        }
	        i = (int) (Math.floor(hsv[0]));
	        f = hsv[0] - i;
	        if (i % 2 == 0)
	        {
	            f = 1 - f; // if i is even
	        }
	        m = hsv[2] * (1 - hsv[1]);
	        n = hsv[2] * (1 - hsv[1] * f);
	        switch (i)
	        {
	            case 0:
	                rgb[0] = hsv[2];
	                rgb[1] = n;
	                rgb[2] = m;
	                break;
	            case 1:
	                rgb[0] = n;
	                rgb[1] = hsv[2];
	                rgb[2] = m;
	                break;
	            case 2:
	                rgb[0] = m;
	                rgb[1] = hsv[2];
	                rgb[2] = n;
	                break;
	            case 3:
	                rgb[0] = m;
	                rgb[1] = n;
	                rgb[2] = hsv[2];
	                break;
	            case 4:
	                rgb[0] = n;
	                rgb[1] = m;
	                rgb[2] = hsv[2];
	                break;
	            case 5:
	                rgb[0] = hsv[2];
	                rgb[1] = m;
	                rgb[2] = n;
	                break;
	        }

	        return rgb;
	}
	//https://gist.github.com/fairlight1337/4935ae72bcbcc1ba5c72
	//converted C++ code to java
	/**\brief Convert RGB to HSV color space

	  Converts a given set of RGB values `r', `g', `b' into HSV
	  coordinates. The input RGB values are in the range [0, 1], and the
	  output HSV values are in the ranges h = [0, 360], and s, v = [0,
	  1], respectively.

	  @param fR Red component, used as input, range: [0, 1]
	  @param fG Green component, used as input, range: [0, 1]
	  @param fB Blue component, used as input, range: [0, 1]

	*/
	public static float[] RGBtoHSV(float fR, float fG, float fB) {
		float fH = 0.0f;
		float fS = 0.0f;
		float fV = 0.0f;
		float fCMax = max(max(fR, fG), fB);
		float fCMin = min(min(fR, fG), fB);
		float fDelta = fCMax - fCMin;

		if(fDelta > 0) {
			if(fCMax == fR) {
				fH = 60 * ((((fG - fB) / fDelta)% 6));
			} else if(fCMax == fG) {
				fH = 60 * (((fB - fR) / fDelta) + 2);
			} else if(fCMax == fB) {
				fH = 60 * (((fR - fG) / fDelta) + 4);
			}

			if(fCMax > 0) {
				fS = fDelta / fCMax;
			} else {
				fS = 0;
			}

			fV = fCMax;
		} else {
			fH = 0;
			fS = 0;
			fV = fCMax;
		}

		if(fH < 0) {
			fH = 360 + fH;
		}

		return new float[] {fH, fS, fV};
	}
	
//		formula source:	http://www.rapidtables.com/convert/color/hsv-to-rgb.htm
		public static float[] HSVtoRGB2(float h, float s, float v){
			float c = s*v;
			float x = c * (1 - (Math.abs((h/60)%2-1)));
			float m = v-c;
			
			float r;
			float g;
			float b;
			
			if (h >= 0 && h < 60){
				r = c;
				g = x;
				b = 0;
			}
			else if (h >= 60 && h < 120){
				r = x;
				g = c;
				b = 0;
			}
			else if (h >= 120 && h < 180){
				r = 0;
				g = c;
				b = x;
			}
			else if (h >= 180 && h < 240){
				r = 0;
				g = x;
				b = c;
			}
			else if (h >= 240 && h < 300){
				r = x;
				g = 0;
				b = c;
			}
			else {
				r = c;
				g = 0;
				b = x;
			}
			float[] rgb = {r+m, g+m, b+m}; 
			return rgb;
		}
}
