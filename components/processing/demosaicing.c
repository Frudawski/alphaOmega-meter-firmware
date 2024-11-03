// DEMOSIAICING FUNCTIONS

#include "demosaicing.h"
#include "processing.h"



// DOWNSAMPLING
demosaicing downsampling(imagebuffer *pic, imagebuffer *expt, imageformat format, uint32_t RESO1, uint32_t RESO2, uint32_t n, float calR, float calG, float calB) {

	demosaicing demo;
	demo.rawrgb[0] = 0;
	demo.rawrgb[1] = 0;
	demo.rawrgb[2] = 0;

	demo.floatrgb[0] = 0.0;
	demo.floatrgb[1] = 0.0;
	demo.floatrgb[2] = 0.0;

	demo.hdrrgb[0] = 0.0;
	demo.hdrrgb[1] = 0.0;
	demo.hdrrgb[2] = 0.0;

	uint8_t rawpixel_r = 0;
	uint8_t rawpixel_g = 0;
	uint8_t rawpixel_b = 0;
	float floatpixel_R = 0.0;
	float floatpixel_G = 0.0;
	float floatpixel_B = 0.0;
	float exptime1 = 0.0;
	float exptime2 = 0.0;
	float exptime3 = 0.0;
	float exptime4 = 0.0;
	float pixel1 = 0.0;
	float pixel2 = 0.0;
	float pixel3 = 0.0;
	float pixel4 = 0.0;
	float HDRpixel_R = 0.0;
	float HDRpixel_G = 0.0;
	float HDRpixel_B = 0.0;

	float calRGB[3] = {calR, calG, calB};

	uint32_t row = 0;
	uint32_t col = 0;
	uint32_t ind1 = 0;
	uint32_t ind2 = 0;
	uint32_t ind3 = 0;
	uint32_t ind4 = 0;

	// determine row and column
	row = (uint32_t)(n/RESO1);
	col = (uint32_t)(n-row*RESO1);
	row++;
	col++;

	//ESP_LOGI(TAG,"Demosaicing - case switch");

	// detect odd or even row
	if((row%2 != 0) && (col%2 != 0)){
		// ODD ROW
		// ODD COLUMN
		// BLUE PIXEL
		switch(format){
		case PIXFORMAT_RGB:
			rawpixel_b = pic->buf[n];
			ind1 = n+1;
			ind2 = n+RESO1;
			rawpixel_g = pic->buf[ind1]/2 + pic->buf[ind2]/2;
			ind3 = n+RESO1+1;
			rawpixel_r = pic->buf[ind3];
			break;
		case PIXFORMAT_FLOAT:
			rawpixel_b = pic->buf[n];
			ind1 = n+1;
			ind2 = n+RESO1;
			rawpixel_g = pic->buf[ind1]/2 + pic->buf[ind2]/2;
			ind3 = n+RESO1+1;
			rawpixel_r = pic->buf[ind3];
			pixel1 = uint8_2_float(rawpixel_b);
			pixel2 = uint8_2_float(rawpixel_g);
			pixel3 = uint8_2_float(rawpixel_r);
			floatpixel_B = pixel1*calRGB[B];
			floatpixel_G = pixel2*calRGB[G];
			floatpixel_R = pixel3*calRGB[R];
			break;
		case PIXFORMAT_HDR:
			pixel1 = uint8_2_float(pic->buf[n]);
			exptime1 = uint8_2_float(expt->buf[n])+1.0;
			HDRpixel_B = pixel1/exptime1*calRGB[B];
			ind1 = n+1;
			ind2 = n+RESO1;
			pixel1 = uint8_2_float(pic->buf[ind1]);
			pixel2 = uint8_2_float(pic->buf[ind2]);
			exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
			exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
			HDRpixel_G = (pixel1/exptime1/2 + pixel2/exptime2/2)*calRGB[G];
			ind3 = n+RESO1+1;
			pixel1 = uint8_2_float(pic->buf[ind1]);
			exptime1  = uint8_2_float(expt->buf[ind3])+1.0;
			HDRpixel_R = pixel1/exptime1*calRGB[R];
			break;
		case PIXFORMAT_AOPIC:
			pixel1 = uint8_2_float(pic->buf[n]);
			exptime1 = uint8_2_float(expt->buf[n])+1.0;
			HDRpixel_B = pixel1/exptime1*calRGB[B];
			ind1 = n+1;
			ind2 = n+RESO1;
			pixel1 = uint8_2_float(pic->buf[ind1]);
			pixel2 = uint8_2_float(pic->buf[ind2]);
			exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
			exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
			HDRpixel_G = (pixel1/exptime1/2 + pixel2/exptime2/2)*calRGB[G];
			ind3 = n+RESO1+1;
			pixel1 = uint8_2_float(pic->buf[ind1]);
			exptime1  = uint8_2_float(expt->buf[ind3])+1.0;
			HDRpixel_R = pixel1/exptime1*calRGB[R];
			break;
		default:
			break;
		}
	}

	demo.rawrgb[0] = rawpixel_r;
	demo.rawrgb[1] = rawpixel_g;
	demo.rawrgb[2] = rawpixel_b;

	demo.floatrgb[0] = floatpixel_R;
	demo.floatrgb[1] = floatpixel_G;
	demo.floatrgb[2] = floatpixel_B;

	demo.hdrrgb[0] = HDRpixel_R;
	demo.hdrrgb[1] = HDRpixel_G;
	demo.hdrrgb[2] = HDRpixel_B;

	return demo;
}


// BILIENAR INTERPOLATION
demosaicing bilinear_demosaicing(imagebuffer *pic, imagebuffer *expt, imageformat format, uint32_t RESO1, uint32_t RESO2, uint32_t n, float calR, float calG, float calB) {

	demosaicing demo;
	demo.rawrgb[0] = 0;
	demo.rawrgb[1] = 0;
	demo.rawrgb[2] = 0;

	demo.floatrgb[0] = 0.0;
	demo.floatrgb[1] = 0.0;
	demo.floatrgb[2] = 0.0;

	demo.hdrrgb[0] = 0.0;
	demo.hdrrgb[1] = 0.0;
	demo.hdrrgb[2] = 0.0;

	uint8_t rawpixel_r = 0;
	uint8_t rawpixel_g = 0;
	uint8_t rawpixel_b = 0;
	float floatpixel_R = 0.0;
	float floatpixel_G = 0.0;
	float floatpixel_B = 0.0;
	float exptime1 = 0.0;
	float exptime2 = 0.0;
	float exptime3 = 0.0;
	float exptime4 = 0.0;
	float pixel1 = 0.0;
	float pixel2 = 0.0;
	float pixel3 = 0.0;
	float pixel4 = 0.0;
	float HDRpixel_R;
	float HDRpixel_G;
	float HDRpixel_B;

	float calRGB[3] = {calR, calG, calB};

	uint32_t row = 0;
	uint32_t col = 0;
	uint32_t ind1 = 0;
	uint32_t ind2 = 0;
	uint32_t ind3 = 0;
	uint32_t ind4 = 0;

	//ESP_LOGI(TAG,"Demosaicing - initialize struct...");

	// determine row and column
	row = (uint32_t)(n/RESO1);
	col = (uint32_t)(n-row*RESO1);
	row++;
	col++;

	//ESP_LOGI(TAG,"Demosaicing - case switch");

	// detect odd or even row
	if(row%2 != 0){
		// ODD ROW
		// detect odd or even column
		if(col%2 != 0){
			// ODD COLUMN
			// BLUE PIXEL

			switch(format){
			case PIXFORMAT_RGB:
				rawpixel_b = pic->buf[n];
				break;
			case PIXFORMAT_FLOAT:
				pixel1 = uint8_2_float(pic->buf[n]);
				floatpixel_B = pixel1*calRGB[B];
				break;
			case PIXFORMAT_HDR:
				pixel1 = uint8_2_float(pic->buf[n]);
				exptime1 = uint8_2_float(expt->buf[n])+1.0;
				HDRpixel_B = pixel1/exptime1*calRGB[B];
				break;
			case PIXFORMAT_AOPIC:
				pixel1 = uint8_2_float(pic->buf[n]);
				exptime1 = uint8_2_float(expt->buf[n])+1.0;
				HDRpixel_B = pixel1/exptime1*calRGB[B];
				break;
			default:
				break;
			}
			if(n == 0){
				// top left pixel
				ind1 = n+1+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = pic->buf[ind1];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					floatpixel_R = pixel1*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				default:
					break;
				}
				ind1 = n+1;
				ind2 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_g = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_G = (pixel1+pixel2)/2*calRGB[G];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_G = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[G];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_G = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[G];
					break;
				default:
					break;
				}
			}
			else if(row == 1){
				// top row
				ind1 = n+1+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = pic->buf[ind1];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					floatpixel_R = pixel1*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				default:
					break;
				}
				ind1 = n-1;
				ind2 = n+1;
				ind3 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_g = (pic->buf[ind1]/3+pic->buf[ind2]/3+pic->buf[ind3]/3);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					floatpixel_G = (pixel1+pixel2+pixel3)/3*calRGB[G];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					HDRpixel_G = (pixel1/exptime1/3+pixel2/exptime2/3+pixel3/exptime3/3)*calRGB[G];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					HDRpixel_G = (pixel1/exptime1/3+pixel2/exptime2/3+pixel3/exptime3/3)*calRGB[G];
					break;
				default:
					break;
				}
			}
			else if(col == 1){
				// left column
				ind1 = n+1-RESO1;
				ind2 = n+1+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_R = (pixel1+pixel2)/2*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_R = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_R = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[R];
					break;
				default:
					break;
				}
				ind1 = n-RESO1;
				ind2 = n+1;
				ind3 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_g = (pic->buf[ind1]/3+pic->buf[ind2]/3+pic->buf[ind3]/3);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					floatpixel_G = (pixel1+pixel2+pixel3)/3*calRGB[G];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					HDRpixel_G = (pixel1/exptime1/3+pixel2/exptime2/3+pixel3/exptime3/3)*calRGB[G];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					HDRpixel_G = (pixel1/exptime1/3+pixel2/exptime2/3+pixel3/exptime3/3)*calRGB[G];
					break;
				default:
					break;
				}
			}
			else{
				// inner image matrix
				ind1 = n-RESO1-1;
				ind2 = n-RESO1+1;
				ind3 = n+RESO1-1;
				ind4 = n+RESO1+1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = (0.25*pic->buf[ind1]+0.25*pic->buf[ind2]+0.25*pic->buf[ind3]+0.25*pic->buf[ind4]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					floatpixel_R = (pixel1+pixel2+pixel3+pixel4)/4*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					exptime4 = uint8_2_float(expt->buf[ind4])+1.0;
					HDRpixel_R = (pixel1/exptime1/4+pixel2/exptime2/4+pixel3/exptime3/4+pixel4/exptime4/4)*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					exptime4 = uint8_2_float(expt->buf[ind4])+1.0;
					HDRpixel_R = (pixel1/exptime1/4+pixel2/exptime2/4+pixel3/exptime3/4+pixel4/exptime4/4)*calRGB[R];
					break;
				default:
					break;
				}
				ind1 = n-RESO1;
				ind2 = n-1;
				ind3 = n+1;
				ind4 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_g = (0.25*pic->buf[ind1]+0.25*pic->buf[ind2]+0.25*pic->buf[ind3]+0.25*pic->buf[ind4]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					floatpixel_G = (pixel1+pixel2+pixel3+pixel4)/4*calRGB[G];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					exptime4 = uint8_2_float(expt->buf[ind4])+1.0;
					HDRpixel_G = (pixel1/exptime1/4+pixel2/exptime2/4+pixel3/exptime3/4+pixel4/exptime4/4)*calRGB[G];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					exptime4 = uint8_2_float(expt->buf[ind4])+1.0;
					HDRpixel_G = (pixel1/exptime1/4+pixel2/exptime2/4+pixel3/exptime3/4+pixel4/exptime4/4)*calRGB[G];
					break;
				default:
					break;
				}
			}
		}
		else{
			// EVEN COLUMN
			// GREEN PIXEL
			switch(format){
			case PIXFORMAT_RGB:
				rawpixel_g = pic->buf[n];
				break;
			case PIXFORMAT_FLOAT:
				pixel1 = uint8_2_float(pic->buf[n]);
				floatpixel_G = pixel1*calRGB[G];
				break;
			case PIXFORMAT_HDR:
				pixel1 = uint8_2_float(pic->buf[n]);
				exptime1 = uint8_2_float(expt->buf[n])+1.0;
				HDRpixel_G =  pixel1/exptime1*calRGB[G];
				break;
			case PIXFORMAT_AOPIC:
				pixel1 = uint8_2_float(pic->buf[n]);
				exptime1 = uint8_2_float(expt->buf[n])+1.0;
				HDRpixel_G =  pixel1/exptime1*calRGB[G];
				break;
			default:
				break;
			}
			if(n+1 == RESO1){
				// top right pixel
				ind1 = n-1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = pic->buf[ind1];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					floatpixel_B = pixel1*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_B = pixel1/exptime1*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_B = pixel1/exptime1*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = pic->buf[ind1];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					floatpixel_R = pixel1*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				default:
					break;
				}
			}
			else if(row == 1){
				// top row
				ind1 = n-1;
				ind2 = n+1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_B = (pixel1+pixel2)/2*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = pic->buf[ind1];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					floatpixel_R = pixel1*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				default:
					break;
				}
			}
			else if(col == RESO1){
				// right column
				ind1 = n-1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = pic->buf[ind1];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					floatpixel_B = pixel1*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_B = pixel1/exptime1*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_B = pixel1/exptime1*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n-RESO1;
				ind2 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_R = (pixel1+pixel2)/2*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_R = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_R = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[R];
					break;
				default:
					break;
				}
			}
			else{
				// inner image matrix
				ind1 = n-1;
				ind2 = n+1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_B = (pixel1+pixel2)/2*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n-RESO1;
				ind2 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_R = (pixel1+pixel2)/2*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					HDRpixel_R = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					HDRpixel_R = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[R];
					break;
				default:
					break;
				}
			}
		}
	}
	else
	{
		// EVEN ROW
		if((n+1)%2 != 0){
			// ODD CLOUMN
			// GREEN PIXEL
			switch(format){
			case PIXFORMAT_RGB:
				rawpixel_g = pic->buf[n];
				break;
			case PIXFORMAT_FLOAT:
				pixel1 = uint8_2_float(pic->buf[n]);
				floatpixel_G = pixel1*calRGB[G];
				break;
			case PIXFORMAT_HDR:
				pixel1 = uint8_2_float(pic->buf[n]);
				exptime1 = uint8_2_float(expt->buf[n])+1.0;
				HDRpixel_G = pixel1/exptime1*calRGB[G];
				break;
			case PIXFORMAT_AOPIC:
				pixel1 = uint8_2_float(pic->buf[n]);
				exptime1 = uint8_2_float(expt->buf[n])+1.0;
				HDRpixel_G = pixel1/exptime1*calRGB[G];
				break;
			default:
				break;
			}
			if((row == RESO2) & (col == 1)){
				// bottom left pixel
				ind1 = n-RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = pic->buf[ind1];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					floatpixel_B = pixel1*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_B = pixel1/exptime1*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_B = pixel1/exptime1*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n+1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = pic->buf[ind1];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					floatpixel_R = pixel1*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				default:
					break;
				}
			}
			else if(row == RESO2){
				//bottom row
				ind1 = n-RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = pic->buf[ind1];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					floatpixel_B = pixel1*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_B = pixel1/exptime1*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_B = pixel1/exptime1*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n-1;
				ind2 = n+1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_R = (pixel1+pixel2)/2*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_R = (pixel1/exptime1+pixel2/exptime2)/2*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_R = (pixel1/exptime1+pixel2/exptime2)/2*calRGB[R];
					break;
				default:
					break;
				}
			}
			else if(col == 1){
				// left column
				ind1 = n-RESO1;
				ind2 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_B = (pixel1+pixel2)/2*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n+1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = pic->buf[ind1];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					floatpixel_R = pixel1*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_R = pixel1/exptime1*calRGB[R];
					break;
				default:
					break;
				}
			}
			else{
				// inner image matrix
				ind1 = n-RESO1;
				ind2 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_B = (pixel1+pixel2)/2*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n-1;
				ind2 = n+1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_r = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_R = (pixel1+pixel2)/2*calRGB[R];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_R = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[R];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_R = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[R];
					break;
				default:
					break;
				}
			}
		}
		else{
			// EVEN COLUMN
			// RED PIXEL
			switch(format){
			case PIXFORMAT_RGB:
				rawpixel_r = pic->buf[n];
				break;
			case PIXFORMAT_FLOAT:
				pixel1 = uint8_2_float(pic->buf[n]);
				floatpixel_R = pixel1*calRGB[R];
				break;
			case PIXFORMAT_HDR:
				pixel1 = uint8_2_float(pic->buf[n]);
				exptime1 = uint8_2_float(expt->buf[n])+1.0;
				HDRpixel_R = pixel1/exptime1*calRGB[R];
				break;
			case PIXFORMAT_AOPIC:
				pixel1 = uint8_2_float(pic->buf[n]);
				exptime1 = uint8_2_float(expt->buf[n])+1.0;
				HDRpixel_R = pixel1/exptime1*calRGB[R];
				break;
			default:
				break;
			}
			if(RESO1*RESO2 == n+1){
				// bottom right pixel
				ind1 = n-RESO1-1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = pic->buf[ind1];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					floatpixel_B = pixel1*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_B = pixel1/exptime1*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					HDRpixel_B = pixel1/exptime1*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n-1;
				ind2 = n-RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_g = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_G = (pixel1+pixel2)/2*calRGB[G];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_G = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[G];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_G = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[G];
					break;
				default:
					break;
				}
			}
			else if(row == RESO2){
				// bottom row
				ind1 = n-RESO1-1;
				ind2 = n-RESO1+1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_B = (pixel1+pixel2)/2*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n-1;
				ind2 = n+1;
				ind3 = n-RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_g = (pic->buf[ind1]/3+pic->buf[ind2]/3+pic->buf[ind3]/3);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					floatpixel_G = (pixel1+pixel2+pixel3)/3*calRGB[G];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					HDRpixel_G = (pixel1/exptime1/3+pixel2/exptime2/3+pixel3/exptime3/3)*calRGB[G];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					HDRpixel_G = (pixel1/exptime1/3+pixel2/exptime2/3+pixel3/exptime3/3)*calRGB[G];
					break;
				default:
					break;
				}
			}
			else if(col == RESO1){
				// right column
				ind1 = n-RESO1-1;
				ind2 = n+RESO1-1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = (0.5*pic->buf[ind1]+0.5*pic->buf[ind2]);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					floatpixel_B = (pixel1+pixel2)/2*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					HDRpixel_B = (pixel1/exptime1/2+pixel2/exptime2/2)*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n-1;
				ind2 = n-RESO1;
				ind3 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_g = (pic->buf[ind1]/3+pic->buf[ind2]/3+pic->buf[ind3]/3);
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					floatpixel_G = (pixel1+pixel2+pixel3)/3*calRGB[G];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					HDRpixel_G = (pixel1/exptime1/3+pixel2/exptime2/3+pixel3/exptime3/3)*calRGB[G];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					HDRpixel_G = (pixel1/exptime1/3+pixel2/exptime2/3+pixel3/exptime3/3)*calRGB[G];
					break;
				default:
					break;
				}
			}
			else{
				// inner image matrix
				ind1 = n-RESO1-1;
				ind2 = n-RESO1+1;
				ind3 = n+RESO1-1;
				ind4 = n+RESO1+1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_b = (0.25*pic->buf[ind1]+0.25*pic->buf[ind2]+0.25*pic->buf[ind3]+0.25*pic->buf[ind4]);
					break;
				case  PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					floatpixel_B = (pixel1+pixel2+pixel3+pixel4)/4*calRGB[B];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					exptime4 = uint8_2_float(expt->buf[ind4])+1.0;
					HDRpixel_B = (pixel1/exptime1/4+pixel2/exptime2/4+pixel3/exptime3/4+pixel4/exptime4/4)*calRGB[B];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					exptime4 = uint8_2_float(expt->buf[ind4])+1.0;
					HDRpixel_B = (pixel1/exptime1/4+pixel2/exptime2/4+pixel3/exptime3/4+pixel4/exptime4/4)*calRGB[B];
					break;
				default:
					break;
				}
				ind1 = n-RESO1;
				ind2 = n-1;
				ind3 = n+1;
				ind4 = n+RESO1;
				switch(format){
				case PIXFORMAT_RGB:
					rawpixel_g = (0.25*pic->buf[ind1]+0.25*pic->buf[ind2]+0.25*pic->buf[ind3]+0.25*pic->buf[ind4])*calRGB[G];
					break;
				case PIXFORMAT_FLOAT:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					floatpixel_G = (pixel1+pixel2+pixel3+pixel4)/4*calRGB[G];
					break;
				case PIXFORMAT_HDR:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					exptime4 = uint8_2_float(expt->buf[ind4])+1.0;
					HDRpixel_G = (pixel1/exptime1/4+pixel2/exptime2/4+pixel3/exptime3/4+pixel4/exptime4/4)*calRGB[G];
					break;
				case PIXFORMAT_AOPIC:
					pixel1 = uint8_2_float(pic->buf[ind1]);
					pixel2 = uint8_2_float(pic->buf[ind2]);
					pixel3 = uint8_2_float(pic->buf[ind3]);
					pixel4 = uint8_2_float(pic->buf[ind4]);
					exptime1 = uint8_2_float(expt->buf[ind1])+1.0;
					exptime2 = uint8_2_float(expt->buf[ind2])+1.0;
					exptime3 = uint8_2_float(expt->buf[ind3])+1.0;
					exptime4 = uint8_2_float(expt->buf[ind4])+1.0;
					HDRpixel_G = (pixel1/exptime1/4+pixel2/exptime2/4+pixel3/exptime3/4+pixel4/exptime4/4)*calRGB[G];
					break;
				default:
					break;
				}
			}
		}
	}

	demo.rawrgb[0] = rawpixel_r;
	demo.rawrgb[1] = rawpixel_g;
	demo.rawrgb[2] = rawpixel_b;

	demo.floatrgb[0] = floatpixel_R;
	demo.floatrgb[1] = floatpixel_G;
	demo.floatrgb[2] = floatpixel_B;

	demo.hdrrgb[0] = HDRpixel_R;
	demo.hdrrgb[1] = HDRpixel_G;
	demo.hdrrgb[2] = HDRpixel_B;

	return demo;
}




