#include "correlation.h"
#include <iostream>

const int R = 0;
const int G = 1;
const int B = 2;

double getRGB(const unsigned char* img, int rgb, int x, int y, int width, int height)
{
	if (x < width && y < height && x >=0 && y >= 0) // check for boundary
	{
		return img[3 * (y * width + x) + rgb];
	}
	else{
		return 0.0000; // out of bound -> return 0.0000
	}
}

/************************ TODO 2 **************************/
/*
 *	INPUT:
 *		origImg:		the original image,
 *		imgWidth:		the width of the image
 *		imgHeight:		the height of the image
 *						the image is arranged such that
 *						origImg[3*(row*imgWidth+column)+0],
 *						origImg[3*(row*imgWidth+column)+1],
 *						origImg[3*(row*imgWidth+column)+2]
 *						are R, G, B values for pixel at (column, row).
 *
 *      kernel:			the 2D filter kernel,
 *		knlWidth:		the width of the kernel
 *		knlHeight:		the height of the kernel
 *
 *		scale, offset:  after correlating the kernel with the origImg,
 *						each pixel should be divided by scale and then added by offset
 *
 *		selection:      a byte array of the same size as the image,
 *						indicating where in the original image should be filtered, e.g.,
 *						selection[k] == 1 ==> pixel k should be filtered
 *                      selection[k] == 0 ==> pixel k should NOT be filtered
 *                      a special case is selection is a NULL pointer, which means all the pixels should be filtered.
 *
 *  OUTPUT:
 *		rsltImg:		the filtered image of the same size as original image.
 *						it is a valid pointer ( allocated already ).
 */

void image_filter(double* rsltImg, const unsigned char* origImg, const unsigned char* selection,
                  int imgWidth, int imgHeight,
                  const double* kernel, int knlWidth, int knlHeight,
                  double scale, double offset)
{

    // Iterate over the image
	for(int row = 0; row < imgHeight; row++)
    {
		for(int col = 0; col < imgWidth; col++)
        {
            // Check if this pixel should be filtered
			if (!selection || selection[row*imgWidth + col])
            {
				pixel_filter(rsltImg + 3*(row * imgWidth + col), //double rsltPixel[3],
							col, //int x, 
							row, //int y, 
							origImg, //const unsigned char* origImg, 
							imgWidth, //int imgWidth, 
							imgHeight, //int imgHeight,
                  			kernel, //const double* kernel, 
                  			knlWidth, //int knlWidth, 
                  			knlHeight, //int knlHeight,
                  			scale, //double scale, 
                  			offset); //double offset)
			}
		}
	}
}

/************************ END OF TODO 2 **************************/


/************************ TODO 3 **************************/
/*
 *	INPUT:
 *      x:				a column index,
 *      y:				a row index,
 *		origImg:		the original image,
 *		imgWidth:		the width of the image
 *		imgHeight:		the height of the image
 *						the image is arranged such that
 *						origImg[3*(row*imgWidth+column)+0],
 *						origImg[3*(row*imgWidth+column)+1],
 *						origImg[3*(row*imgWidth+column)+2]
 *						are R, G, B values for pixel at (column, row).
 *
 *      kernel:			the 2D filter kernel,
 *		knlWidth:		the width of the kernel
 *		knlHeight:		the height of the kernel
 *
 *		scale, offset:  after correlating the kernel with the origImg,
 *						the result pixel should be divided by scale and then added by offset
 *
 *  OUTPUT:
 *		rsltPixel[0], rsltPixel[1], rsltPixel[2]:
 *						the filtered pixel R, G, B values at row y , column x;
 */

void pixel_filter(double rsltPixel[3], int x, int y, const unsigned char* origImg, int imgWidth, int imgHeight,
                  const double* kernel, int knlWidth, int knlHeight,
                  double scale, double offset)
{
    // Determine kernel center
    int kernelCenterX = knlWidth / 2;
    int kernelCenterY = knlHeight / 2;

    // Cooresponding img pixel under currently considered kernel pixel
    int coorImgX = 0;
    int coorImgY = 0;

    // Sum of kernel and image products
    double sumR = 0;
    double sumG = 0;
    double sumB = 0;

    // Loop over the kernel
    for(int kernelRow = 0; kernelRow < knlHeight; kernelRow++)
    {
        for(int kernelCol = 0; kernelCol < knlWidth; kernelCol++)
        {
            // Find cooresponding image pixel
            coorImgY = y + (kernelRow - kernelCenterY);
            coorImgX = x + (kernelCol - kernelCenterX);

            // Add the kernel image product
            sumR += kernel[kernelRow * knlWidth + kernelCol] * getRGB(origImg, R, coorImgX, coorImgY, imgWidth, imgHeight);
            sumG += kernel[kernelRow * knlWidth + kernelCol] * getRGB(origImg, G, coorImgX, coorImgY, imgWidth, imgHeight);
            sumB += kernel[kernelRow * knlWidth + kernelCol] * getRGB(origImg, B, coorImgX, coorImgY, imgWidth, imgHeight);
        }
    }

    // Apply scale and offset and place results in rsltPixel
    rsltPixel[0] = (sumR / scale) + offset;
    rsltPixel[1] = (sumG / scale) + offset;
    rsltPixel[2] = (sumB / scale) + offset;
}

/************************ END OF TODO 3 **************************/


/*
	newKnlWidth = knlWidth;
	newKnlHeight = knlHeight;
	// pad kernel with an extra column of 0.0000s if the width is even
	if (knlWidth % 2 == 0){newKnlWidth += 1;}

	// pad kernel with an extra row of 0.0000 if the height is even
	if (knlHeight % 2 == 0){newKnlHeight += 1;}

	// reconstruct the kernel
	double* newKnl = new double[newKnlWidth * newKnlHeight]();
	for(int i = 0 ; i < knlWidth ; ++i){
		for(int j = 0 ; j < knlHeight; ++j){
			newKnl[i * knlWidth + j] = kernel[i * knlWidth + j];
		}
	}

	int center_x = newKnlWidth / 2;
	int center_y = newKnlHeight / 2;

	for(int i = -1 * newKnlWidth/2; i <= newKnlWidth/2 ; ++i)
	{
		for(int j = -1 * newKnlHeight/2; j <= newKnlHeight/2; ++j)
		{
			newR += newKnl[(centerX+i)*(centerY+j)] * getRGB(origImg, R, x+i, y+j, imgWidth, imgHeight);
			newG += newKnl[(centerX+i)*(centerY+j)] * getRGB(origImg, G, x+i, y+j, imgWidth, imgHeight);
			newB += newKnl[(centerX+i)*(centerY+j)] * getRGB(origImg, B, x+i, y+j, imgWidth, imgHeight);
		}
	}
*/
