#ifndef _BMP_
#define _BMP_
#include "utils.hpp"
#include "v3.hpp"
typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned int dword;

struct BITMAPFILEHEADER {
	dword bfSize;
	word bfReserved1;
	word bfReserved2;
	dword bfOffBits;
};

struct BITMAPINFOHEADER {
	dword biSize;
	long biWidth;
	long biHeight;
	word biPlanes;
	word biBitCount;
	dword biCompression;
	dword biSizeImage;
	long biXPelsPerMeter;
	long biYPelsPerMeter;
	dword biClrUsed;
	dword biClrImportant;
};

struct RGBQUAD {
	byte rgbBlue;
	byte rgbGreen;
	byte rgbRed;
	byte rgbReserved;
};

struct IMAGEDATA {
	byte red;
	byte green;
	byte blue;
	V3 GetColor() {
		return V3( red , green , blue ) / 256;
	}
};

struct Bmp {
    //二维
    V3 **rgb = nullptr;
	BITMAPFILEHEADER header;
	BITMAPINFOHEADER info;
	Bmp(char *fn) {
		read(fn);
	}
    void Initialize( int H , int W ) {
	    header.bfReserved1 = 0;
	    header.bfReserved2 = 0;
	    header.bfOffBits = 54;

	    info.biSize = 40;
	    info.biPlanes = 1;
	    info.biHeight = H;
	    info.biWidth = W;
	    info.biBitCount = 24;
	    info.biCompression = 0;
	    info.biSizeImage = H * W * 3;
	    info.biXPelsPerMeter = 0;
	    info.biYPelsPerMeter = 0;
    	info.biClrUsed = 0;
    	info.biClrImportant = 0;

    	header.bfSize = info.biSizeImage + info.biBitCount;
}

    bool read(char *fn) {
    	FILE *fpi = fopen(fn, "rb" );
	    word bfType;
	    fread(&bfType , 1 , sizeof( word ) , fpi );
	    fread(&header , 1 , sizeof( BITMAPFILEHEADER ) , fpi );
	    fread(&info , 1 , sizeof( BITMAPINFOHEADER ) , fpi );
	
	    RGBQUAD Pla;
    	for ( int i = 0 ; i < ( int ) info.biClrUsed ; i++ ) {
	    	fread( ( char * ) & ( Pla.rgbBlue ) , 1 , sizeof( byte ) , fpi );
	    	fread( ( char * ) & ( Pla.rgbGreen ) , 1 , sizeof( byte ) , fpi );
		    fread( ( char * ) & ( Pla.rgbRed ) , 1 , sizeof( byte ) , fpi );
    	}
	
        std::cout << info.biHeight << std::endl;
        std::cout << info.biWidth << std::endl;
	    Initialize( info.biHeight , info.biWidth );
        IMAGEDATA **ima;
	    for(int i = 0 ; i < info.biHeight ; i++ )
	    	for(int j = 0 ; j < info.biWidth ; j++ ) {
			    fread( &ima[i][j].blue , 1 , sizeof( byte ) , fpi );
	    		fread( &ima[i][j].green , 1 , sizeof( byte ) , fpi );
		    	fread( &ima[i][j].red , 1 , sizeof( byte ) , fpi );

	    	}

    	fclose( fpi );
    }
};


#endif
