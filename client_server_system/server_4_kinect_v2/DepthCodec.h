// Copyright 2017 University of Kentucky
// Po-Chang Su, Ju Shen, Wanxin Xu, Sen-ching Samson Cheung, Ying Luo

#ifndef DepthCodec_H_
#define DepthCodec_H_

// This is a standalone implementation of the kinect's depth codec from OpenNI

#include <stdlib.h>

class DepthCodec
{
public:
	DepthCodec(){};
	~DepthCodec(){};

	// Compression
	// pInput       = Input Depth Data in raster order
	// nInputSize   = Size of Input Depth Data in bytes
	// pOutput      = Output compressed data (should allocate at least nInputSize bytes)
	// pnOutputSize = Size of Output Compress Data in bytes
	// return 0 if okay, -1 otherwise
	int encode(const unsigned short *pInput,
			   const unsigned long nInputSize,
			   unsigned char *pOutput,
			   unsigned long *pnOutputSize);

	// Uncompression
	// pInput       = Input Compressed Data
	// nInputSize   = Size of input compressed data
	// pOutput      = Output Depth Data (high and width must be determined as well)
	// pnOutputSize = Size of Depth Data in bytes (need to set!)
	// return 0 if okay, -1 otherwise
	int decode(const unsigned char *pInput,
			   const unsigned long nInputSize,
			   unsigned short *pOutput,
			   unsigned long *pnOutputSize);
};
#endif
