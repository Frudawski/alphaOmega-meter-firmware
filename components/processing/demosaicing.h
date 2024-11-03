// DEMOSAICING HEADER

#ifndef __DEMOSAICING_H__
#define __DEMOSAICING_H__
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include "processing.h"

// DEFINITIONS

// Demosaicing struct
typedef struct
{
	uint8_t rawrgb[3];
	float floatrgb[3];
	float hdrrgb[3];
}demosaicing;


// FUNCTIONS

demosaicing bilinear_demosaicing(imagebuffer *pic, imagebuffer *expt, imageformat format, uint32_t RESO1, uint32_t RESO2, uint32_t n, float calR, float calG, float calB);

demosaicing downsampling(imagebuffer *pic, imagebuffer *expt, imageformat format, uint32_t RESO1, uint32_t RESO2, uint32_t n, float calR, float calG, float calB);

#endif /* __DEMOSAICING_H__ */


