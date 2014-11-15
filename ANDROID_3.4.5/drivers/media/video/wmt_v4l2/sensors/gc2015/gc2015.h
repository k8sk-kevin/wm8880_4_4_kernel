#ifndef GC2015_H
#define GC2015_H


// Scene Mode
uint8_t gc2015_scene_mode_auto[] = {
};

uint8_t gc2015_scene_mode_night[] = {
};


// White Balance
uint8_t gc2015_wb_auto [] = {
};

uint8_t gc2015_wb_incandescent [] = {
};

uint8_t gc2015_wb_fluorescent [] = {
};

uint8_t gc2015_wb_daylight [] = {
};

uint8_t gc2015_wb_cloudy [] = {
};

uint8_t gc2015_wb_tungsten [] = {
};


// Exposure
uint8_t gc2015_exposure_neg6[] = {
};

uint8_t gc2015_exposure_neg3[] = {
};

uint8_t gc2015_exposure_zero[] = {
};

uint8_t gc2015_exposure_pos3[] = {
};

uint8_t gc2015_exposure_pos6[] = {
};


// Color Effect
uint8_t gc2015_colorfx_none[] = {
};

uint8_t gc2015_colorfx_bw[] = {
};

uint8_t gc2015_colorfx_sepia[] = {
};

uint8_t gc2015_colorfx_negative[] = {
};

uint8_t gc2015_colorfx_emboss[] = {
};

uint8_t gc2015_colorfx_sketch[] = {
};

uint8_t gc2015_colorfx_sky_blue[] = {
};

uint8_t gc2015_colorfx_grass_green[] = {
};

uint8_t gc2015_colorfx_skin_whiten[] = {
};

uint8_t gc2015_colorfx_vivid[] = {
};

// Brightness
uint8_t gc2015_brightness_neg4[] = {
};

uint8_t gc2015_brightness_neg3[] = {
};

uint8_t gc2015_brightness_neg2[] = {
};

uint8_t gc2015_brightness_neg1[] = {
};

uint8_t gc2015_brightness_zero[] = {
};

uint8_t gc2015_brightness_pos1[] = {
};

uint8_t gc2015_brightness_pos2[] = {
};

uint8_t gc2015_brightness_pos3[] = {
};

uint8_t gc2015_brightness_pos4[] = {
};

// Contrast
uint8_t gc2015_contrast_neg4[] = {
};

uint8_t gc2015_contrast_neg3[] = {
};

uint8_t gc2015_contrast_neg2[] = {
};

uint8_t gc2015_contrast_neg1[] = {
};

uint8_t gc2015_contrast_zero[] = {
};

uint8_t gc2015_contrast_pos1[] = {
};

uint8_t gc2015_contrast_pos2[] = {
};

uint8_t gc2015_contrast_pos3[] = {
};

uint8_t gc2015_contrast_pos4[] = {
};

// Saturation
uint8_t gc2015_saturation_neg4[] = {
};

uint8_t gc2015_saturation_neg3[] = {
};

uint8_t gc2015_saturation_neg2[] = {
};

uint8_t gc2015_saturation_neg1[] = {
};

uint8_t gc2015_saturation_zero[] = {
};

uint8_t gc2015_saturation_pos1[] = {
};

uint8_t gc2015_saturation_pos2[] = {
};

uint8_t gc2015_saturation_pos3[] = {
};

uint8_t gc2015_saturation_pos4[] = {
};


// Resolution

// QCIF
uint8_t gc2015_176x144[]={
};

// QVGA
uint8_t gc2015_320x240[]={
0xfe, 0x80,
0xfe, 0x80,
0xfe, 0x80,
0xfe, 0x00,
0x45, 0x00,
0x02, 0x01,
0x2a, 0xca,
0x48, 0x40,
0x59, 0x11,
0x50, 0x01,
0x51, 0x00,
0x52, 0x00,
0x53, 0x00,
0x54, 0x00,
0x55, 0x02,
0x56, 0x58,
0x57, 0x03,
0x58, 0x20,
0x59, 0xaa,
0x5b, 0x01,
0x5c, 0x34,
0x5d, 0x00,
0x5e, 0x00,
0x5f, 0x01,
0x60, 0x34,
0x61, 0x00,
0x62, 0x00,
0xfe, 0x01,
0xb0, 0x13,
0xb1, 0x20,
0xb2, 0x20,
0xb3, 0x20,
0xb4, 0x20,
0xb5, 0x20,
0xb6, 0x20,
0xb7, 0x00,
0xb8, 0x80,
0xb9, 0x80,
0xba, 0x13,
0xbb, 0x20,
0xbc, 0x20,
0xbd, 0x20,
0xbe, 0x20,
0xbf, 0x20,
0xc0, 0x20,
0xc1, 0x00,
0xc2, 0x80,
0xc3, 0x80,
0xfe, 0x00,
0x29, 0x00,
0x2b, 0x06,
0x32, 0x0c,
0x33, 0x0f,
0x34, 0x00,
0x35, 0x88,
0x37, 0x16,
0x40, 0xff,
0x41, 0x24,
0x42, 0x76,
0x4b, 0xea,
0x4d, 0x03,
0x4f, 0x01,
0x63, 0x77,
0x66, 0x00,
0x6d, 0x04,
0x6e, 0x18,
0x70, 0x18,
0x6f, 0x10,
0x71, 0x10,
0x73, 0x03,
0x80, 0x07,
0x82, 0x08,
0x8a, 0x7c,
0x8c, 0x02,
0x8e, 0x02,
0x8f, 0x48,
0xb0, 0x44,
0xb1, 0xfe,
0xb2, 0x00,
0xb3, 0xf8,
0xb4, 0x48,
0xb5, 0xf8,
0xb6, 0x00,
0xb7, 0x04,
0xb8, 0x00,
0xbF, 0x0E,
0xc0, 0x1C,
0xc1, 0x34,
0xc2, 0x48,
0xc3, 0x5A,
0xc4, 0x6B,
0xc5, 0x7B,
0xc6, 0x95,
0xc7, 0xAB,
0xc8, 0xBF,
0xc9, 0xCE,
0xcA, 0xD9,
0xcB, 0xE4,
0xcC, 0xEC,
0xcD, 0xF7,
0xcE, 0xFD,
0xcF, 0xFF,
0xCF, 0xFF,
0xd1, 0x38,
0xd2, 0x38,
0xde, 0x21,
0x98, 0x30,
0x99, 0xf0,
0x9b, 0x00,
0xfe, 0x01,
0x10, 0x45,
0x11, 0x32,
0x13, 0x60,
0x17, 0x00,
0x1c, 0x96,
0x1e, 0x11,
0x21, 0xc0,
0x22, 0x40,
0x2d, 0x06,
0x2e, 0x00,
0x1e, 0x32,
0x33, 0x00,
0x57, 0x40,
0x5d, 0x44,
0x5c, 0x35,
0x5e, 0x29,
0x5f, 0x50,
0x60, 0x50,
0x65, 0xc0,
0x80, 0x82,
0x81, 0x00,
0x83, 0x00,
0xfe, 0x00,
0x45, 0x0f,
0x46, 0x02,
//0x46, 0x03,
0x44, 0xa2,
0xfe, 0x00,
0xfe, 0x01,
0x33, 0x20,
0xfe, 0x00,
0x43, 0x00,
0x42, 0x74,
0x7a, 0x5f,
0x7b, 0x40,
0x7c, 0x47,
0x42, 0x76,
0xfe, 0x01,
0x13, 0x60,
0xfe, 0x00,
0xd5, 0x00,
0x05, 0x01,
0x06, 0xc1,
0x07, 0x00,
0x08, 0x40,
0xfe, 0x01,
0x29, 0x01,
0x2a, 0x00,
0x2b, 0x05,
0x2c, 0x00,
0x2d, 0x06,
0x2e, 0x00,
0x2f, 0x08,
0x30, 0x00,
0x31, 0x09,
0x32, 0x00,
0xfe, 0x00,
0xfe, 0x01,
0x33, 0x20,
0xfe, 0x00,
};

// CIF
uint8_t gc2015_352x288[]={
};

// VGA
uint8_t gc2015_640x480[]={
	0xfe,0x00,
	0x02,0x01,
	0x2a,0xca,
	0x59,0x55,
	0x5a,0x06,
	0x5b,0x00,
	0x5c,0x00,
	0x5d,0x01,
	0x5e,0x23,
	0x5f,0x00,
	0x60,0x00,
	0x61,0x01,
	0x62,0x23,
	0x50,0x01,
	0x51,0x00,
	0x52,0x00,
	0x53,0x00,
	0x54,0x00,
	0x55,0x01,
	0x56,0xe0,
	0x57,0x02,
	0x58,0x80,
	0x48,0x68,
	0x4f,0x01,
};

// SVGA
uint8_t gc2015_800x600[]={
};

// XGA
uint8_t gc2015_1024x768[]={
};

// 720p
uint8_t gc2015_1280x720[]={
};

// UXGA
uint8_t gc2015_1600x1200[]={
0x4f,0x00,
0x42,0x74,
0xfe,0x00,
0x02,0x00,
0x2a,0x0a,
0x59,0x11,
0x5a,0x06,
0x5b,0x00,
0x5c,0x00,
0x5d,0x00,
0x5e,0x00,
0x5f,0x00,
0x60,0x00,
0x61,0x00,
0x62,0x00,
0x50,0x01,
0x51,0x00,
0x52,0x00,
0x53,0x00,
0x54,0x00,
0x55,0x04,
0x56,0xb0,
0x57,0x06,
0x58,0x40,
0x48,0x60,
0x0,0x0,
0x12,0x1,
0x13,0x2a,
0x6e,0x1b,
0x6f,0x20,
0x70,0x1b,
0x71,0x20,
};


// Initiliztion
uint8_t gc2015_default_regs_init[] = {
0xfe, 0x80,
0xfe, 0x80,
0xfe, 0x80,
0xfe, 0x00,
0x45, 0x00,
0x02, 0x01,
0x2a, 0xca,
0x48, 0x40,
0x59, 0x11,
0x50, 0x01,
0x51, 0x00,
0x52, 0x00,
0x53, 0x00,
0x54, 0x00,
0x55, 0x02,
0x56, 0x58,
0x57, 0x03,
0x58, 0x20,
0x59, 0x55,
0x5b, 0x01,
0x5c, 0x34,
0x5d, 0x00,
0x5e, 0x00,
0x5f, 0x01,
0x60, 0x34,
0x61, 0x00,
0x62, 0x00,
0xfe, 0x01,
0xb0, 0x13,
0xb1, 0x20,
0xb2, 0x20,
0xb3, 0x20,
0xb4, 0x20,
0xb5, 0x20,
0xb6, 0x20,
0xb7, 0x00,
0xb8, 0x80,
0xb9, 0x80,
0xba, 0x13,
0xbb, 0x20,
0xbc, 0x20,
0xbd, 0x20,
0xbe, 0x20,
0xbf, 0x20,
0xc0, 0x20,
0xc1, 0x00,
0xc2, 0x80,
0xc3, 0x80,
0xfe, 0x00,
0x29, 0x00,
0x2b, 0x06,
0x32, 0x0c,
0x33, 0x0f,
0x34, 0x00,
0x35, 0x88,
0x37, 0x16,
0x40, 0xff,
0x41, 0x24,
0x42, 0x76,
0x4b, 0xea,
0x4d, 0x03,
0x4f, 0x01,
0x63, 0x77,
0x66, 0x00,
0x6d, 0x04,
0x6e, 0x18,
0x70, 0x18,
0x6f, 0x10,
0x71, 0x10,
0x73, 0x03,
0x80, 0x07,
0x82, 0x08,
0x8a, 0x7c,
0x8c, 0x02,
0x8e, 0x02,
0x8f, 0x22,//48
0xb0, 0x44,
0xb1, 0xfe,
0xb2, 0x00,
0xb3, 0xf8,
0xb4, 0x48,
0xb5, 0xf8,
0xb6, 0x00,
0xb7, 0x04,
0xb8, 0x00,
0xbF, 0x0E,
0xc0, 0x1C,
0xc1, 0x34,
0xc2, 0x48,
0xc3, 0x5A,
0xc4, 0x6B,
0xc5, 0x7B,
0xc6, 0x95,
0xc7, 0xAB,
0xc8, 0xBF,
0xc9, 0xCE,
0xcA, 0xD9,
0xcB, 0xE4,
0xcC, 0xEC,
0xcD, 0xF7,
0xcE, 0xFD,
0xcF, 0xFF,
0xCF, 0xFF,
0xd1, 0x38,
0xd2, 0x38,
0xde, 0x21,
0x98, 0x30,
0x99, 0xf0,
0x9b, 0x00,
0xfe, 0x01,
0x10, 0x45,
0x11, 0x32,
0x13, 0x70,//60
0x17, 0x00,
0x1c, 0x96,
0x1e, 0x11,
0x21, 0xc0,
0x22, 0x40,
0x2d, 0x06,
0x2e, 0x00,
0x1e, 0x32,
0x33, 0x00,
0x57, 0x40,
0x5d, 0x44,
0x5c, 0x35,
0x5e, 0x29,
0x5f, 0x50,
0x60, 0x50,
0x65, 0xc0,
0x80, 0x82,
0x81, 0x00,
0x83, 0x00,
0xfe, 0x00,
0x45, 0x0f,
0x46, 0x02,
//0x46, 0x03,
0x44, 0xa2,
0xfe, 0x00,
0xfe, 0x01,
0x33, 0x20,
0xfe, 0x00,
0x43, 0x00,
0x42, 0x74,
0x7a, 0x5f,
0x7b, 0x40,
0x7c, 0x47,
0x42, 0x76,
0xfe, 0x01,
0x13, 0x70,//60
0xfe, 0x00,
0xd5, 0x00,
0x05, 0x01,
0x06, 0xc1,
0x07, 0x00,
0x08, 0x40,
0xfe, 0x01,
0x29, 0x01,
0x2a, 0x00,
0x2b, 0x05,
0x2c, 0x00,
0x2d, 0x06,
0x2e, 0x00,
0x2f, 0x08,
0x30, 0x00,
0x31, 0x09,
0x32, 0x00,
0xfe, 0x00,
0xfe, 0x01,
0x33, 0x20,
0xfe, 0x00,
};

#endif