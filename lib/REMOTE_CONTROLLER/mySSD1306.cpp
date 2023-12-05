#include <mySSD1306.h>

Adafruit_SSD1306 display(OLED_RESET);


///////////////////////////////////////////////////////
//CREATE BITMAP
///////////////////////////////////////////////////////
const unsigned char MrSonBitMap[] PROGMEM = {
    // 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x0c, 0xb3, 0x6c, 0xdb, 0x36, 0xf7, 0xef, 0xbb, 0xbd, 0xec, 0x90, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x0c, 0xb3, 0x6c, 0xdb, 0x36, 0xf7, 0xef, 0xbb, 0xbd, 0xec, 0x94, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x06, 0xcd, 0xb7, 0x6c, 0xcd, 0x99, 0x32, 0xec, 0xcd, 0x26, 0x66, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x0d, 0xdd, 0xb6, 0xed, 0x91, 0x6d, 0xde, 0xdf, 0x77, 0x51, 0x2a, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x09, 0x76, 0xdb, 0xb2, 0x4c, 0x44, 0xd9, 0x93, 0x72, 0x49, 0x2a, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0x66, 0xdb, 0x66, 0x42, 0x12, 0x27, 0x7d, 0x98, 0x88, 0xdb, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x05, 0x9b, 0x6d, 0xd9, 0x90, 0x89, 0xad, 0x66, 0xcd, 0x26, 0xdd, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x0c, 0xfb, 0x6f, 0x49, 0x2d, 0x24, 0x59, 0xba, 0x65, 0x67, 0x67, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x0b, 0x26, 0xf3, 0x6e, 0x67, 0x66, 0x56, 0xcf, 0xb4, 0x99, 0xbb, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xbd, 0x9f, 0xb6, 0xd9, 0x99, 0xa6, 0xd9, 0x9b, 0xb6, 0xdd, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x04, 0xd7, 0x7c, 0xd5, 0x9a, 0x49, 0xa9, 0xb6, 0x6a, 0x4a, 0x67, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x0d, 0x9b, 0xef, 0x76, 0x66, 0x66, 0x5b, 0x36, 0xcc, 0x89, 0x3a, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0x6a, 0xbb, 0x6b, 0x61, 0x32, 0x56, 0xd9, 0xb2, 0x21, 0x1b, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x06, 0x6e, 0xfb, 0xa9, 0x08, 0x09, 0xa4, 0xcf, 0x24, 0x02, 0xc6, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x05, 0xb7, 0xce, 0xbc, 0x80, 0x09, 0x2d, 0x32, 0xcc, 0x02, 0x85, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x0d, 0xb5, 0x7d, 0x92, 0x20, 0x62, 0x59, 0xba, 0x50, 0x03, 0x33, 0x40, 0x00, 0x00, 
	0x00, 0x00, 0x6b, 0x4d, 0xf7, 0x60, 0x20, 0x30, 0xc6, 0xcd, 0x91, 0x11, 0x7e, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x9b, 0x7b, 0xb6, 0x6d, 0x99, 0x18, 0x36, 0x76, 0xac, 0x5d, 0xcd, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0xb4, 0xdb, 0xbb, 0x9d, 0xee, 0xcb, 0x69, 0xb6, 0xcb, 0x66, 0xdb, 0x60, 0x00, 0x00, 
	0x00, 0x00, 0x27, 0xac, 0xed, 0xf3, 0x36, 0xe4, 0x9b, 0xb9, 0x72, 0x2a, 0x77, 0xc0, 0x00, 0x00, 
	0x00, 0x01, 0x4d, 0xb7, 0xed, 0x6c, 0xd3, 0x34, 0xd6, 0xcf, 0x34, 0xab, 0xb5, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x72, 0x76, 0xb7, 0x6f, 0x59, 0x13, 0x6c, 0xf6, 0xcf, 0xb4, 0x9d, 0x60, 0x00, 0x00, 
	0x00, 0x00, 0x9b, 0xdb, 0xb6, 0xd9, 0xcd, 0xdb, 0x6b, 0x3d, 0xb2, 0xd6, 0xdb, 0xc0, 0x00, 0x00, 
	0x00, 0x01, 0x2d, 0x6d, 0xfb, 0x9e, 0x76, 0x6d, 0x9b, 0xdf, 0x6d, 0x5b, 0x6f, 0xe0, 0x00, 0x00, 
	0x00, 0x01, 0x6d, 0x37, 0x4d, 0xe7, 0xb3, 0xb6, 0xf6, 0xdb, 0xdd, 0xd9, 0xfd, 0x60, 0x00, 0x00, 
	0x00, 0x00, 0xd1, 0xdb, 0x76, 0x79, 0xad, 0xb6, 0xb6, 0xfe, 0xd6, 0xb7, 0x37, 0xf0, 0x00, 0x00, 
	0x00, 0x00, 0x98, 0xdb, 0xbb, 0x9e, 0xed, 0xcd, 0x9b, 0x37, 0xfb, 0xb7, 0xee, 0xb0, 0x00, 0x00, 
	0x00, 0x01, 0x68, 0xb6, 0xed, 0xe6, 0xdb, 0x79, 0xdb, 0xdf, 0x6d, 0xdc, 0xee, 0xe0, 0x00, 0x00, 
	0x00, 0x01, 0x4c, 0xed, 0xb6, 0x7b, 0x76, 0x66, 0x6c, 0xfb, 0xf6, 0x6f, 0xbb, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x93, 0x4f, 0x77, 0x9b, 0x6d, 0xbf, 0xb7, 0x2f, 0xb6, 0xed, 0xbf, 0x70, 0x00, 0x00, 
	0x00, 0x00, 0xb3, 0x73, 0x59, 0xed, 0xbb, 0xa4, 0xd7, 0xfd, 0xfb, 0x37, 0x6d, 0xd0, 0x00, 0x00, 
	0x00, 0x00, 0x8c, 0xdd, 0xee, 0x6d, 0xb6, 0xf6, 0x58, 0xd7, 0x6f, 0x9b, 0xf7, 0xf0, 0x00, 0x00, 
	0x00, 0x00, 0x66, 0xdd, 0xb7, 0xb6, 0xdd, 0x5b, 0x6f, 0x76, 0xfd, 0xea, 0xdd, 0xb0, 0x00, 0x00, 
	0x00, 0x00, 0x66, 0xa6, 0xd9, 0xdb, 0x6b, 0x49, 0xb7, 0x6d, 0x9f, 0x6d, 0xbb, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x59, 0x3b, 0x6f, 0x6d, 0xbb, 0xbc, 0xd9, 0xbb, 0x77, 0x95, 0x6e, 0xf0, 0x00, 0x00, 
	0x00, 0x00, 0x59, 0x6d, 0xed, 0xb6, 0xd6, 0xa6, 0x7e, 0xcb, 0x76, 0xd3, 0x6d, 0xb0, 0x00, 0x00, 
	0x00, 0x00, 0x2e, 0x66, 0xb6, 0xf6, 0xdc, 0xeb, 0x66, 0xdd, 0x9e, 0xd9, 0x9b, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x2e, 0x9b, 0xb6, 0xdb, 0x6b, 0x39, 0xbb, 0x36, 0xf3, 0x2c, 0xdb, 0x70, 0x00, 0x00, 
	0x00, 0x00, 0x33, 0xed, 0xdb, 0x6d, 0x6b, 0x4c, 0xbb, 0xa6, 0xc9, 0xa6, 0x6d, 0xd0, 0x00, 0x00, 
	0x00, 0x00, 0x19, 0x26, 0x7b, 0x6d, 0xb5, 0xd6, 0x48, 0xbb, 0x49, 0xb3, 0x26, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x0e, 0xdb, 0xad, 0xb6, 0xb4, 0x93, 0x40, 0x09, 0x26, 0x6c, 0xb6, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x06, 0xd9, 0xed, 0xb6, 0xce, 0x6d, 0xb4, 0x44, 0xb3, 0x6d, 0x9b, 0xa0, 0x00, 0x00, 
	0x00, 0x00, 0x01, 0x26, 0x76, 0xdb, 0x69, 0x6c, 0xa6, 0x56, 0x99, 0x93, 0x2b, 0x60, 0x00, 0x00, 
	0x00, 0x00, 0x01, 0xb7, 0x9b, 0x6d, 0x35, 0x93, 0xb3, 0x33, 0x4c, 0xf6, 0x6d, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xd9, 0xed, 0xb5, 0xb4, 0xdc, 0xd9, 0x8c, 0x67, 0x35, 0x96, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0a, 0x76, 0xdc, 0xca, 0x66, 0x4c, 0xed, 0xb3, 0xcc, 0xce, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x01, 0xb6, 0xd3, 0x6b, 0x33, 0x6e, 0x65, 0x9c, 0xfa, 0x4b, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x6d, 0xbf, 0x31, 0x99, 0xb3, 0x9c, 0xdf, 0x37, 0x6d, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x7b, 0x69, 0x94, 0xcd, 0xb6, 0xf3, 0x63, 0x6d, 0x17, 0x40, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x36, 0xce, 0xe6, 0x66, 0xd9, 0x1b, 0x29, 0x9b, 0x56, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x6d, 0xb6, 0x69, 0x3b, 0x4d, 0x49, 0xac, 0x92, 0x4e, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x3b, 0x35, 0xb1, 0x99, 0x24, 0xec, 0xd6, 0x80, 0xbb, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x36, 0xdd, 0x9c, 0x44, 0xb2, 0x37, 0xdb, 0x81, 0x9b, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x1d, 0xda, 0x64, 0x42, 0x9b, 0xfd, 0xef, 0x40, 0x4d, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x37, 0x67, 0xf6, 0x02, 0x6f, 0xbf, 0xfd, 0xc6, 0x6f, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x1b, 0x3c, 0x99, 0x01, 0xb7, 0xfb, 0xdd, 0x85, 0xb6, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x1d, 0xd3, 0x6d, 0x41, 0xb6, 0xfd, 0xf6, 0x34, 0x9d, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x16, 0xdd, 0xb6, 0x74, 0x5b, 0x37, 0x26, 0xcd, 0xbb, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x1b, 0x6c, 0xd9, 0x96, 0x49, 0x10, 0xc9, 0x9b, 0x36, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x0f, 0x6b, 0x5b, 0x33, 0x36, 0xde, 0xfd, 0xb2, 0x76, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x0d, 0xb6, 0xd6, 0xcd, 0xa6, 0xc9, 0x26, 0x7d, 0xda, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x0d, 0xb6, 0xb4, 0xd9, 0xb9, 0x33, 0x6b, 0xcd, 0x6c, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x0e, 0xcd, 0xaf, 0x36, 0xcd, 0xac, 0xdb, 0x73, 0x6c, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x06, 0xd9, 0x6b, 0x36, 0xf6, 0xcf, 0x9d, 0xdc, 0xd8, 0x00, 0x00, 0x00
};


///////////////////////////////////////////////////////
//FUNCTION TO SETUP THE DISPLAY
///////////////////////////////////////////////////////
void SSD1306_Setup(){
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	display.clearDisplay();
	delay(2000);
}

void DrawMrSonBitMap(){
	display.drawBitmap(0, 0, MrSonBitMap, 256, 64, WHITE);
	display.display();
}