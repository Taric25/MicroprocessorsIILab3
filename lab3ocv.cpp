/*--------------- lab3ocv.cpp ---------------
by:	Mr. Ryan D Aldrich
	Mr. Taric S Alani
	Mr. John A Nicholson
EECE.5520-201 Microprocessors II
ECE Dept.
UMASS Lowell
Based on specification by Dr. Yan Luo:
lab3i2cimage.pdf 2016-11-02 18:31:52

PURPOSE
Understand the I2C bus protocol and use it to control an 
I2C device using Linux on the bus protocol. Another 
objective is to capture, store, and process camera images
in Linux. Connect to an I2C temperature sensor and gesture
sensor and use them to trigger the camera to take a picture.      
*/

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <fcntl.h>

using namespace cv;
using namespace std;

/* I2C address defines */
#define TEMP_ADDR             0x48   // temp sensor address
#define GEST_ADDR             0x39   // gesture sensor device address

/* Gesture sensor register defines */
#define APDS9960_ENABLE	      0x80 	 // gesture enable register
#define APDS9960_ATIME	      0x81   // ADC integration time register
#define APDS9960_WTIME	      0x83   // wait time register
#define APDS9960_AILTL        0x84	 // ALS interrupt low threshold low byte
#define APDS9960_AILTH        0x85   // ALS interrupt low threshold high byte
#define APDS9960_AIHTL        0x86   // ALS interrupt high threshold low byte
#define APDS9960_AIHTH        0x87   // ALS interrupt high threshold high byte
#define APDS9960_PILT	      0x89   // Proximity interrupt low threshold register
#define APDS9960_PIHT         0x8B   // Proximity interrupt high threshold register
#define APDS9960_PERS	      0x8C   // Interupt persistence filters (non-gesture)
#define APDS9960_CONFIG1      0x8D	 // Configuration register one
#define APDS9960_PPULSE   	  0x8E   // Proximity pulse count and length register
#define APDS9960_CONTROL      0x8F   // Gain control register
#define APDS9960_CONFIG2      0x90   // Configuration register two
#define APDS9960_ID           0x92   // Device ID register
#define APDS9960_STATUS       0x93   // Gesture device status register
#define APDS9960_CDATAL       0x94   // Low byte of clear channel data register
#define APDS9960_CDATAH       0x95   // High byte of clear channel data register
#define APDS9960_PDATA    	  0x9C   // Proximity data register
#define APDS9960_POFFSET_UR	  0x9D   // Proximity offset for UP and RIGHT photodiodes
#define APDS9960_POFFSET_DL   0x9E	 // Proximity offset for DOWN and LEFT photodiodes
#define APDS9960_CONFIG3	  0x9F   // Configuration register three
#define APDS9960_GPENTH       0xA0   // Gesture proximity enter threshold register
#define APDS9960_GEXTH        0xA1   // Gesture exit threshold register
#define APDS9960_GCONF1       0xA2   // Gesture configuration one
#define APDS9960_GCONF2       0xA3   // Gesture configuration two register
#define APDS9960_GOFFSET_U	  0xA4   // Gesture up offset register
#define APDS9960_GOFFSET_D	  0xA5   // Gesture down offset register
#define APDS9960_GOFFSET_L    0xA7   // Gesture left offset register
#define APDS9960_GOFFSET_R    0xA9   // Gesture right offset register
#define APDS9960_GPULSE       0xA6   // Gesture pulse count and length
#define APDS9960_GCONF3       0xAA   // Gesture configuration three register
#define APDS9960_GCONF4       0xAB   // Gesture configuration four register
#define APDS9960_GFLVL        0xAE   // Gesture FIFO level register
#define APDS9960_GSTATUS      0xAF	 // Gesture status register
#define APDS9960_IFORCE       0xE4   // Force interrupt register
#define APDS9960_PICLEAR      0xE5   // Proximity interrupt clear register
#define APDS9960_CICLEAR      0xE6   // ALS clear channel interrupt clear register
#define APDS9960_AICLEAR      0xE7   // All non-gesture interrupts clear register
#define APDS9960_GFIFO_U      0xFC   // Gesture FIFO UP value
#define APDS9960_GFIFO_D      0xFD   // Gesture FIFO DOWN value
#define APDS9960_GFIFO_L      0xFE   // Gesture FIFO LEFT value
#define APDS9960_GFIFO_R      0xFF   // Gesture FIFO RIGHT value

//global variables
int fd;
int r;
int s;

// write to gesture sensor configuration registers
void gestureInit(void)
{
	unsigned char data[] = {0,0};
	data[0] = APDS9960_ENABLE;
	data[1] = 0b01011111; // gesture enable, wait enable, proximity detect enable, and power on
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_ATIME;
	data[1] = 0xDB;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_WTIME;
	data[1] = 0xF6;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_PPULSE;
	data[1] = 0x87;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_POFFSET_UR;
	data[1] = 0x00;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_POFFSET_DL;
	data[1] = 0x00;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_CONFIG1;
	data[1] = 0x60;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_PERS;
	data[1] = 0x11;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
     	
	data[0] = APDS9960_CONFIG2;
	data[1] = 0x01;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_CONFIG3;
	data[1] = 0x00;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_GCONF1;
	data[1] = 0x60;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_GOFFSET_U;
	data[1] = 0x00;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_GOFFSET_D;
	data[1] = 0x00;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_GOFFSET_L;
	data[1] = 0x00;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_GOFFSET_R;
	data[1] = 0x00;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_GPULSE;
	data[1] = 0xC9;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_GCONF2;
	data[1] = 0x01;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_GCONF3;
	data[1] = 0;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_GCONF4;
	data[1] = 0;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_GPENTH;
	data[1] = 40;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_GEXTH;
	data[1] = 30;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_PILT;
	data[1] = 0;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_PIHT;
	data[1] = 50;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
	
	data[0] = APDS9960_CONTROL;
	data[1] = 0x05;
	s = write(fd,data,2);
	if (s != 2){printf("Failed to write to the i2c bus.\n");}
}

// open video capture and store image to a file	
void photoCap(void)
{
	VideoCapture cap(0); // open the video camera no. 0

	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

	cout << "Frame Size = " << dWidth << "x" << dHeight << endl;  // display size of the image

	vector<int> compression_params; //vector that stores the compression parameters of the image
	compression_params.push_back(IMWRITE_PNG_COMPRESSION); //specify the compression technique
	compression_params.push_back(0); //specify the compression quality
  
	Mat img(dHeight, dWidth, CV_8UC4);  //create an image with dHeight x dWidth size and CV_8UC4 array type

	bool bSuccess1 = cap.read(img); // read a new frame from video and store it in img
		 
	bool bSuccess2 = imwrite("/media/card/Image.png", img, compression_params); //write the image to file
	
	sleep(5);
}


int main(int argc, char* argv[])
{
	// declare variables
    unsigned short temp_val[2] = {0,0};
    useconds_t delay = 2000;
    float temp_extval = 0.00;
	unsigned short pdata[2] = {0,0};
	
	char *dev = "/dev/i2c-0";
	
	printf("Sensor capture running\n");
	
	// open the I2C channel
	fd = open(dev, O_RDWR );
	if(fd < 0)
	{
		perror("Opening i2c device node\n");
		return 1;
	}
	
	// select gesture sensor as the I2C slave device
	s = ioctl(fd, I2C_SLAVE, GEST_ADDR);
	if(s < 0)
	{
		perror("Selecting i2c device\n");
	}
	
	// initialize gesture sensor
	gestureInit();
	
	// close I2C channel
	close(fd);
	
	while(1)
	{
		// open the I2C channel
		fd = open(dev, O_RDWR );
		if(fd < 0)
		{
			perror("Opening i2c device node\n");
			return 1;
		}
		
		// select temperature sensor as the I2C slave device
		r = ioctl(fd, I2C_SLAVE, TEMP_ADDR);
		if(r < 0)
		{
			perror("Selecting i2c device\n");
		}

		// read out data bytes from the temperature sensor
		for(unsigned char i = 0; i < 2; i++)
        {
            r = read(fd, &temp_val[i], 2);
            if(r != 2)
            {
                perror("reading i2c device\n");
            }
            usleep(delay);
        }
		
		// print out temperature sensor values
		printf("Temperature in bytes %x\n", temp_val[0] & 0xFFF);
		temp_val[0] = temp_val[0] << 4;
		printf("Temperature in bytes, shifted 4 bits %x\n", temp_val[0] & 0xFFF);
		temp_extval = ((temp_val[0]*0.0625) - 5.0);
		printf("Temperature in celsius with decimals, %f\n", temp_extval);
		temp_extval = (((temp_extval*9.0)/5.0)+32.0);
		printf("Temperature in fahrenheit, %f\n", temp_extval);
		usleep(5000);
		
		// close I2C channel
		close(fd);
		
		// open the I2C channel
		fd = open(dev, O_RDWR );
		if(fd < 0)
		{
			perror("Opening i2c device node\n");
			return 1;
		}
		
		// select the gesture sensor as the I2C slave device
		s = ioctl(fd, I2C_SLAVE, GEST_ADDR);
		if(s < 0)
		{
			perror("Selecting i2c device\n");
		}
		
		// read out data bytes from the gesture sensor
		for(int i=0;i<2;i++)
		{
			pdata[0] = APDS9960_PDATA;
			pdata[1] = 0x00; 
			r = read(fd, &pdata[i], 2);
			if(r != 2)
			{
				perror("reading i2c device\n");
			}
			usleep(50000);
		}
		
		// print out gesture sensor proximity data
		printf("Proximity data is: %x\n",pdata[1]);
		
		// close the I2C channel
		close(fd);
		
		// if the temperature is above 70 degrees or the proximity data is within this range take a picture
		if ((temp_extval > 70.0) || (pdata[1] < 0x9999 && pdata[1] > 0x8888))
		{
			usleep(10000);
			photoCap();
			printf("Temperature exceeds 70F or object detected too close\nPhoto Captured, stored in:\n/media/card/\n");
		}
		
	}

    return 0;

}
