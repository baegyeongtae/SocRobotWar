#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <math.h>
#include <time.h>
#include <getopt.h>
#include <termios.h>

#include "amazon2_sdk.h"
#include "graphic_api.h"
#include "robot_protocol.h"
#include "uart_api.h"
 
enum {
	WIDTH = 180,
	HEIGHT = 120
};

#define MAX_3(a, b, c)	( ((a)>(b)) ? ( (((a)>(c)) ? (a) : (c)) ) : ( (((b)>(c)) ? (b) : (c)) ) )
#define MIN_3(a, b, c)	( ((a)>(b)) ? ( (((b)>(c)) ? (c) : (b)) ) : ( (((a)>(c)) ? (c) : (a)) ) )
 ////////////////////////////////////////////////////////////
#define EXTRACT_R565(c)	(unsigned char)((c & 0x7c00) >> 10)
#define EXTRACT_G565(c)	(unsigned char)((c & 0x03e0) >> 5)
#define EXTRACT_B565(c)	(unsigned char)(c & 0x1f)
#define EXTRACT_BLACK(c)(unsigned char)(c>>15)

////////////////////////////////////////////////////////////
unsigned short fpga_videodata[WIDTH*HEIGHT] = { 0, };
//unsigned short videodata[WIDTH*HEIGHT] = { 0, };
///////////////////////////////////////////////////////
unsigned char r[WIDTH*HEIGHT] = { 0, };
unsigned char g[WIDTH*HEIGHT] = { 0, };
unsigned char b[WIDTH*HEIGHT] = { 0, };
////////////////////////////////////////////////////////
unsigned char binary_black[WIDTH*HEIGHT] = { 0, };
unsigned char hole_x = 0;
unsigned char ball_x = 0;
unsigned char kick = 0;
unsigned char check_1 = 0;

static struct termios inittio, newtio;

void init_consolve(void);
void Dely(void);
void select_motion(int Item);
void fpga_videodata_read(int x, int x_reg, int y, int y_reg);

void center_line(void);
void center_line_mine(void);
void center_line_yellow(void);
void barricade_1(void);
void mine(void);
void green_bridge_ready(void);
void green_bridge_run(void);
void green_bridge_end(void);
void golf_ready(void);
void golf_ready_1(void);
void golf_ready_1_1(void);
void kick_the_ball(void);
void hole(void);
void yellow_bridge_ready_center(void);
void crevasse(void);
void barricade_2(void);

void init_console(void)
{
	tcgetattr(0, &inittio);
	newtio = inittio;
	newtio.c_lflag &= ~ICANON;
	newtio.c_lflag &= ~ECHO;
	newtio.c_lflag &= ~ISIG;
	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] = 0;

	cfsetispeed(&newtio, B115200);

	tcsetattr(0, TCSANOW, &newtio);
}

void Delay(clock_t n)
{
	clock_t start;
	clock_t end;

	start = clock();

	while (clock() - start < n);
	{}

	end = clock();

//	////printf("\n\n\n Time : %f \n", ((double)(end - start)) / CLOCKS_PER_SEC);
}

void select_motion(int Item) // Motion Function : Item에 모션 번호 넣으면 됨.
{
	unsigned char state_buffer[1] = { 0, };
	unsigned short buffer[1] = { 0, };

	switch (Item)
	{
	case 1:
		motion_1();
		uart1_buffer_read(state_buffer, 1);	
		while (1)
		{
			if (buffer[0] == 100)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 2:
		motion_2();
		uart1_buffer_read(state_buffer, 1);	
		while (1)
		{
			if (buffer[0] == 1700)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 3:
		motion_3();
		uart1_buffer_read(state_buffer, 1);		
		while (1)
		{
			if (buffer[0] == 2700)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 4:
		motion_4();
		uart1_buffer_read(state_buffer, 1);		
		while (1)
		{
			if (buffer[0] == 4300)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 5:
		motion_5();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 4500)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 6:
		motion_6();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 1300)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 7:
		motion_7();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 800)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 8:
		motion_8();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 800)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 9:
		motion_9();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 1500)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 10:
		motion_10();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 1500)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 11:
		motion_11();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 1000)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 12:
		motion_12();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 100)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 13:
		motion_13();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 300)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 14:
		motion_14();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 800)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 15:
		motion_15();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 100)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 16:
		motion_16();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 3200)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("motion_16 : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 17:
		motion_17();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 5000)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 18:
		motion_18();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 4800)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 19:
		motion_19();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 4100)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 20:
		motion_20();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 4000)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 21:
		motion_21();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 400)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 22:
		motion_22();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 400)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 23:
		motion_23();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 400)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 24:
		motion_24();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 400)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 25:
		motion_25();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 1500)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 26:
		motion_26();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 650)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 27:
		motion_27();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 300)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 28:
		motion_28();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 650)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 29:
		motion_29();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 850)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 30:
		motion_30();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 500)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 31:
		motion_31();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 100)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 32:
		motion_32();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 300)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 33:
		motion_33();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 6500)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 34:
		motion_34();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 700)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 35:
		motion_35();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 200)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}	
		break;
	case 36:
		motion_36();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 100)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 37:
		motion_37();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 500)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 38:
		motion_38();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 800)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 39:
		motion_39();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 900)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	case 40:
		motion_40();
		uart1_buffer_read(state_buffer, 1);
		while (1)
		{
			if (buffer[0] == 5500)
			{
				break;
			}
			else
			{
				uart1_buffer_read(state_buffer, 1);
				printf("Current State : %d \n ", buffer[0]);
			}
			buffer[0] += 1;
		}
		break;
	default:
		break;
	}	
}

void fpga_videodata_read(int x, int x_reg, int y, int y_reg)
{
	int xx = 0, yy = 0;
	//int c = 0, w = 0;
	//int i, scan, tmp;
	//unsigned short filter[8] = { 0, };

	direct_camera_display_on();
	direct_camera_display_off();
	read_fpga_video_data(fpga_videodata);
	clear_screen();	
	/*
	for (yy = 1; yy < 119; yy++)
	{
		for (xx = 1; xx < 179; xx++)
		{
			for (c = 0; c < 3; c++)
			{
				for (w = 0; w < 3; w++)
				{
					filter[w + c * 3] = fpga_videodata[(xx - 1 + w) + (yy - 1 + c)*WIDTH];
				}
			}
						
			for (scan = 0; scan < 8; scan++)
			{
				for (i = 0; i < 8; i++)
				{
					if (filter[i] > filter[i + 1])
					{
						tmp = filter[i];
						filter[i] = filter[i + 1];
						filter[i + 1] = tmp;
					}
				}
			}
			videodata[xx + yy*WIDTH] = filter[4];
			//printf("videodata : %x\n", videodata[xx + yy*WIDTH]);
		}
	}
	*/
	for (xx = x; xx <= x_reg; xx++)
	{
		for (yy = y; yy <= y_reg; yy++)
		{
			//r[xx + yy*WIDTH] = (EXTRACT_R565(fpga_videodata[xx + yy*WIDTH]) * 527 + 23) >> 6;
			//g[xx + yy*WIDTH] = (EXTRACT_G565(fpga_videodata[xx + yy*WIDTH]) * 259 + 33) >> 6;
			//b[xx + yy*WIDTH] = (EXTRACT_B565(fpga_videodata[xx + yy*WIDTH]) * 527 + 23) >> 6;			

			r[xx + yy*WIDTH] = EXTRACT_R565(fpga_videodata[xx + yy*WIDTH]);
			g[xx + yy*WIDTH] = EXTRACT_G565(fpga_videodata[xx + yy*WIDTH]);
			b[xx + yy*WIDTH] = EXTRACT_B565(fpga_videodata[xx + yy*WIDTH]);
			binary_black[xx+yy*WIDTH] = EXTRACT_BLACK(fpga_videodata[xx + yy*WIDTH]);
			//printf("r : %d  g : %d  b : %d binary : %d\n", r[xx + yy*WIDTH], g[xx + yy*WIDTH], b[xx + yy*WIDTH], binary_black[xx + yy*WIDTH]);
		}
	}
	//direct_camera_display_on();
}

/*
void CvtHSV(int x, int x_reg, int y, int y_reg)
{
	int xx = 0, yy = 0;
	unsigned char min, max, delta;

	fpga_videodata_read(x, x_reg, y, y_reg);
	
	for (xx = x; xx < x_reg; xx++) {	
		for (yy = y; yy < y_reg; yy++) {

			min = MIN_3(r[xx + yy*WIDTH],g[xx + yy*WIDTH],b[xx + yy*WIDTH]);
			max = MAX_3(r[xx + yy*WIDTH],g[xx + yy*WIDTH],b[xx + yy*WIDTH]);			
			
			v[xx + yy*WIDTH] = max;                                // v
			delta = max - min;

			if (v[xx + yy*WIDTH] == 0) { // NOTE: if Max is == 0, this divide would cause a crash
				h[xx + yy*WIDTH] = 0;    
				s[xx + yy*WIDTH] = 0; 
				//return;
			}
			
			s[xx + yy*WIDTH] = 255 * (long)(max - min) / v[xx + yy*WIDTH];

			if (s[xx + yy*WIDTH] == 0)
			{
				h[xx + yy*WIDTH] = 0;
			}
			
			if( r[xx + yy*WIDTH] == max )
			{
				h[xx + yy*WIDTH] = 0 + 43 * (g[xx + yy*WIDTH] - b[xx + yy*WIDTH]) / delta;
			}// between yellow & magenta

	        else if( g[xx + yy*WIDTH] == max )
			{
		       h[xx + yy*WIDTH] = 85 + 43 * ( b[xx + yy*WIDTH] - r[xx + yy*WIDTH] ) / delta;
			} // between cyan & yellow

	        else
			{
		        h[xx + yy*WIDTH] = 171 + 43 * ( r[xx + yy*WIDTH] - g[xx + yy*WIDTH] ) / delta;
			} // between magenta & cyan				
	
			//printf("h : %d   s : %d   v :%d \n", h[xx + yy*WIDTH], s[xx + yy*WIDTH], v[xx + yy*WIDTH]);				
		}
	}	
	
}
*/
void mine(void)
{
	int x = 0, y = 0;
	int x_f = 0, y_f = 0;
	int blue = 0;	
	int counter_1 = 0, counter_2 = 0, counter_3 = 0, counter_4 = 0;
	int select = 0;
	int a = 0;

	while (1)
	{
		clear_screen();
		Delay(100000);
		select_motion(37);
		Delay(250000);
				
		fpga_videodata_read(50, 140, 10, 100);
		
		x_f = 0;
		y_f = 0;
	
		for (y = 100; y >= 10; y--)
		{
			for (x = 50; x <= 140; x++)
			{
				if ((binary_black[x + WIDTH * y] == 0) && (x_f == 0) && (y_f == 0))
				{
					x_f = x;
					y_f = y;
				}				
			}
		}

		for (y = 100; y >= 10; y--)
		{
			for (x = 50; x <= 140; x++)
			{
				if ((binary_black[x + WIDTH * y] == 0))
				{
					//count++;
					fpga_videodata[x + WIDTH * y] = 0x0;
				}			

				if ((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 31))
				{
					fpga_videodata[x + WIDTH * y] = 0x001f;
					blue++;
				}
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		printf("x_f : %d  y_f : %d  blue : %d\n", x_f, y_f, blue);

		if (blue > 200)
		{
			select = binary_black[(x_f + 10) + y_f * WIDTH] == 0 ? 1 : 0;
			
			if (select == 1)
			{
				select_motion(25);
				Delay(250000);
				select_motion(5);
				Delay(250000);
				select_motion(38);
				Delay(250000);
				select_motion(38);
				Delay(250000);
				break;
			}
		}

		if (y_f < 40)
		{
			int i = 0;

			if (a == 0)
			{
				//printf("noting\n");
				select_motion(25);
			}
			if (a > 0)
			{
				//printf("passing!!!\n");
				center_line_mine();
				select_motion(4);
				a = 0;
			}
		
			
			if (counter_1 > 0)
			{
				for (i = 0; i < counter_1; i++)
				{
					select_motion(9);
					Delay(150000);
				}
				counter_1 = 0;
			}
			else if (counter_2 > 0)
			{
				for (i = 0; i < counter_2; i++)
				{
					select_motion(22);
					Delay(150000);
				}
				counter_2 = 0;
			}
			else if (counter_3 > 0)
			{
				for (i = 0; i < counter_3; i++)
				{
					select_motion(10);
					Delay(150000);
				}
				counter_3 = 0;
			}
			else if (counter_4 > 0)
			{
				for (i = 0; i < counter_4; i++)
				{
					select_motion(21);
					Delay(150000);
				}
				counter_4 = 0;
			}

			Delay(200000);			
		}

		else if ((y_f >= 40) && (y_f < 65))
		{
			//printf("ing_3~~~~~~\n");
			select_motion(3);
			Delay(200000);
		}

		else if ((y_f >= 65) && (y_f < 75))
		{
			//printf("ing_2~~~~~~\n");
			select_motion(2);
			Delay(200000);
		}

		else if (y_f >= 75)
		{
			//printf("stop!!!!\n");
			if ((x_f >= 95) && (x_f <= 123))
			{
				//printf("left_5cm\n");
				select_motion(10);
				counter_1++;				
				Delay(200000);
				a++;
			}
			else if ((x_f > 123) && (x_f <= 140))
			{
				//printf("left_20\n");
				select_motion(21);
				counter_2++;
				Delay(200000);	
				a++;
			}
			else if ((x_f < 95) && (x_f >= 67))
			{
				//printf("right_5cm\n");
				select_motion(9);				
				counter_3++;
				Delay(200000);
				a++;
			}
			else if ((x_f < 67) && (x_f >= 50))
			{
				//printf("right_20\n");
				select_motion(22);
				counter_4++;
				Delay(200000);		
				a++;
			}			
		}		
	}		
}

void center_line(void) 
{
	int x = 0, y = 0;
	int y_f = 0, y_l = 0;
	int differ_1 = 0;
	int blue = 0;

	while (1)
	{	
		clear_screen();
		Delay(100000);
		select_motion(28);
		Delay(100000);
				
		fpga_videodata_read(50, 90, 5, HEIGHT - 10);
		y_f = 0;
		y_l = 0;
		blue = 0;
				
		for (y = HEIGHT - 10; y >= 5; y--)
		{
			if ((binary_black[51 + WIDTH * y] == 0) && (y_f == 0))
			{
				y_f = y;
			}
			else if ((binary_black[89 + WIDTH * y] == 0) && (y_l == 0))
			{
				y_l = y;
			}
		}
		
		for (y = 5; y <= 110; y++)
		{
			for (x = 50; x <= 90; x++)
			{
				if (binary_black[x + WIDTH * y] == 0)
				{
					fpga_videodata[x + WIDTH * y] = 0xf800;
				}
				else
					fpga_videodata[x + WIDTH * y] = 0xffff;
			}
		}

		for (y = 5; y <= 110; y++)
		{
			for (x = 50; x <= 90; x++)
			{
				if ((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 31))
				{
					blue++;
				}
			}
		}		
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		printf("y_f : %d   y_l : %d\n", y_f, y_l);
		differ_1 = y_l - y_f;	
		
		if (blue > 200)
		{
			select_motion(1);
			Delay(250000);
			select_motion(25);
			Delay(250000);
			continue;
		}
		
		if ((differ_1 <= 4) && (differ_1 >= 0))  ///기울지 않는 상태
		{
			if ((y_f >= 47) && (y_f <= 52))
			{
				printf("center!!\n");
				select_motion(1);
				break;
			}			
			else if (y_f > 57)
			{				
				select_motion(21);
				Delay(200000);
				select_motion(21);
				Delay(200000);
				select_motion(21);
				Delay(200000);				
			}		
			else if ((y_f > 52) && (y_f <= 57))
			{
				select_motion(21);
				Delay(200000);				
			}
			else if (y_f < 42)
			{
				select_motion(22);
				Delay(200000);
				select_motion(22);
				Delay(200000);
				select_motion(22);
				Delay(200000);			
			}
			else if ((y_f >= 42) && (y_f < 47))
			{
				select_motion(22);
				Delay(200000);				
			}
		}
		else if ((differ_1 > 4) && (differ_1 <= 10))
		{
			select_motion(14);
			Delay(200000);
		}
		else if (differ_1 > 10)
		{
			select_motion(14);
			Delay(200000);
			select_motion(14);
			Delay(200000);
		}
		else if ((differ_1 < 0) && (differ_1 >= -5))
		{
			select_motion(8);
			Delay(200000);
		}
		else if (differ_1 < -5)
		{
			select_motion(8);
			Delay(200000);
			select_motion(8);
			Delay(200000);
		}
	}	
}

void center_line_mine(void)
{
	int x = 0, y = 0;
	int y_f = 0, y_l = 0;
	int differ_1 = 0;
	
	while (1)
	{
		clear_screen();
		Delay(100000);
		select_motion(28);
		Delay(100000);

		fpga_videodata_read(50, 90, 5, HEIGHT - 10);
		y_f = 0;
		y_l = 0;
		
		for (y = HEIGHT - 10; y >= 5; y--)
		{
			if ((binary_black[51 + WIDTH * y] == 0) && (y_f == 0))
			{
				y_f = y;
			}
			else if ((binary_black[89 + WIDTH * y] == 0) && (y_l == 0))
			{
				y_l = y;
			}
		}

		for (y = 5; y <= 110; y++)
		{
			for (x = 50; x <= 90; x++)
			{
				if (binary_black[x + WIDTH * y] == 0)
				{
					fpga_videodata[x + WIDTH * y] = 0xf800;
				}
				else
					fpga_videodata[x + WIDTH * y] = 0xffff;
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		printf("y_f : %d   y_l : %d\n", y_f, y_l);
		differ_1 = y_l - y_f;
				
		if ((differ_1 <= 4) && (differ_1 >= 0))  ///기울지 않는 상태
		{
			break;
		}
		else if ((differ_1 > 4) && (differ_1 <= 10))
		{
			select_motion(14);
			Delay(200000);
		}
		else if (differ_1 > 10)
		{
			select_motion(14);
			Delay(200000);
			select_motion(14);
			Delay(200000);
		}
		else if ((differ_1 < 0) && (differ_1 >= -5))
		{
			select_motion(8);
			Delay(200000);
		}
		else if (differ_1 < -5)
		{
			select_motion(8);
			Delay(200000);
			select_motion(8);
			Delay(200000);
		}
	}
}

void center_line_yellow(void)
{
	int x = 0, y = 0;
	int y_f = 0, y_l = 0;
	int differ_1 = 0;
	int blue = 0;

	while (1)
	{
		clear_screen();
		Delay(100000);
		select_motion(28);
		Delay(100000);

		fpga_videodata_read(50, 90, 5, HEIGHT - 10);
		y_f = 0;
		y_l = 0;
		blue = 0;

		for (y = HEIGHT - 10; y >= 5; y--)
		{
			if ((binary_black[51 + WIDTH * y] == 0) && (y_f == 0))
			{
				y_f = y;
			}
			else if ((binary_black[89 + WIDTH * y] == 0) && (y_l == 0))
			{
				y_l = y;
			}
		}

		for (y = 5; y <= 110; y++)
		{
			for (x = 50; x <= 90; x++)
			{
				if (binary_black[x + WIDTH * y] == 0)
				{
					fpga_videodata[x + WIDTH * y] = 0xf800;
				}
				else
					fpga_videodata[x + WIDTH * y] = 0xffff;
			}
		}

		for (y = 5; y <= 110; y++)
		{
			for (x = 50; x <= 90; x++)
			{
				if ((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 31))
				{
					blue++;
				}
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		printf("y_f : %d   y_l : %d\n", y_f, y_l);
		differ_1 = y_l - y_f;

		if (blue > 200)
		{
			select_motion(1);
			Delay(250000);
			select_motion(25);
			Delay(250000);
			continue;
		}

		if ((differ_1 <= 4) && (differ_1 >= 0))  ///기울지 않는 상태
		{
			if ((y_f >= 45) && (y_f <= 52))
			{
				printf("center!!\n");
				select_motion(1);
				break;
			}
			else if (y_f > 57)
			{
				select_motion(21);
				Delay(200000);
				select_motion(21);
				Delay(200000);
				select_motion(21);
				Delay(200000);
			}
			else if ((y_f > 52) && (y_f <= 57))
			{
				select_motion(21);
				Delay(200000);
			}
			else if (y_f < 42)
			{
				select_motion(22);
				Delay(200000);
				select_motion(22);
				Delay(200000);
				select_motion(22);
				Delay(200000);
			}
			else if ((y_f >= 42) && (y_f < 45))
			{
				select_motion(22);
				Delay(200000);
			}
		}
		else if ((differ_1 > 4) && (differ_1 <= 10))
		{
			select_motion(14);
			Delay(200000);
		}
		else if (differ_1 > 10)
		{
			select_motion(14);
			Delay(200000);
			select_motion(14);
			Delay(200000);
		}
		else if ((differ_1 < 0) && (differ_1 >= -5))
		{
			select_motion(8);
			Delay(200000);
		}
		else if (differ_1 < -5)
		{
			select_motion(8);
			Delay(200000);
			select_motion(8);
			Delay(200000);
		}
	}
}

void ex(void) //색깔 검출 연습용 함수
{
	int x = 0, y = 0;	
	int y_f = 0, y_l = 0;
	int dot = 0;
	int count = 0;

	while (1)
	{
		y_f = 0;
		y_l = 0;
		dot = 0;
		clear_screen();
		select_motion(1);
		fpga_videodata_read(5, 175, 10, 115);

		for (y = 10; y <= 115; y++)
		{
			for (x = 5; x <= 175; x++)
			{
				if ((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 31)) // blue
				{
					fpga_videodata[x + WIDTH * y] = 0x001f;
					count++;
				}
				
				else
				{
					fpga_videodata[x + WIDTH*y] = 0xffff;
				}				
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		printf("dot : %d\n", dot);		
		
		
		if (count >= 50000)
			break;
	
	}
}

void green_bridge_ready(void) 
{
	int x = 0, y = 0;
	int x_f = 0, y_f = 0;
	int greendot = 0;
	//int count = 0;
	
	while (1)
	{
		y_f = 0;
		clear_screen();
		Delay(300000);
		select_motion(39);
		Delay(300000);
		fpga_videodata_read(5, 175, 5, 60);
		
		for (y = 60; y >= 5; y--)
		{
			for (x = 5; x <= 175; x++)
			{
				if ((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0) && (y_f == 0))
				{
					fpga_videodata[x + y*WIDTH] = 0x07e0;
					y_f = y;
					greendot++;
				}

				else if ((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0) && (y_f != 0))
				{
					fpga_videodata[x + y*WIDTH] = 0x07e0;
					greendot++;
				}
				else
					fpga_videodata[x + y*WIDTH] = 0xffff;
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		if ((y_f == 0) || (greendot < 200))
		{
			select_motion(25);
			Delay(200000);
		}
		else if (greendot > 200)
		{
			select_motion(25);
			Delay(250000);
			select_motion(38);
			break;
		}
	}

	while (1)
	{
		x_f = 0;
		clear_screen();
		Delay(300000);
		select_motion(1);
		Delay(300000);
		fpga_videodata_read(5, 175, 30, 110);
		
		for (y = 110; y >= 30; y--)
		{
			for (x = 5; x <= 175; x++)
			{
				if ((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0) && (x_f == 0))
				{
					fpga_videodata[x + y*WIDTH] = 0x07e0;
					x_f = x;
					y_f = y;
				}

				else if ((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0) && (x_f != 0))
				{
					fpga_videodata[x + y*WIDTH] = 0x07e0;
					//count++;
				}
				else
					fpga_videodata[x + y*WIDTH] = 0xffff;
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		printf("x_f : %d   y_f : %d\n", x_f, y_f);	
		
		if ((x_f >= 56) && (x_f <= 63)) /////center
		{
			Delay(250000);
			select_motion(38);
			Delay(250000);
			select_motion(38);
			Delay(600000);
			select_motion(19);
			Delay(250000);
			select_motion(3);
			break;
		}
		else if ((x_f > 63) && (x_f <= 68))
		{
			select_motion(32);
			Delay(200000);
			select_motion(38);
		}
		else if ((x_f > 68) && (x_f <= 105))
		{
			select_motion(22);
			Delay(200000);
			select_motion(38);
		}
		else if (x_f > 105)
		{
			select_motion(22);
			Delay(200000);
			select_motion(22);
			Delay(200000);
			select_motion(38);
		}
		else if ((x_f >= 51) && (x_f < 56))
		{
			select_motion(27);
			Delay(200000);
			select_motion(38);
		}
		else if ((x_f >= 20) && (x_f < 51))
		{
			select_motion(21);
			Delay(200000);
			select_motion(38);
		}
		else if (x_f < 20)
		{
			select_motion(21);
			Delay(200000);
			select_motion(21);
			Delay(200000);
			select_motion(38);
		}
		/*
		if (count >= 100000)
			break;
			*/
	}
}

void green_bridge_run(void) 
{
	int x = 0, y = 0;
	int x_right_1 = 0, x_right_2 = 0;
	int x_left_1 = 0, x_left_2 = 0;	
	int differ_1 = 0, differ_2 = 0;
	int break_point = 0;

	while (1)
	{
		clear_screen();
		select_motion(37);
		Delay(250000);
		
		fpga_videodata_read(25, 165, 10, 105);

		x_right_1 = 0;
		x_right_2 = 0;
		x_left_1 = 0;
		x_left_2 = 0;
		differ_1 = 0;
		differ_2 = 0;
		
		for (y = 105; y >= 10; y--)
		{
			for (x = 25; x <= 165; x++)
			{
				if (((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0)) && (y != 40) && (y != 90))// || ((r[(x + 1) + y*WIDTH] == 0) && (g[(x + 1) + y*WIDTH] == 31) && (b[(x + 1) + y*WIDTH] == 0) && (y != 40) && (y != 90)))
				{
					fpga_videodata[x + y*WIDTH] = 0x07e0;					
				}
				else if (((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0)) && (y == 40) && (x_right_1 == 0))// || ((r[(x + 1) + y*WIDTH] == 0) && (g[(x + 1) + y*WIDTH] == 31) && (b[(x + 1) + y*WIDTH] == 0) && (y == 40) && (x_right_1 == 0)))
				{
					fpga_videodata[x + y*WIDTH] = 0x07e0;
					x_right_1 = x;					
				}
				else if (((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0)) && (y == 90) && (x_right_2 == 0))// || ((r[(x + 1) + y*WIDTH] == 0) && (g[(x + 1) + y*WIDTH] == 31) && (b[(x + 1) + y*WIDTH] == 0) && (y == 90) && (x_right_2 == 0)))
				{
					fpga_videodata[x + y*WIDTH] = 0x07e0;
					x_right_2 = x;					
				}
				else if (((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0)) && (y == 40)) //|| ((r[(x + 1) + y*WIDTH] == 0) && (g[(x + 1) + y*WIDTH] == 31) && (b[(x + 1) + y*WIDTH] == 0) && (y == 40)))
				{
					fpga_videodata[x + y*WIDTH] = 0x07e0;					
				}
				else if (((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0)) && (y == 90)) //|| ((r[(x + 1) + y*WIDTH] == 0) && (g[(x + 1) + y*WIDTH] == 31) && (b[(x + 1) + y*WIDTH] == 0) && (y == 90)))
				{
					fpga_videodata[x + y*WIDTH] = 0x07e0;					
				}				
			}
		}		
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		for (x = 150; x >= 40; x--)
		{
			if (((fpga_videodata[x + 40 * WIDTH] == 0x07e0) && (x_left_1 == 0)) || ((fpga_videodata[(x + 1) + 40 * WIDTH] == 0x07e0) && (x_left_1 == 0)))
			{
				x_left_1 = x;
			}
			else if (((fpga_videodata[x + 90 * WIDTH] == 0x07e0) && (x_left_2 == 0)) || ((fpga_videodata[(x + 1) + 90 * WIDTH] == 0x07e0) && (x_left_2 == 0)))
			{
				x_left_2 = x;
			}
		}

		differ_1 = x_right_2 - x_right_1;
		differ_2 = x_left_2 - x_left_1;

		printf("x_right_1 : %d, x_right_2 : %d  x_left_1 : %d, x_left_2 : %d\n", x_right_1, x_right_2, x_left_1, x_left_2);
		printf("differ_1 : %d  differ_2 : %d\n", differ_1, differ_2);

		if (break_point == 3)
		{
			select_motion(2);
			break;
		}

		if ((differ_2 >= 1) && (differ_2 <= 7)) ///////////기울지 않음
		{
			if ((x_right_1 >= 47) && (x_right_1 <= 52))// && (x_left_1 >= 126) && (x_left_1 <= 129)) ////////center    ////(x_right_1 >= 48) && (x_right_1 <= 50)
			{
				break_point++;
				select_motion(5);
				Delay(200000);
			}
			else if ((x_right_1 > 52) && (x_right_1 <= 55))// && (x_left_1 <= 129) && (x_left_1 > 135)) /////////왼쪽으로 쳐짐
			{
				select_motion(32);
				Delay(250000);
				select_motion(32);
				Delay(250000);
			}
			else if ((x_right_1 > 55))// && (x_left_1 > 135)) /////////왼쪽으로 쳐짐
			{
				select_motion(22);
				Delay(250000);				
			}
			else if ((x_right_1 < 47) && (x_right_1 >= 44))// && (x_left_1 >= 121) && (x_left_1 < 126))   ////////////오른쪽으로 쳐짐
			{
				select_motion(27);
				Delay(200000);
				select_motion(27);
				Delay(200000);
			}
			else if ((x_right_1 < 44))// && (x_left_1 < 124))   ////////////오른쪽으로 쳐짐
			{
				select_motion(21);
				Delay(200000);				
			}
		}
		else if (differ_2 > 7)// && (differ_2 <= 13))
		{
			select_motion(8);
			Delay(200000);
		}		
		else if (differ_2 < 1)
		{
			select_motion(14);
			Delay(200000);
		}				
	}
}

void green_bridge_end(void) 
{
	int x = 0, y = 0;
	int y_f = 0, y_l = 0, y_c = 0;	
	int differ = 0;
	int not_turn = 0;

	while (1)
	{
		y_f = 0;
		y_l = 0;
		y_c = 0;
		differ = 0;
		clear_screen();
		select_motion(37);
		Delay(300000);
		fpga_videodata_read(60, 130, 60, 105);

		for (x = 60; x <= 130; x++)
		{
			for (y = 60; y <= 105; y++)
			{
				if ((binary_black[60 + y*WIDTH] == 0) && (y_f == 0))
				{
					fpga_videodata[x + y*WIDTH] = 0x0;
					y_f = y;
				}
				if ((binary_black[130 + y*WIDTH] == 0) && (y_l == 0))
				{
					fpga_videodata[x + y*WIDTH] = 0x0;
					y_l = y;
				}
				if ((binary_black[80 + y*WIDTH] == 0) && (y_c == 0))
				{
					fpga_videodata[x + y*WIDTH] = 0x0;
					y_c = y;
				}
			}
		}
		
		for (x = 60; x <= 130; x++)
		{
			for (y = 60; y <= 105; y++)
			{
				if (binary_black[x + y*WIDTH] == 0)
				{
					fpga_videodata[x + y*WIDTH] = 0x0;						
				}
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		differ = y_l - y_f;
		printf("y_f : %d   y_l : %d\n", y_f, y_l);
		
		if (((differ <= 3) && (differ >= 0)) || (y_f == 0) || (y_c == 0))
		{
			printf("go~~~!!!\n");

			if ((y_f <= 95) && (y_f >= 60))
			{
				printf("ing~~~~!!!!!\n");
				select_motion(2);
				Delay(250000);
			}
			else if (y_f > 95)
			{
				printf("close~~~~!!!!!\n");
			    select_motion(38);
				Delay(250000);
				select_motion(38);
				Delay(250000);
				not_turn++;
			}
			
			else if (y_f == 0)
			{
				printf("the end!!!\n");
				break;
			}
		}

		else if (((differ > 3) && (not_turn == 0) && (y_f != 0)) || (y_c != 0))
		{
			printf("turn left~~~~~~!!!!\n");
			select_motion(14);
			Delay(250000);
		}

		else if (((differ < 0) && (not_turn == 0) && (y_f != 0)) || (y_c != 0))
		{
			printf("turn right~~~~~~!!!!\n");
			select_motion(8);
			Delay(250000);
		}
	}
	select_motion(20);
}

void golf_ready()
{
	int x = 0, y = 0;
	int x_f = 0, x_l = 0;
	unsigned char y_f = 0;

	while (1)
	{
		printf("golf_ready!!!\n");
		clear_screen();
		select_motion(23);		
		Delay(250000);
		fpga_videodata_read(10, 170, 10, 110);		

		x_f = 0;
		x_l = 0;
		y_f = 0;		
		ball_x = 0;			

		for (y = 110; y >= 10; y--)
		{
			for (x = 10; x <= 170; x++)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 0) && (y_f == 0))
				{					
					x_f = x;
					y_f = y;
				}
			}
		}		

		for (y = 10; y <= 110; y++)
		{
			for (x = 170; x >= 10; x--)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 0) && (x_l == 0))
				{
					x_l = x;
				}
			}
		}

		for (y = 10; y <= 110; y++)
		{
			for (x = 170; x >= 10; x--)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 0))
				{
					fpga_videodata[x + y*WIDTH] = 0xf800;
				}
				else
					fpga_videodata[x + y*WIDTH] = 0xffff;
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();		
		
		ball_x = (x_f + x_l) / 2;		
		printf("y_f : %d\n", y_f);
		
		if (y_f < 80)
		{
			select_motion(2);
			Delay(250000);
		}

		else
		{			
			break;					
		}			
	}	
}

void golf_ready_1(void)
{
	int x = 0, y = 0;
	int x_f = 0, x_l = 0;	
	int differ = 0;

	while (1)
	{
		printf("golf_ready_1\n");
		clear_screen();
		select_motion(23);
		Delay(250000);
		fpga_videodata_read(10, 170, 10, 110);

		x_f = 0;
		x_l = 0;		
		//ball_x = 0;

		for (y = 110; y >= 10; y--)
		{
			for (x = 10; x <= 170; x++)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 0) && (x_f == 0))
				{
					x_f = x;					
				}
			}
		}

		for (y = 10; y <= 110; y++)
		{
			for (x = 170; x >= 10; x--)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 0) && (x_l == 0))
				{
					x_l = x;
				}
			}
		}

		for (y = 10; y <= 110; y++)
		{
			for (x = 170; x >= 10; x--)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 0))
				{
					fpga_videodata[x + y*WIDTH] = 0xf800;
				}
				else
					fpga_videodata[x + y*WIDTH] = 0xffff;
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		differ = x_l - x_f;
		ball_x = (x_f + x_l) / 2;
		printf("ball_x : %d\n", ball_x);
		//printf("x_f : %d, x_l : %d\n", x_f, x_l);

		if (abs(differ) > 13)
			continue;
			
		if ((ball_x < 110) && (ball_x > 104))
		{
			printf("right_1~~~~~~\n");
			select_motion(32);
			Delay(300000);
		}
		else if (ball_x >= 110)
		{
			printf("right_20~~~~~~\n");
			select_motion(22);
			Delay(300000);
		}
		else if ((ball_x > 95) && (ball_x < 103))
		{
			printf("left_1~~~~~~\n");
			select_motion(27);
			Delay(300000);
		}
		else if (ball_x <= 93)
		{
			printf("left_20~~~~~~\n");
			select_motion(21);
			Delay(300000);
		}
		else
		{
			printf("center~~~~~~~\n");
			break;
		}
		
	}
}

void golf_ready_1_1(void)
{
	int x = 0, y = 0;
	int y_f = 0;

	while (1)
	{
		printf("golf_ready_1_1\n");
		clear_screen();
		select_motion(23);
		Delay(250000);
		fpga_videodata_read(10, 170, 10, 110);
				
		y_f = 0;
		//ball_x = 0;

		for (y = 110; y >= 10; y--)
		{
			for (x = 10; x <= 170; x++)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 0) && (y_f == 0))
				{					
					y_f = y;
				}
			}
		}

		for (y = 10; y <= 110; y++)
		{
			for (x = 170; x >= 10; x--)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 0))
				{
					fpga_videodata[x + y*WIDTH] = 0xf800;
				}
				else
					fpga_videodata[x + y*WIDTH] = 0xffff;
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		printf("y_f : %d\n", y_f);

		if ((y_f >= 83) && (y_f < 100))
		{
			select_motion(38);
			Delay(300000);
		}
		else if (y_f < 80)
		{
			select_motion(2);
			Delay(250000);
		}
		else
		{
			kick++;
			break;
		}
	}
}

void golf_ready_ex(void)
{
	int x = 0, y = 0;
	int x_f = 0, x_l = 0;
	int y_f = 0;

	while (1)
	{
		clear_screen();
		select_motion(23);
		Delay(250000);
		fpga_videodata_read(10, 170, 10, 110);

		x_f = 0;
		x_l = 0;
		y_f = 0;
		ball_x = 0;

		for (y = 110; y >= 10; y--)
		{
			for (x = 10; x <= 170; x++)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 0) && (y_f == 0))
				{
					x_f = x;
					y_f = y;
				}
			}
		}

		for (y = 10; y <= 110; y++)
		{
			for (x = 170; x >= 10; x--)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 0) && (x_l == 0))
				{
					x_l = x;
				}
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		ball_x = (x_f + x_l) / 2;
		printf("ball_x : %d\n", ball_x);
		break;
	}
}

void hole(void) //홀보고 값찾기
{
	int x = 0, y = 0;
	int x_f = 0, x_l = 0;	
	//int differ = 0;
	int differ_x = 0;

	while (1)
	{
		printf("hole!!!\n");
		clear_screen();
		Delay(300000);
		select_motion(24);
		Delay(300000);
		fpga_videodata_read(5, 175, 20, 115);

		hole_x = 0;				
		x_f = 0;
		x_l = 0;		
		differ_x = 0;

		for (x = 5; x <= 175; x++)
		{
			for (y = 115; y >= 20; y--)
			{
				if ((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 31) && (x_f == 0))
					x_f = x;					
			}
		}
		
		for (x = 175; x >= 5; x--)
		{
			for (y = 20; y <= 115; y++)
			{
				if ((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 31) && (x_l == 0))
					x_l = x;				
			}
		}

		for (x = 175; x >= 5; x--)
		{
			for (y = 20; y <= 115; y++)
			{
				if ((r[x + y*WIDTH] == 0) && (g[x + y*WIDTH] == 0) && (b[x + y*WIDTH] == 31))
					fpga_videodata[x + y*WIDTH] = 0x001f;
				else
					fpga_videodata[x + y*WIDTH] = 0xffff;
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();
		
		hole_x = (x_f + x_l) / 2;
		//differ = x_l - x_f;

		if (x_f == 0)
			continue;
		
		printf("x_f : %d,  x_l : %d\n", x_f, x_l);
		printf("hole --> hole_x : %d\n", hole_x);

		//golf_ready_ex();
		//differ_x = hole_x - ball_x;
		//printf("differ_x : %d\n", differ_x);
		break;
	}
}

void kick_the_ball()
{
	int differ_x = 0;
	int first = 0;	

	while (1)
	{
		differ_x = 0;

		switch (check_1)
		{
		case 0:
			if (first == 0)
				golf_ready();
			golf_ready_1();
			break;
		case 1:
			golf_ready_1();
			golf_ready_1_1();
			break;		
		default:
			break;
		}		
		////////////////////////////  대회때 hole 위치보고 다시 수정하기  ///////////////////////////////
		/*
		if (first == 0)
		{
			select_motion(14);
			Delay(250000);
			select_motion(14);    /////////// 홀이 로봇기준 오른쪽!!!!!!!!
			Delay(250000);
			first++;
			continue;
		}
		*/
		
		if (first == 0)
		{
			select_motion(8);
			Delay(250000);
			select_motion(8);      /////////// 홀이 로봇기준 왼쪽!!!!!!!!
			Delay(250000);
			first++;
			continue;
		}
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////
		hole();

		differ_x = hole_x - ball_x;
		printf("differ_x : %d\n", differ_x);		

		if (kick >= 1) ///////////////// kick_the_ball !!!!!!!!
		{

			/////////////    <홀이 오른쪽에 있는 경우>   ///////////////////////
			/*
			if ((differ_x > 3) && (differ_x <= 30)) //////////////// hole & ball location !!!!!
			{
				select_motion(14);
				Delay(250000);
				continue;
			}

			else if (differ_x > 30) //////////////// hole & ball location !!!!!
			{
				select_motion(14);
				Delay(250000);
				select_motion(14);
				Delay(250000);
				continue;
			}
			*/

			///////////////   <홀이 왼쪽에 있는 경우>   /////////////////////////
			if ((differ_x < -5) && (differ_x > -10))
			{
				select_motion(7);
				Delay(250000);
				continue;
			}

			else if (differ_x <= -10)
			{
				select_motion(8);
				Delay(250000);
				continue;
			}

			else if ((differ_x > 5) && (differ_x <= 30)) //////////////// hole & ball location !!!!!
			{
				select_motion(14);
				Delay(250000);
				continue;
			}
			/////////////////////////////////////////////////////////////////////

			else   ///////////  kick !!!!!!
			{
				select_motion(30);
				break;
			}			
		}

	            	/////////////    <홀이 오른쪽에 있는 경우>   ///////////////////////
		/*
		if ((differ_x > 3) && (differ_x <= 30)) //////////////// hole & ball location !!!!!
		{
			select_motion(14);
			Delay(250000);
			continue;
		}

		else if (differ_x > 30) //////////////// hole & ball location !!!!!
		{
			select_motion(14);
			Delay(250000);
			select_motion(14);
			Delay(250000);
			continue;
		}
		*/
		///////////////   <홀이 왼쪽에 있는 경우>   /////////////////////////
		if ((differ_x < -5) && (differ_x >= -30))
		{
			select_motion(8);
			Delay(250000);
			continue;
		}

		else if (differ_x < -30)
		{
			select_motion(8);
			Delay(250000);
			select_motion(8);
			Delay(250000);
			continue;
		}
		
		else   ///////////  hole & ball same line -> next levels !!!!!!
		{
			check_1++;
			continue;
		}
	}
}

void yellow_bridge_ready_center(void)
{
	int x = 0, y = 0;
	int yellow_dot = 0;
	int y_f = 0, y_l = 0;
	int blackdot = 0;

	while (1)
	{		
		yellow_dot = 0;
		blackdot = 0;
		clear_screen();
		Delay(300000);
		select_motion(39);
		Delay(300000);
		fpga_videodata_read(5, 175, 5, 60);

		for (y = 60; y >= 5; y--)
		{
			for (x = 5; x <= 175; x++)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0))
				{
					fpga_videodata[x + y*WIDTH] = 0xffe0;
					yellow_dot++;
				}
			
				else if (binary_black[x + y*WIDTH] == 0)
				{
					fpga_videodata[x + y*WIDTH] = 0xffe0;
					blackdot++;
				}
				else
					fpga_videodata[x + y*WIDTH] = 0xffff;
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		if (yellow_dot < 200)
		{
			select_motion(25);
			Delay(250000);
			select_motion(25);
			Delay(250000);
		}
		else if ((yellow_dot > 200) || (blackdot > 100))
		{
			select_motion(25);
			Delay(250000);
			select_motion(25);
			Delay(250000);
			select_motion(3);
			Delay(250000);
			select_motion(38);
			break;
		}
	}
	int differ_1 = 0;

	while (1)
	{
		clear_screen();
		Delay(100000);
		select_motion(28);
		Delay(100000);

		fpga_videodata_read(50, 90, 5, HEIGHT - 10);
		y_f = 0;
		y_l = 0;

		for (y = HEIGHT - 10; y >= 5; y--)
		{
			if ((binary_black[51 + WIDTH * y] == 0) && (y_f == 0))
				y_f = y;
			else if ((binary_black[89 + WIDTH * y] == 0) && (y_l == 0))
				y_l = y;		
		}

		for (y = 5; y <= 110; y++)
		{
			for (x = 50; x <= 90; x++)
			{
				if (binary_black[x + WIDTH * y] == 0)
					fpga_videodata[x + WIDTH * y] = 0xf800;
				
				else
					fpga_videodata[x + WIDTH * y] = 0xffff;
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		printf("y_f : %d   y_l : %d\n", y_f, y_l);
		differ_1 = y_l - y_f;
		
		if ((y_f >= 51) && (y_f <= 53))
		{
			printf("center!!\n");
			select_motion(1);
			Delay(200000);
			select_motion(38);
			Delay(200000);
			select_motion(19);
			break;
		}
		else if (y_f > 60)
		{
			select_motion(21);
			Delay(200000);
			select_motion(21);
			Delay(200000);
			select_motion(21);
			Delay(200000);
			select_motion(38);
			Delay(200000);
			select_motion(38);
			Delay(200000);
		}
		else if ((y_f > 56) && (y_f <= 60))
		{
			select_motion(21);
			Delay(200000);
			select_motion(38);
			Delay(200000);
			select_motion(38);
			Delay(200000);
		}
		else if ((y_f > 53) && (y_f <= 56))
		{
			select_motion(27);
			Delay(200000);
			select_motion(27);
			Delay(200000);
			select_motion(38);
			Delay(200000);
			select_motion(38);
			Delay(200000);
		}
		else if (y_f < 42)
		{
			select_motion(22);
			Delay(200000);
			select_motion(22);
			Delay(200000);
			select_motion(22);
			Delay(200000);
			select_motion(38);
			Delay(200000);	
			select_motion(38);
			Delay(200000);
		}
		else if ((y_f >= 48) && (y_f < 51))
		{
			select_motion(32);
			Delay(200000);
			select_motion(32);
			Delay(200000);
			select_motion(38);
			Delay(200000);
			select_motion(38);
			Delay(200000);
		}
		else if ((y_f >= 42) && (y_f < 48))
		{
			select_motion(22);
			Delay(200000);
			select_motion(38);
			Delay(200000);
			select_motion(38);
			Delay(200000);
		}
	}
}

void barricade_1(void)
{
	int x = 0, y = 0;
	int yellow_dot = 0;
	int a = 0;

	while (1)
	{		
		yellow_dot = 0;		
		clear_screen();
		select_motion(35);
		//Delay(300000);
		fpga_videodata_read(30, 150, 20, 100);		
				
		for (y = 100; y >= 20 ; y--)
		{
			for (x = 30; x <= 150; x++)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0))
				{					
					yellow_dot++;
					fpga_videodata[x + y*WIDTH] = 0xffe0;			
				}
				else
					fpga_videodata[x + y*WIDTH] = 0xffff;
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();
				
		printf("yellow_dot : %d\n", yellow_dot);
		
		if ((yellow_dot >= 500))//500
			a = 1;
		
		else if ((yellow_dot < 30))
		{
			if (a == 1)
			{	
				select_motion(25);
				Delay(250000);
				select_motion(25);
				Delay(250000);
				//printf("break\n");
				break;
			}			
		}	
	}
}

void crevasse(void)
{
	int x = 0, y = 0;
	int y_f = 0, y_l = 0;
	int differ_1 = 0;
	
	while (1)
	{
		clear_screen();
		Delay(100000);
		select_motion(34);
		Delay(100000);

		fpga_videodata_read(50, 90, 5, HEIGHT - 10);
		y_f = 0;
		y_l = 0;		

		for (y = HEIGHT - 10; y >= 5; y--)
		{
			if ((binary_black[51 + WIDTH * y] == 0) && (y_f == 0))
			{
				y_f = y;
			}
			else if ((binary_black[89 + WIDTH * y] == 0) && (y_l == 0))
			{
				y_l = y;
			}
		}

		for (y = 5; y <= 110; y++)
		{
			for (x = 50; x <= 90; x++)
			{
				if (binary_black[x + WIDTH * y] == 0)
				{
					fpga_videodata[x + WIDTH * y] = 0xf800;
				}
				else
					fpga_videodata[x + WIDTH * y] = 0xffff;
			}
		}
		
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		printf("y_f : %d   y_l : %d\n", y_f, y_l);
		differ_1 = y_l - y_f;
						
	    if ((differ_1 > 4) && (differ_1 <= 10))
		{
			select_motion(14);
			Delay(200000);
		}
		else if (differ_1 > 10)
		{
			select_motion(14);
			Delay(200000);
			select_motion(14);
			Delay(200000);
		}
		else if ((differ_1 < 0) && (differ_1 >= -5))
		{
			select_motion(8);
			Delay(200000);
		}
		else if (differ_1 < -5)
		{
			select_motion(8);
			Delay(200000);
			select_motion(8);
			Delay(200000);
		}
	}
	select_motion(10);
	Delay(300000);
}

void barricade_2(void)
{
	int x = 0, y = 0;
	int yellow_dot = 0;
	int a = 0;

	while (1)
	{
		yellow_dot = 0;
		clear_screen();
		select_motion(35);
		Delay(300000);
		fpga_videodata_read(10, 170, 30, 110);

		for (y = 110; y >= 30; y--)
		{
			for (x = 10; x <= 170; x++)
			{
				if ((r[x + y*WIDTH] == 31) && (g[x + y*WIDTH] == 31) && (b[x + y*WIDTH] == 0))
				{
					yellow_dot++;
					fpga_videodata[x + y*WIDTH] = 0xffe0;
				}
				else
					fpga_videodata[x + y*WIDTH] = 0xffff;
			}
		}
		draw_fpga_video_data_full(fpga_videodata);
		flip();

		printf("yellow_dot : %d\n", yellow_dot);

		if ((yellow_dot >= 500))//500
			a = 1;

		else if ((yellow_dot < 30))
		{
			if (a == 1)
			{
				Delay(400000);
				select_motion(25);
				Delay(250000);
				select_motion(25);
				Delay(250000);
				printf("break\n");
				break;
			}
		}
	}
}


int main(void)
{
	//int i, j, x;
	int ret;	
	
	//SURFACE* bmpsurf = 0;
	//int a=0, b=0, c=0;

	if (open_graphic() < 0)
	{
		return -1;
	}		
		
	direct_camera_display_off();		
	
	init_console();
	ret = uart_open();
	if (ret < 0) return EXIT_FAILURE;

	uart_config(UART1, 4800, 8, UART_PARNONE, 1);	
		
	//ex(); ///////////////////   연습용 함수	
	
	
	///////////////////////////////////////////      <<<<< start!!!!!! >>>>>     ////////////////////////////////////////
	/*
	barricade_1();
	center_line();
	Delay(250000);
	select_motion(3); ////////////////////////// 빨강다리 전까지 모션  (( 대회때 바리게이트부터 빨강 다리까지 거리 파악하기!!!!))
	Delay(250000);
	select_motion(38);

	select_motion(40);
	Delay(200000);
	select_motion(2);
	Delay(200000);
	select_motion(38);     ///////////////////////////// 빨강다리 모션
	Delay(200000);
	select_motion(17);
	Delay(300000);

	select_motion(25);
	Delay(250000);		/////////////////////////////////////// 빨강다리 내려옴
	center_line();

	mine();	        ///////////////////////////////////////////// 지뢰 & 허들넘기  (대회때 지뢰위치 파악하기!!!!!!! center_line_mine 함수 사용 유무 판단 하기)
	*/
	
	select_motion(18);
	Delay(250000);
	select_motion(6);
	Delay(250000);
	select_motion(6);        //////////////////////////////// 허들 넘고 턴
	Delay(250000);	
	select_motion(6);
	Delay(250000);
	select_motion(10);
	Delay(250000);
	select_motion(10);
	Delay(250000);
	select_motion(10);
	Delay(250000);
	select_motion(10);
	Delay(250000);
	center_line();	
	
		
	select_motion(25);
	Delay(250000);
	select_motion(25);
	Delay(250000);
	select_motion(25);
	Delay(250000);
	center_line();
	select_motion(25);
	Delay(250000);
	center_line();     ///////////////////////// 초록다리까지 접근 후 중앙잡기 
	Delay(250000);	
	green_bridge_ready();	
	
	green_bridge_run();    ///////////////////////// 초록다리 걷기 & 내려오기
	green_bridge_end();
	
	
	//////////////////////////    <<<<대회때 수정해야 할 곳>>>> /////////////////////////
	
	select_motion(25);
	Delay(250000);
	center_line();
	select_motion(25);      /////////////////////////// 초록다리 내려오고나서 골프공까지 접근인데 대회때 걸음수 파악해서 다시 짜기 
	Delay(250000);
	select_motion(25);
	Delay(250000);
	select_motion(25);       
	Delay(250000);
	select_motion(25);
	Delay(250000);
	center_line();

	//////////////////////////    <<<<대회때 수정해야 할 곳>>>> /////////////////////////
	kick_the_ball();        
	select_motion(25);       ////////////////// 골프공차기 & 노랑다리까지 가기전인데 이것도 대회때 거리 파악 및 골프공 위치 파악
	Delay(300000);
	center_line();

	/*
	yellow_bridge_ready_center();    /////////////////// 노랑다리 까지 접근 & 중앙잡고 올라가기
			
	Delay(250000);
	select_motion(2);
	center_line_yellow();
	Delay(250000);
	select_motion(3);    ///////////////////////////// 노랑다리 올라가서 덤블링까지
	Delay(250000);
	select_motion(2);
	Delay(250000);
	select_motion(38);
	Delay(250000);
	select_motion(38);
	Delay(250000);
	select_motion(33);     
    Delay(400000);

	center_line();
	Delay(250000);
	select_motion(4);
	Delay(250000);	
	select_motion(4);
	Delay(250000);
	select_motion(3);   ///////////////////////////////    크레바스 까지 접근 & 넘기
	Delay(250000);
	select_motion(11);
	Delay(250000);
	*/
	/*
	center_line();
	select_motion(25);
	Delay(250000);
	select_motion(25);   /////////////////// THE END
	Delay(250000);
    barricade_2();
	*/
	direct_camera_display_on();
	
	/*
	if (bmpsurf != 0)
	{
		release_surface(bmpsurf);
	}
	*/
	uart_close();
	close_graphic();

	return 0;
}
