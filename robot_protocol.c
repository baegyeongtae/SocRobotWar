/************************************************************************
Title     : Robot Body Protocol Source File
File name : robot_protocol.c

Author    : adc inc. (oxyang@adc.co.kr)
History
+ v0.0  2007/2/14
+ v1.0  2008/8/6
************************************************************************/
#include <stdio.h>
#include <string.h>
#include "robot_protocol.h"
#include "uart_api.h"
//////////////////////////////////////////////////// Protocol Test

void DelayLoop(int delay_time)
{
	while (delay_time)
		delay_time--;
}

void Send_Command(unsigned char command)
{
	int i;
	unsigned char Command_Buffer[1] = { 0, };

	Command_Buffer[0] = command;	// Command Byte
	for (i = 0; i < 1; i++) printf("motion ; %d\n ", Command_Buffer[i]);

	uart1_buffer_write(Command_Buffer, 1);
	
}

/*unsigned int receive_command(void)
{
int i = 0;
unsigned int state_buffer[1] = {0,};

for(i = 0 ; i < 1 ; i++)
{
uart1_buffer_read(state_buffer, 2);
}

return state_buffer;\]

}*/

#define ERROR 0
#define OK	1


void motion_1(void)
{
	Send_Command(1);

	//DelayLoop(1000000);		// 3second delay
}

void motion_2(void)
{

	Send_Command(2);

	//DelayLoop(1000000);		// 3second delay
}

void motion_3(void)
{
	Send_Command(3);

	//DelayLoop(1000000);		// 3second delay
}

void motion_4(void)
{
	Send_Command(4);

	//DelayLoop(1000000);		// 3second delay
}

void motion_5(void)
{
	Send_Command(5);

	//DelayLoop(1000000);		// 3second delay
}

void motion_6(void)
{
	Send_Command(6);

	//DelayLoop(1000000);		// 3second delay

}

void motion_7(void)
{
	Send_Command(7);

	//DelayLoop(1000000);		// 3second delay
}

void motion_8(void)
{
	Send_Command(8);

	//DelayLoop(1000000);		// 3second delay
}

void motion_9(void)
{
	Send_Command(9);

	//DelayLoop(1000000);		// 3second delay
}

void motion_10(void)
{
	Send_Command(10);

	//DelayLoop(1000000);		// 3second delay
}

void motion_11(void)
{
	Send_Command(11);

	//DelayLoop(1000000);		// 3second delay

}

void motion_12(void)
{
	Send_Command(12);

	//DelayLoop(1000000);		// 3second delay

}

void motion_13(void)
{
	Send_Command(13);

	//DelayLoop(1000000);		// 3second delay

}

void motion_14(void)
{
	Send_Command(14);

	//DelayLoop(1000000);		// 3second delay
}

void motion_15(void)
{
	Send_Command(15);

	//DelayLoop(1000000);		// 3second delay

}

void motion_16(void)
{
	Send_Command(16);

	//DelayLoop(1000000);		// 3second delay
}

void motion_17(void)
{
	Send_Command(17);

	//DelayLoop(1000000);		// 3second delay
}

void motion_18(void)
{
	Send_Command(18);

	//DelayLoop(1000000);		// 3second delay
}

void motion_19(void)
{
	Send_Command(19);

	//DelayLoop(1000000);		// 3second delay
}

void motion_20(void)
{
	Send_Command(20);

	//DelayLoop(1000000);		// 3second delay
}

void motion_21(void)
{
	Send_Command(21);

	//DelayLoop(1000000);		// 3second delay

}

void motion_22(void)
{
	Send_Command(22);

	//DelayLoop(1000000);		// 3second delay

}

void motion_23(void)
{
	Send_Command(23);

	//DelayLoop(1000000);		// 3second delay

}

void motion_24(void)
{

	Send_Command(24);

	//DelayLoop(1000000);		// 3second delay

}

void motion_25(void)
{

	Send_Command(25);

	//DelayLoop(1000000);		// 3second delay

}

void motion_26(void)
{

	Send_Command(26);

	//DelayLoop(1000000);		// 3second delay

}

void motion_27(void)
{

	Send_Command(27);

	//DelayLoop(1000000);		// 3second delay

}

void motion_28(void)
{

	Send_Command(28);

	//DelayLoop(1000000);		// 3second delay

}

void motion_29(void)
{

	Send_Command(29);

	//DelayLoop(1000000);		// 3second delay

}

void motion_30(void)
{

	Send_Command(30);

	//DelayLoop(1000000);		// 3second delay

}

void motion_31(void)
{

	Send_Command(31);

	//DelayLoop(1000000);		// 3second delay

}

void motion_32(void)
{
	Send_Command(32);

	//DelayLoop(1000000);		// 3second delay

}

void motion_33(void)
{

	Send_Command(33);

	//DelayLoop(1000000);		// 3second delay

}

void motion_34(void)
{
	Send_Command(34);

	//DelayLoop(1000000);		// 3second delay

}

void motion_35(void)
{
	Send_Command(35);

	//DelayLoop(1000000);		// 3second delay

}

void motion_36(void)
{

	Send_Command(36);

	//DelayLoop(1000000);		// 3second delay

}

void motion_37(void)
{

	Send_Command(37);

	//DelayLoop(1000000);		// 3second delay

}

void motion_38(void)
{

	Send_Command(38);

	//DelayLoop(1000000);		// 3second delay

}

void motion_39(void)
{
	Send_Command(39);

	//DelayLoop(1000000);		// 3second delay

}
void motion_40(void)
{
	Send_Command(40);

	//DelayLoop(1000000);		// 3second delay

}
