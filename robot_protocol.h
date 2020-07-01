/* ------------------------------------------------------------------------ */
/*                  KAIST BATTLE ROBOT CONTROL PROGRAM						*/
/*                   														*/
/*          Coding by YKH      												*/
/*          file : robot_protocol.h    										*/
/* ------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */
/*                  KAIST BATTLE ROBOT CONTROL PROGRAM            			*/
/*                   														*/
/*          Modified by Goon-Ho Choi, HANOOL ROBOTICS 2003.07.16     		*/
/* ------------------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */
#ifndef __ROBOT_PROTOCOL_H__
#define __ROBOT_PROTOCOL_H__
// �Ʒ��� ���������� ����(Head B/D)���� �κ� ���� ���� �������� ���� ��������
#define SCI_BUF_SIZE   4
// Check Sum �ڵ�(���� ����� ���� ������ġ)�� �߰��� ���� �� ����Ʈ�� �ø�
/*********************************************************************
Communication Protocol :

 Start Code(1byte) + Command(1byte) + Data(1byte) + CheckSum = total 4bytes
  0xFF    0x??   0x?? 0x??

*********************************************************************/
#define START_CODE    0xFF
/* command definitions */
#define CONT_FWD_LEFT	0x23 /* Left Motor move foward Continuously */ 
#define CONT_FWD_RIGHT  0x20 /* Right Motor move foward Continuously */ 
#define CONT_FWD_ALL	0x03 /* All Motor move foward Continuously */ 
#define CONT_BWD_LEFT   0x04 /* Left Motor move backward Continuously */ 
#define CONT_BWD_RIGHT  0x05 /* Right Motor move backward Continuously */ 
#define CONT_BWD_ALL	0x06 /* All Motor move backward Continuously */
#define CONT_STOP		0x07 /* All Motor stop */
//#define SET_STEP_LEFT   0x08 /* Left Motor Setting steps */
//#define SET_STEP_RIGHT  0x09 /* Right Motor Setting steps */
//#define SET_STEP_ALL	0x0a /* All Motor Setting steps */
//#define GET_STEP_LEFT   0x0b
//#define GET_STEP_RIGHT  0x0c
//#define GET_STEP_ALL	0x0d
//#define STEP_FWD_LEFT   0x0e
//#define STEP_FWD_RIGHT  0x0f
//#define STEP_FWD_ALL	0x10
//#define STEP_BWD_LEFT   0x11
//#define STEP_BWD_RIGHT	0x12
//#define STEP_BWD_ALL	0x13

#define LASER_SHOOT		0x14 /* Shoot the laser pointer */
#define GET_ENERGY		0x15 /* Get the remains energy */
#define GET_SHOT		0x16 /* Get the remains bullet number */

#define SET_V_LEFT		0x17
#define SET_V_RIGHT		0x18
#define SET_V_ALL		0x19
#define GET_V_LEFT		0x1a
#define GET_V_RIGHT		0x1b

#define GET_V_ALL		0x1c ////////////////////////////////////////////

//#define DAMAGE_ALERT	0xFE

// �ѿ� �߰���
#define CHECK_BOTTOM	0x20		// �ٴ� ������ ���� ���ο� ������ ��ġ ���� ����
#define CHECK_DAMAGE_POINT	0x30	// �ǰݵǾ����� ���ο� �¾��� ��� ��� ��ġ���� ���� ���� ����
#define LASER_READY	0x40	// �غ� �Ǿ������� data byte�� 1, �غ� �ȵǸ� 0���� �ö� �ɴϴ�.

/************************************************************************************/
/* �Ʒ� �κ��� ������ ����� �κа� Ʋ�� �κ���. -> ���� ��� ������ 				*/
/* ���� Command�� ������ ��� Receive�κ��� Comm_OK�� �Ƿ��� �ö��					*/
/************************************************************************************/ 
#define MOTOR_ALARM	0x41	// �տ� ��ֹ��� �־ ���Ͱ� �����ϰ� �ɷ��� �����Ҽ� ���� ��� ��.
#define GAME_OVER	0x42	// ������ ������ ��� ��. �̶��� ��� ����� ������ ���� ������ �̰��� �ö��.
#define DEAD 0x44
// ���� �� �¾Ƽ� �׾������� �������� �ʽ��ϴ�. �̶��� ��� ����� ������ 
// ���� ������ �� ������ ���� �ǿ��� ���� �Ͻñ� �ٶ��ϴ�.

// CheckSum ���
/*	CheckSum = ((unsigned char)Command & 0x0f) + (((unsigned char)Command>>4)&0x0f) 
				+ ((unsigned char)Data & 0x0f) + (((unsigned char)Data>>4)&0x0f);
*/


//
//
// �κ� ���� ���忡�� ����(HEAD) ������� ���� ��������
//
//
/*********************************************************************
Communication Protocol :

 Start Code(1byte) + Comm_OK(1byte) + Data(1byte) + CheckSum = total 4bytes
  0xFF    0x??   0x?? 0x??

*********************************************************************/
// �����ʹ� �ϴ� 1byte�� ������ �ۼ����� ũ�⸦ ��ġ���׽��ϴ�.
//#define CHECK_BOTTOM�� ���� ���� ������
// ���� �� ���� :		Data=0x01;
// ���� �Ʒ� ���� :		Data=0x04;
// ������ �� ���� :		Data=0x11;
// ������ �Ʒ� ���� :	Data=0x14;
/******************Bottom*****************************/
// ���� ���� Ʋ�� ���� ->�Ʒ� ���� �����ϼ���
// ��ũ�� ������ ������ ��� �Դϴ�.
// ��Ʈ 0->���� ��
// ��Ʈ 1->������ ��
// ��Ʈ 2->���� ��
// ��� 3->������ ��

//#define CHECK_DAMAGE_POINT�� ���� ���� ������
// ���� �ǰ� :		Data=0x01; 
// �ĸ� �ǰ� :		Data=0x04;
// ������ �ǰ� :	Data=0x11;
// ������ �ǰ� :	Data=0x14;

/********************* Damage Point ******************/
// ��Ʈ 0-> ����
// ��Ʈ 1-> ������
// ��Ʈ 2-> �ĸ�
// ��Ʈ 3-> ����

//#define COMM_ERROR
// ���� ���� ��ɿ� ������ ���� ��� �������� �䱸��.
//		Comm_OK=0xFF;

//#define COMM_OK
// ���� ���� ��ɿ�  ������ ���� ���
// Comm_OK=0x00;
// ���� ���� ����� ���� �����͸� �䱸�ϴ� ��� : Data�� �ش� ������ ����
// ���� ���� ����� ���� �����͸� �䱸���� ���� ��� : Data=0x00 

// CheckSum ���
/*	CheckSum = ((unsigned char)Comm_OK & 0x0f) + (((unsigned char)Comm_OK>>4)&0x0f) 
				+ ((unsigned char)Data & 0x0f) + (((unsigned char)Data>>4)&0x0f);
*/

/* 
- ���������� ���� ��� ���������� ������ �����ϰ� ������ �߽��ϴ�.

- �߰��� ���������� �輱�� ����ȭ ���� ���� �Ͱ� ��⸦ ���� �� ȿ������ ������ ���� ���Դϴ�.

- CheckSum �ڵ��� �߰��� ���� ����� ���� ��ġ�Դϴ�.

  CheckSum�ڵ��� �߰��� HEAD ���� �ʿ��� ����� ���� �� �ݵ�� ���� ���� ��ȯ ��� �ϼž� �մϴ�.
�� ��� ���� ���� ������ Hang Up�� �ɸ� �� �����Ƿ� ���� �ð� ���� �������� ������ ������ �˶� ���� ��ġ�� �־�� �մϴ�.

- ���� �ʿ��� ������ �䱸�� �������ų�  CheckSum�� ��ġ ���� ������ �ݵ�� ���� ����� �ٽ� ���� �ֽʽÿ�.
*/

////////////////////////////////////////////////////////////////////////////////
void DelayLoop(int delay_time);
void Send_Command(unsigned char command);
//unsigned char Receive_Ack(void);

void motion_1(void);
void motion_2(void);
void motion_3(void);
void motion_4(void);
void motion_5(void);
void motion_6(void);
void motion_7(void);
void motion_8(void);
void motion_9(void);
void motion_10(void);
void motion_11(void);
void motion_12(void);
void motion_13(void);
void motion_14(void);
void motion_15(void);
void motion_16(void);
void motion_17(void);
void motion_18(void);
void motion_19(void);
void motion_20(void);
void motion_21(void);
void motion_22(void);
void motion_23(void);
void motion_24(void);
void motion_25(void);
void motion_26(void);
void motion_27(void);
void motion_28(void);
void motion_29(void);
void motion_30(void);
void motion_31(void);
void motion_32(void);
void motion_33(void);
void motion_34(void);
void motion_35(void);
void motion_36(void);
void motion_37(void);
void motion_38(void);
void motion_39(void);
void motion_40(void);


////////////////////////////////////////////////////////////////////////////////

#endif // __ROBOT_PROTOCOL_H__

