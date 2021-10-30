#include <stm32f4xx.h>
#include "main.h"

RC_Ctl_t RC_CtrlData;
ChassisSpeed_Ref_t ChassisSpeedRef;
Gimbal_Ref_t GimbalRef;

static InputMode_e inputmode = STOP;//REMOTE_INPUT;   //输入模式设定
int rotate_num_ture=0;		//掉头标志

//遥控器数据初始化，斜坡函数等的初始化
RampGen_t frictionRamp = RAMP_GEN_DAFAULT;  //摩擦轮斜坡
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   //mouse左右移动斜坡
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;   //mouse前后移动斜坡
void RemoteTaskInit()
{
  frictionRamp.ResetCounter(&frictionRamp);
  LRSpeedRamp.ResetCounter(&LRSpeedRamp);
  FBSpeedRamp.ResetCounter(&FBSpeedRamp);
  //底盘云台给定值初始化
  GimbalRef.pitch_angle_dynamic_ref = 0.0f;
  GimbalRef.yaw_angle_dynamic_ref = 0.0f;
  ChassisSpeedRef.forward_back_ref = 0.0f;
  ChassisSpeedRef.left_right_ref = 0.0f;
  ChassisSpeedRef.rotate_ref = 0.0f;

}


InputMode_e GetInputMode()
{
  return inputmode;
}

//遥控器读值
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
  static uint32_t switch_cnt = 0;
  /* 最新状态值 */
  sw->switch_value_raw = val;
  sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;
  /* 取最新值和上一次值 */
  sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
                      (sw->switch_value_buf[sw->buf_index]);

  /* 最老的状态值的索引 */
  sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

  /* 合并三个值 */
  sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;
  /* 长按判断 */
  if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
    {
      switch_cnt++;
    }
  else
    {
      switch_cnt = 0;
    }

  if(switch_cnt >= 40)
    {
      sw->switch_long_value = sw->switch_value_buf[sw->buf_index];
    }

  sw->buf_last_index = sw->buf_index;
  sw->buf_index++;
  if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
    {
      sw->buf_index = 0;
    }
}
//输入模式设置
void SetInputMode(Remote *rc)
{
  if(rc->s2 == 1)
    {
      inputmode = REMOTE_INPUT;

    }
  else if(rc->s2 == 3)
    {
      inputmode = KEY_MOUSE_INPUT;

    }
  else if(rc->s2 == 2)
    {
      inputmode = STOP;
      rotate_num_ture=0;
    }

}
//遥控器控制模式处理
RemoteSwitch_t switch1;   //遥控器左侧拨杆
void RemoteControlProcess(Remote *rc)
{
			ChassisSpeedRef.forward_back_ref = (rc->ch1- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
      ChassisSpeedRef.left_right_ref   = (rc->ch0- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
//      chassis.vw = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
      chassis.vw = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;
	
//  /*******************遥控器拨杆数据处理*******************/
//#if REMOTE_SHOOT == 1
//  RemoteShootControl(&switch1, rc->s1);			//此处两个函数二选一，左拨杆功能分别是发射
//#elif REMOTE_SHOOT == 0
//  Remote_Rotate_Reverse_Control(&switch1, rc->s1);        //小陀螺
//#endif
//  /***********************************************************/

}

//遥控器数据处理
void RemoteDataPrcess(uint8_t *pData)
{
  if(pData == NULL)
    {
      return;
    }

  RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
  RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
  RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                        ((int16_t)pData[4] << 10)) & 0x07FF;
  RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
  RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
  RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);//模式切换
  RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
  RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
  RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
  RC_CtrlData.mouse.press_l = pData[12];
  RC_CtrlData.mouse.press_r = pData[13];
  RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
  SetInputMode(&RC_CtrlData.rc);


  switch(GetInputMode( ))
    {
    case REMOTE_INPUT:
    {
      //遥控器控制模式
      RemoteControlProcess(&(RC_CtrlData.rc));
//      SetWorkState(NORMAL_STATE);
    }
    break;
    case KEY_MOUSE_INPUT:
    {
      //键盘控制模式
//      MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);
//      SetWorkState(NORMAL_STATE);
    }
    break;
    case STOP:
    {
//      SetWorkState(PREPARE_STATE);
      //紧急停车
    }
    break;
    }
}
/*********************遥控模式射击***********************/
/*
flag： friction_rotor    0：摩擦轮停止   1：摩擦轮开启   2：摩擦轮关闭（REF -> 0）
*/
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val)
{
}



int rotate_num = 0;

/*********************遥控模式小陀螺***********************/

u8 close_rotate_flag = 0;		//小陀螺停转
void Remote_Rotate_Reverse_Control(RemoteSwitch_t *sw, uint8_t val)
{
  GetRemoteSwitchAction(sw, val);
  switch (chassis.ctrl_mode)
    {
			
    case MANUAL_FOLLOW_GIMBAL:
    {

//      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)
//        {

//          chassis.ctrl_mode = CHASSIS_ROTATE;
//        }
//      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO2)
//        {
//          chassis.ctrl_mode = CHASSIS_CHANGE_REVERSE;
//          GimbalRef.yaw_angle_dynamic_ref +=180;
//        }
    }
    break;
		
    case CHASSIS_ROTATE:
    {
//      if((sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)&&(close_rotate_flag==0))
//        {
//          // 底盘云台相对角度给到下一个360的倍数
//          rotate_num = GMYawEncoder.ecd_angle/360 ;
//          close_rotate_flag = 1;
//        }
//      if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-((rotate_num+1)*360)<=35)&&(GMYawEncoder.ecd_angle-((rotate_num+1)*360)>=-35))
//        {
//          chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
//          chassis_rotate_flag ^=1;
//          chassis.position_ref = (rotate_num+1)*360;
//          close_rotate_flag = 0;
//        }
//      else if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-((rotate_num-1)*360)<=35)&&(GMYawEncoder.ecd_angle-((rotate_num-1)*360)>=-35))
//        {
//          chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
//          chassis_rotate_flag ^=1;
//          chassis.position_ref = (rotate_num-1)*360;
//          close_rotate_flag = 0;
//        }
//      else if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-(rotate_num*360)<=35)&&(GMYawEncoder.ecd_angle-(rotate_num*360)>=-35))
//        {
//          chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
//          chassis_rotate_flag ^=1;
//          chassis.position_ref = rotate_num*360;
//          close_rotate_flag = 0;
//        }

    }
    break;

    case CHASSIS_REVERSE:
    {
//      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_2TO3)
//        {
//          chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
//        }

    }
    break;
		
    default:
    break;
		
    }

}
