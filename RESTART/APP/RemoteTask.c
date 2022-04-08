#include <stm32f4xx.h>
#include "main.h"

ChassisSpeed_Ref_t ChassisSpeedRef;
Gimbal_Ref_t GimbalRef;
FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF;


 InputMode_e inputmode = STOP;															//输入模式设定
static volatile Shoot_State_e shootState = NOSHOOTING;						//射击模式

//遥控器数据初始化，斜坡函数等的初始化
RampGen_t frictionRamp = RAMP_GEN_DAFAULT;  //摩擦轮斜坡
void RemoteTaskInit()
{
	frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_TICK_COUNT );
  frictionRamp.ResetCounter(&frictionRamp);
  //底盘云台给定值初始化
  GimbalRef.pitch_angle_dynamic_ref = 0.0f;
  GimbalRef.yaw_angle_dynamic_ref = 0.0f;
  ChassisSpeedRef.forward_back_ref = 0.0f;
  ChassisSpeedRef.left_right_ref = 0.0f;
  ChassisSpeedRef.rotate_ref = 0.0f;

}

//遥控器数据处理		usart1中断执行
void RemoteDataPrcess(refrom_mainboard_t *rc)
{
  SetInputMode ( &refromData );
	 switch(GetInputMode( ))
    {
    case REMOTE_INPUT:
    {
      //遥控器控制模式
      RemoteControlProcess(&refromData);
    }
    break;
    case KEY_MOUSE_INPUT:
    {
      //巡逻模式
    }
    break;
    case STOP:
    {
      //紧急停车
    }
    break;
    }
}


uint8_t IsRemoteBeingAction ( void )
{

    return ( fabs ( ChassisSpeedRef.forward_back_ref ) >= 10 || fabs ( ChassisSpeedRef.left_right_ref ) >= 10 \
					|| fabs ( GimbalRef.yaw_speed_ref ) >= 10 || fabs ( GimbalRef.pitch_speed_ref ) >= 10 );

}

InputMode_e GetInputMode( void )									//获取控制模式
{
  return inputmode;
}
Shoot_State_e GetShootState()     					//获得拨盘状态     和摩擦轮状态设置
{
    return shootState;
}
void SetShootState ( Shoot_State_e state ) //设置拨盘转态     在
{
    shootState = state;
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
void SetInputMode(refrom_mainboard_t *rc)
{
  if(rc->s2 == 3)
    {
      inputmode = REMOTE_INPUT;				//遥控器模式

    }
  else if(rc->s2 == 1)
    {
      inputmode = KEY_MOUSE_INPUT;		//巡逻模式

    }
  else if(rc->s2 == 2)
    {
      inputmode = STOP;								//停车
    }

}

//遥控器控制模式处理
RemoteSwitch_t switch1;   //遥控器左侧拨杆
void RemoteControlProcess(refrom_mainboard_t *rc)
{    
	if ( gim.ctrl_mode == GIMBAL_REMOTE_MODE )
  {
      GimbalRef.pitch_angle_dynamic_ref -= ( rc->ch3 - ( int16_t ) REMOTE_CONTROLLER_STICK_OFFSET ) * STICK_TO_PITCH_ANGLE_INC_FACT;
      GimbalRef.yaw_angle_dynamic_ref   -= ( rc->ch2 - ( int16_t ) REMOTE_CONTROLLER_STICK_OFFSET ) * STICK_TO_YAW_ANGLE_INC_FACT  ;
  }
	RemoteShootControl ( &switch1, rc->s1 ); //s1   遥控器控制发射

}

void RemoteShootControl ( RemoteSwitch_t *sw, uint8_t val ) //遥控器控制发射
{
    GetRemoteSwitchAction ( sw, val ); //   遥控器的复杂s1  ，模式切换的索引

    switch ( friction_wheel_state )
    {
        case FRICTION_WHEEL_OFF:								//摩擦轮关闭
        {
            frictionRamp.ResetCounter ( &frictionRamp );
            if ( sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1  ) //从关闭到start turning
            {
                SetShootState ( NOSHOOTING );  //拨盘选择是否开启
                friction_rotor = 0;
                frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_TICK_COUNT );
                frictionRamp.ResetCounter ( &frictionRamp );
                friction_wheel_state = FRICTION_WHEEL_START_TURNNING;
            }
        }
        break;
        case FRICTION_WHEEL_START_TURNNING:			//摩擦轮斜坡启动
        {
            if ( sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1 ) //刚启动就被关闭
            {
                SetShootState ( NOSHOOTING );  //拨盘选择是否开启
                friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
                frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT );
                frictionRamp.ResetCounter ( &frictionRamp );
                friction_rotor = 2;
            }
            else
            {
                friction_rotor = 1;
                if ( frictionRamp.IsOverflow ( &frictionRamp ) )
                {
                    friction_wheel_state = FRICTION_WHEEL_ON;
                }

            }
        }
        break;
        case FRICTION_WHEEL_ON:								//摩擦轮运行
        {
						LASER_ON();
            if ( sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1 ) //关闭摩擦轮
            {
                friction_rotor = 2;
                friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
                frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT );
                frictionRamp.ResetCounter ( &frictionRamp );
                SetShootState ( NOSHOOTING );  //拨盘选择是否开启
								LASER_OFF();
            }
            else if ( sw->switch_value_raw == 2 )
            {
                SetShootState ( SHOOTING );   		//拨盘开启
            }
            else
            {
                SetShootState ( NOSHOOTING );    //拨盘关闭
            }
        }
        break;

        case FRICTION_WHEEL_STOP_TURNNING:			//摩擦轮停转
        {
            friction_rotor = 2;
            if ( frictionRamp.IsOverflow ( &frictionRamp ) )
            {
                friction_rotor = 0;
                friction_wheel_state = FRICTION_WHEEL_OFF;
            }
        }
        break;
    }
}
