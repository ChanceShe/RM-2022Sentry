#include <stm32f4xx.h>
#include "main.h"

ChassisSpeed_Ref_t ChassisSpeedRef;
Gimbal_Ref_t GimbalRef;
FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF;


 InputMode_e inputmode = STOP;															//����ģʽ�趨
static volatile Shoot_State_e shootState = NOSHOOTING;						//���ģʽ

//ң�������ݳ�ʼ����б�º����ȵĳ�ʼ��
RampGen_t frictionRamp = RAMP_GEN_DAFAULT;  //Ħ����б��
void RemoteTaskInit()
{
	frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_TICK_COUNT );
  frictionRamp.ResetCounter(&frictionRamp);
  //������̨����ֵ��ʼ��
  GimbalRef.pitch_angle_dynamic_ref = 0.0f;
  GimbalRef.yaw_angle_dynamic_ref = 0.0f;
  ChassisSpeedRef.forward_back_ref = 0.0f;
  ChassisSpeedRef.left_right_ref = 0.0f;
  ChassisSpeedRef.rotate_ref = 0.0f;

}

//ң�������ݴ���		usart1�ж�ִ��
void RemoteDataPrcess(refrom_mainboard_t *rc)
{
  SetInputMode ( &refromData );
	 switch(GetInputMode( ))
    {
    case REMOTE_INPUT:
    {
      //ң��������ģʽ
      RemoteControlProcess(&refromData);
    }
    break;
    case KEY_MOUSE_INPUT:
    {
      //Ѳ��ģʽ
    }
    break;
    case STOP:
    {
      //����ͣ��
    }
    break;
    }
}


uint8_t IsRemoteBeingAction ( void )
{

    return ( fabs ( ChassisSpeedRef.forward_back_ref ) >= 10 || fabs ( ChassisSpeedRef.left_right_ref ) >= 10 \
					|| fabs ( GimbalRef.yaw_speed_ref ) >= 10 || fabs ( GimbalRef.pitch_speed_ref ) >= 10 );

}

InputMode_e GetInputMode( void )									//��ȡ����ģʽ
{
  return inputmode;
}
Shoot_State_e GetShootState()     					//��ò���״̬     ��Ħ����״̬����
{
    return shootState;
}
void SetShootState ( Shoot_State_e state ) //���ò���ת̬     ��
{
    shootState = state;
}



//ң������ֵ
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
  static uint32_t switch_cnt = 0;
  /* ����״ֵ̬ */
  sw->switch_value_raw = val;
  sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;
  /* ȡ����ֵ����һ��ֵ */
  sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
                      (sw->switch_value_buf[sw->buf_index]);

  /* ���ϵ�״ֵ̬������ */
  sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	
  /* �ϲ�����ֵ */
  sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;
  /* �����ж� */
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
//����ģʽ����
void SetInputMode(refrom_mainboard_t *rc)
{
  if(rc->s2 == 3)
    {
      inputmode = REMOTE_INPUT;				//ң����ģʽ

    }
  else if(rc->s2 == 1)
    {
      inputmode = KEY_MOUSE_INPUT;		//Ѳ��ģʽ

    }
  else if(rc->s2 == 2)
    {
      inputmode = STOP;								//ͣ��
    }

}

//ң��������ģʽ����
RemoteSwitch_t switch1;   //ң������ದ��
void RemoteControlProcess(refrom_mainboard_t *rc)
{    
	if ( gim.ctrl_mode == GIMBAL_REMOTE_MODE )
  {
      GimbalRef.pitch_angle_dynamic_ref -= ( rc->ch3 - ( int16_t ) REMOTE_CONTROLLER_STICK_OFFSET ) * STICK_TO_PITCH_ANGLE_INC_FACT;
      GimbalRef.yaw_angle_dynamic_ref   -= ( rc->ch2 - ( int16_t ) REMOTE_CONTROLLER_STICK_OFFSET ) * STICK_TO_YAW_ANGLE_INC_FACT  ;
  }
	RemoteShootControl ( &switch1, rc->s1 ); //s1   ң�������Ʒ���

}

void RemoteShootControl ( RemoteSwitch_t *sw, uint8_t val ) //ң�������Ʒ���
{
    GetRemoteSwitchAction ( sw, val ); //   ң�����ĸ���s1  ��ģʽ�л�������

    switch ( friction_wheel_state )
    {
        case FRICTION_WHEEL_OFF:								//Ħ���ֹر�
        {
            frictionRamp.ResetCounter ( &frictionRamp );
            if ( sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1  ) //�ӹرյ�start turning
            {
                SetShootState ( NOSHOOTING );  //����ѡ���Ƿ���
                friction_rotor = 0;
                frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_TICK_COUNT );
                frictionRamp.ResetCounter ( &frictionRamp );
                friction_wheel_state = FRICTION_WHEEL_START_TURNNING;
            }
        }
        break;
        case FRICTION_WHEEL_START_TURNNING:			//Ħ����б������
        {
            if ( sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1 ) //�������ͱ��ر�
            {
                SetShootState ( NOSHOOTING );  //����ѡ���Ƿ���
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
        case FRICTION_WHEEL_ON:								//Ħ��������
        {
						LASER_ON();
            if ( sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1 ) //�ر�Ħ����
            {
                friction_rotor = 2;
                friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
                frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT );
                frictionRamp.ResetCounter ( &frictionRamp );
                SetShootState ( NOSHOOTING );  //����ѡ���Ƿ���
								LASER_OFF();
            }
            else if ( sw->switch_value_raw == 2 )
            {
                SetShootState ( SHOOTING );   		//���̿���
            }
            else
            {
                SetShootState ( NOSHOOTING );    //���̹ر�
            }
        }
        break;

        case FRICTION_WHEEL_STOP_TURNNING:			//Ħ����ͣת
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
