#include <stm32f4xx.h>
#include "main.h"

RC_Ctl_t RC_CtrlData;
ChassisSpeed_Ref_t ChassisSpeedRef;
Gimbal_Ref_t GimbalRef;
FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF;


static InputMode_e inputmode = STOP;															//����ģʽ�趨
static volatile Shoot_State_e shootState = NOSHOOTING;						//���ģʽ
static volatile Shooting_State_e Shooting_State = NORMAL_SHOOTING;//�������ģʽ

//ң�������ݳ�ʼ����б�º����ȵĳ�ʼ��
RampGen_t frictionRamp = RAMP_GEN_DAFAULT;  //Ħ����б��
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   //mouse�����ƶ�б��
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;   //mouseǰ���ƶ�б��
void RemoteTaskInit()
{
  frictionRamp.ResetCounter(&frictionRamp);
  LRSpeedRamp.ResetCounter(&LRSpeedRamp);
  FBSpeedRamp.ResetCounter(&FBSpeedRamp);
  //������̨����ֵ��ʼ��
  GimbalRef.pitch_angle_dynamic_ref = 0.0f;
  GimbalRef.yaw_angle_dynamic_ref = 0.0f;
  ChassisSpeedRef.forward_back_ref = 0.0f;
  ChassisSpeedRef.left_right_ref = 0.0f;
  ChassisSpeedRef.rotate_ref = 0.0f;

}

//ң�������ݴ���		usart1�ж�ִ��
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
  RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);//ģʽ�л�
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
      //ң��������ģʽ
      RemoteControlProcess(&(RC_CtrlData.rc));
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



InputMode_e GetInputMode()									//��ȡ����ģʽ
{
  return inputmode;
}
Shoot_State_e GetShootState()     					//��ò���״̬     ��Ħ����״̬����
{
    return shootState;
}


void SetShootState ( Shoot_State_e state )  //���ò���ת̬
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
void SetInputMode(Remote *rc)
{
  if(rc->s2 == 1)
    {
      inputmode = REMOTE_INPUT;				//ң����ģʽ

    }
  else if(rc->s2 == 3)
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
void RemoteControlProcess(Remote *rc)
{
	ChassisSpeedRef.forward_back_ref = (rc->ch1- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
  ChassisSpeedRef.left_right_ref   = (rc->ch0- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
	RemoteShootControl ( &switch1, rc->s1 ); //s1   ң�������Ʒ���

}

void RemoteShootControl ( RemoteSwitch_t *sw, uint8_t val ) //ң����  -ѡ����Ħ���ֺͼ���
{
    GetRemoteSwitchAction ( sw, val ); //   ң�����ĸ���s1  ��ģʽ�л�������

    switch ( friction_wheel_state )
    {
        case FRICTION_WHEEL_OFF:								//Ħ���ֹر�
        {
            frictionRamp.ResetCounter ( &frictionRamp );
            if ( sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1  ) //�ӹرյ�start turning
            {
                SetShootState ( NOSHOOTING ); //����ѡ���Ƿ���
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
                SetShootState ( NOSHOOTING ); //����ѡ���Ƿ���
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
            if ( sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1 ) //�ر�Ħ����
            {
                friction_rotor = 2;
                friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
                frictionRamp.SetScale ( &frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT );
                frictionRamp.ResetCounter ( &frictionRamp );
                SetShootState ( NOSHOOTING ); //����ѡ���Ƿ���
            }
            else if ( sw->switch_value_raw == 2 )
            {
                SetShootState ( SHOOTING );  		//���̿���
            }
            else
            {
                SetShootState ( NOSHOOTING );   //���̹ر�
            }
        }
        break;

        case FRICTION_WHEEL_STOP_TURNNING:
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
