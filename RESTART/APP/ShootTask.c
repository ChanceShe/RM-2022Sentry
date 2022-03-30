#include "main.h"

uint8_t  friction_rotor = 0;
int16_t FRICTION_SPEED_REF = 0;

/* shot task global parameter */
shoot_t  shot;
uint32_t shoot_time_last;
uint32_t start_shooting_count = 0;//��ת��ʱ
uint32_t start_lock_rotor_count = 0;//��ת��ʱ
uint32_t start_reversal_count = 0;//��ת��ʱ
uint8_t lock_rotor = 0;//��ת��־λ
int16_t pwm_ccr = 0;
uint8_t overheat = 0;//�����ʱ�־λ
//�����������������


void shoot_friction_handle ( void ) //Ħ���� ����   friction_rotor��־  ���������
{

    if ( friction_rotor == 1 )			//Ħ����ת��
    {

        pid_rotate[0].set = - ( FRICTION_SPEED_REF ) * frictionRamp.Calc ( &frictionRamp );
        pid_rotate[1].set = ( FRICTION_SPEED_REF ) * frictionRamp.Calc ( &frictionRamp );


    }
    else if ( friction_rotor == 2 )	//Ħ����ͣת
    {
        pid_rotate[0].set = ( FRICTION_SPEED - ( FRICTION_SPEED_REF ) * frictionRamp.Calc ( &frictionRamp ) );
        pid_rotate[1].set = - pid_rotate[0].set;
    }
    else
    {
        friction_rotor = 0;
        pid_rotate[0].set = 0;
        pid_rotate[1].set = 0;
    }

    pid_rotate[0].get = Friction1Encoder.filter_rate;
    pid_rotate[1].get = Friction2Encoder.filter_rate;
    pid_calc ( & pid_rotate[0], pid_rotate[0].get, pid_rotate[0].set );
    pid_calc ( & pid_rotate[1], pid_rotate[1].get, pid_rotate[1].set );
    CAN1_Send_Msg ( CAN1, 0, 0, pid_rotate[0].out, pid_rotate[1].out );

}
static void shoot_bullet_handle ( void ) //   ���� (GetShootState() == ????)  ���趨����
{
    if ( ( GetShootState() == SHOOTING ) && ( lock_rotor == 0 ) && shot.ctrl_mode != 0 )
    {
        start_reversal_count = 0;//���㷴ת��ʱ
        start_shooting_count++;
				if ( shot.limit_heart0 < 50 )
				{
						pid_trigger_speed.set = 0;//PID_SHOOT_MOTOR_SPEED * ( float ) ( shot.limit_heart0 / shot.max_heart0 );
				}
				else
				{
						pid_trigger_speed.set = PID_SHOOT_MOTOR_SPEED;
				}

        if ( start_shooting_count > 100 && abs ( PokeEncoder.filter_rate ) < 10  ) //��ʼ��һ��ʱ�䲢��ת�ٵ���һ����ֵ  ˵����ת
        {
            lock_rotor = 1;
            start_shooting_count = 0;
        }
    }
    else
    {
        if ( ( GetShootState() == NOSHOOTING ) || shot.ctrl_mode == 0 )
        {
            start_shooting_count = 0;//������ת��ʱ
            start_reversal_count = 0;//���㷴ת��ʱ
            pid_trigger_speed.set = 0;
        }
        else  if ( lock_rotor == 1 ) //��ת
        {
            start_shooting_count = 0;//������ת��ʱ
            pid_trigger_speed.set = -PID_SHOOT_MOTOR_SPEED ;
            start_reversal_count++;
            if ( start_reversal_count > 10 ) //��תһ��ʱ��
            {
                lock_rotor = 0;

            }
        }
    }


    if ( ( GetShootState() == NOSHOOTING ) )
    {
        
        CAN2_Send_Msg ( CAN2, 0, 0, 0, 0 );
    }
    else
    {
        
        pid_trigger_speed.get = PokeEncoder.filter_rate;
        pid_calc ( &pid_trigger_speed, pid_trigger_speed.get, pid_trigger_speed.set );
        CAN2_Send_Msg ( CAN2, pid_trigger_speed.out, 0, 0, 0 );

    }

}
void heat0_limit ( void )
{
    //ǹ����������320��ǹ��ÿ����ȴ100
    shot.max_heart0 = 320;
    shot.cooling_ratio = 50;
    shot.limit_heart0 = shot.max_heart0 - refromData.shoot_heart0 + 0.005 * shot.cooling_ratio;
    shot.total_speed = 0;

    if ( shot.limit_heart0 < 30 )
    {
				CAN2_Send_Msg ( CAN2, 0, 0, 0, 0 );
        shot.ctrl_mode = 1;
        SetShootState ( NOSHOOTING );
    }
}

void shot_task ( void )
{
    shoot_friction_handle();
    shoot_bullet_handle();
}


void shot_param_init(void)
{
  PID_struct_init(&pid_trigger_speed, POSITION_PID, 10000, 10000,35, 0.7f, 4);		//����
  PID_struct_init(&pid_rotate[0], POSITION_PID,15500,11500,50,0.0,10);						//Ħ����
  PID_struct_init(&pid_rotate[1], POSITION_PID,15500,11500,50,0.0,100);

  FRICTION_SPEED_REF = FRICTION_SPEED;
  friction_rotor = 0;
  shot.ctrl_mode=REMOTE_CTRL_SHOT;
  shot.limit_heart0=120;
}

