#include "main.h"

uint8_t  friction_rotor = 0;
int16_t FRICTION_SPEED_REF = 0;

/* shot task global parameter */
shoot_t  shot;
uint32_t shoot_time_last;
uint32_t start_shooting_count = 0;//正转计时
uint32_t start_lock_rotor_count = 0;//堵转计时
uint32_t start_reversal_count = 0;//反转计时
uint8_t lock_rotor = 0;//堵转标志位
int16_t pwm_ccr = 0;
uint8_t overheat = 0;//超功率标志位
//发射机构射击电机任务


void shoot_friction_handle ( void ) //摩擦轮 根据   friction_rotor标志  来设置输出
{

    if ( friction_rotor == 1 )			//摩擦轮转动
    {

        pid_rotate[0].set = - ( FRICTION_SPEED_REF ) * frictionRamp.Calc ( &frictionRamp );
        pid_rotate[1].set = ( FRICTION_SPEED_REF ) * frictionRamp.Calc ( &frictionRamp );


    }
    else if ( friction_rotor == 2 )	//摩擦轮停转
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
static void shoot_bullet_handle ( void ) //   根据 (GetShootState() == ????)  来设定给定
{
    if ( ( GetShootState() == SHOOTING ) && ( lock_rotor == 0 ) && shot.ctrl_mode != 0 )
    {
        start_reversal_count = 0;//清零反转计时
        start_shooting_count++;
				if ( shot.limit_heart0 < 50 )
				{
						pid_trigger_speed.set = 0;//PID_SHOOT_MOTOR_SPEED * ( float ) ( shot.limit_heart0 / shot.max_heart0 );
				}
				else
				{
						pid_trigger_speed.set = PID_SHOOT_MOTOR_SPEED;
				}

        if ( start_shooting_count > 100 && abs ( PokeEncoder.filter_rate ) < 10  ) //开始了一段时间并且转速低于一定的值  说明堵转
        {
            lock_rotor = 1;
            start_shooting_count = 0;
        }
    }
    else
    {
        if ( ( GetShootState() == NOSHOOTING ) || shot.ctrl_mode == 0 )
        {
            start_shooting_count = 0;//清零正转计时
            start_reversal_count = 0;//清零反转计时
            pid_trigger_speed.set = 0;
        }
        else  if ( lock_rotor == 1 ) //堵转
        {
            start_shooting_count = 0;//清零正转计时
            pid_trigger_speed.set = -PID_SHOOT_MOTOR_SPEED ;
            start_reversal_count++;
            if ( start_reversal_count > 10 ) //反转一段时间
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
    //枪管热量上限320，枪口每秒冷却100
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
  PID_struct_init(&pid_trigger_speed, POSITION_PID, 10000, 10000,35, 0.7f, 4);		//拨盘
  PID_struct_init(&pid_rotate[0], POSITION_PID,15500,11500,50,0.0,10);						//摩擦轮
  PID_struct_init(&pid_rotate[1], POSITION_PID,15500,11500,50,0.0,100);

  FRICTION_SPEED_REF = FRICTION_SPEED;
  friction_rotor = 0;
  shot.ctrl_mode=REMOTE_CTRL_SHOT;
  shot.limit_heart0=120;
}

