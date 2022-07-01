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
        pid_rotate[2].set = ( FRICTION_SPEED_REF ) * frictionRamp.Calc ( &frictionRamp );
        pid_rotate[3].set = - ( FRICTION_SPEED_REF ) * frictionRamp.Calc ( &frictionRamp );

    }
    else if ( friction_rotor == 2 )	//摩擦轮停转
    {
        pid_rotate[0].set = ( FRICTION_SPEED - ( FRICTION_SPEED_REF ) * frictionRamp.Calc ( &frictionRamp ) );
        pid_rotate[1].set = -pid_rotate[0].set;
			  pid_rotate[2].set = -pid_rotate[0].set;
        pid_rotate[3].set = pid_rotate[0].set;

    }
    else
    {
        friction_rotor = 0;
        pid_rotate[0].set = 0;
        pid_rotate[1].set = 0;
        pid_rotate[2].set = 0;
        pid_rotate[3].set = 0;
    }

    pid_rotate[0].get = Friction1Encoder.filter_rate;
    pid_rotate[1].get = Friction2Encoder.filter_rate;
    pid_rotate[2].get = Friction3Encoder.filter_rate;
    pid_rotate[3].get = Friction4Encoder.filter_rate;
    pid_calc ( & pid_rotate[0], pid_rotate[0].get, pid_rotate[0].set );
    pid_calc ( & pid_rotate[1], pid_rotate[1].get, pid_rotate[1].set );
    pid_calc ( & pid_rotate[2], pid_rotate[2].get, pid_rotate[2].set );
    pid_calc ( & pid_rotate[3], pid_rotate[3].get, pid_rotate[3].set );
//    CAN1_Send_Msg ( CAN1, pid_rotate[0].out, pid_rotate[1].out, pid_rotate[2].out, pid_rotate[3].out );
    CAN1_Send_Msg ( CAN1, 0, 0, pid_rotate[2].out, pid_rotate[3].out );		//左枪管发射
		
//    CAN1_Send_Msg ( CAN1, 0, 0, 0, 0 );
}
static void shoot_bullet_handle ( void ) //   根据 (GetShootState() == ????)  来设定给定
{
    if ( ( GetShootState() == SHOOTING ) && ( lock_rotor == 0 ) && shot.ctrl_mode != 0 )
    {
        start_reversal_count = 0;//清零反转计时
        start_shooting_count++;
				if ( shot.limit_heart0 < 20 )
				{
						pid_trigger[0].set = 0;//PID_SHOOT_MOTOR_SPEED * ( float ) ( shot.limit_heart0 / shot.max_heart0 );
						pid_trigger[1].set = 0;//PID_SHOOT_MOTOR_SPEED * ( float ) ( shot.limit_heart0 / shot.max_heart0 );
						SetShootState ( NOSHOOTING );
				}
				else
				{
						pid_trigger[0].set = -PID_SHOOT_MOTOR_SPEED;
						pid_trigger[1].set = -PID_SHOOT_MOTOR_SPEED;
				}

        if ( start_shooting_count > 100 && abs ( Poke1Encoder.filter_rate ) < 10  ) //开始了一段时间并且转速低于一定的值  说明堵转
        {
            lock_rotor = 1;
            start_shooting_count = 0;
        }
				if ( start_shooting_count > 100 && abs ( Poke2Encoder.filter_rate ) < 10  ) //开始了一段时间并且转速低于一定的值  说明堵转
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
            pid_trigger[0].set = 0;
            pid_trigger[0].set = 0;
        }
        else  if ( lock_rotor == 1 ) //堵转
        {
            start_shooting_count = 0;//清零正转计时
            pid_trigger[0].set = PID_SHOOT_MOTOR_SPEED ;
            pid_trigger[1].set = PID_SHOOT_MOTOR_SPEED ;
            start_reversal_count++;
            if ( start_reversal_count > 2 ) //反转一段时间
            {
                lock_rotor = 0;\
								start_reversal_count = 0;//清零反转计时

            }
        }
    }


    if ( ( GetShootState() == NOSHOOTING ) )
    {
        CAN2_Send_Msg ( CAN2, 0, 0, 0, 0 );
				CloseLimit;
    }
    else
    {
				OpenLimit;
        pid_trigger[0].get = Poke1Encoder.filter_rate;
        pid_calc ( &pid_trigger[0], pid_trigger[0].get, pid_trigger[0].set );
			  pid_trigger[1].get = Poke2Encoder.filter_rate;
        pid_calc ( &pid_trigger[1], pid_trigger[1].get, pid_trigger[1].set );
//        CAN2_Send_Msg ( CAN2, pid_trigger[0].out, pid_trigger[1].out, 0, 0 );
//        CAN2_Send_Msg ( CAN2, pid_trigger[0].out, 0, 0, 0 );	//转右拨盘
        CAN2_Send_Msg ( CAN2, 0, pid_trigger[1].out, 0, 0 );	//转左拨盘
    }

}
void heat0_limit ( void )
{
    //枪管热量上限320，枪口每秒冷却100
    shot.max_heart0 = 320;
    shot.cooling_ratio = 50;
    shot.limit_heart0 = shot.max_heart0 - refromData.shoot_heart_l + 0.005 * shot.cooling_ratio;
    shot.total_speed = 0;


    if ( shot.limit_heart0 < 20 )
    {
				CAN2_Send_Msg ( CAN2, 0, 0, 0, 0 );
        shot.ctrl_mode = REMOTE_CTRL_SHOT;
        SetShootState ( NOSHOOTING );
    }
}

void shot_task ( void )
{
		heat0_limit ();
    shoot_friction_handle();
    shoot_bullet_handle();
}


void shot_param_init(void)
{
  PID_struct_init(&pid_trigger[0], POSITION_PID, 10000, 10000,60, 0.5f, 0);		//拨盘
  PID_struct_init(&pid_trigger[1], POSITION_PID, 10000, 10000,60, 0.5f, 0);		//拨盘
  PID_struct_init(&pid_rotate[0], POSITION_PID,15500,11500,50,0.0,100);						//摩擦轮
  PID_struct_init(&pid_rotate[1], POSITION_PID,15500,11500,50,0.0,100);
  PID_struct_init(&pid_rotate[2], POSITION_PID,15500,11500,50,0.0,100);						//摩擦轮
  PID_struct_init(&pid_rotate[3], POSITION_PID,15500,11500,50,0.0,100);

  FRICTION_SPEED_REF = FRICTION_SPEED;
  friction_rotor = 0;
  shot.ctrl_mode=REMOTE_CTRL_SHOT;
  shot.limit_heart0=120;
}

void gun_limit_init(void)
{
	  GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC ,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  

	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource8, GPIO_AF_TIM3);
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource9, GPIO_AF_TIM3);
	                        
	  gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;       
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
	  gpio.GPIO_OType = GPIO_OType_PP;
	  gpio.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOC,&gpio);
	
	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource8, GPIO_AF_TIM3);  //TIM3->CCR1 ????PWM4
	  GPIO_PinAFConfig(GPIOC,GPIO_PinSource9, GPIO_AF_TIM3);  //TIM3->CCR2 ????PWM5
	
	  /* ??????TIM3 */
	  tim.TIM_Prescaler = 999;    //???
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 1679;   
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3,&tim);

	  oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 1000;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
		
    TIM_OC3Init(TIM3,&oc);
   	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);  
	  TIM_OC4Init(TIM3,&oc);
	  TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);  
		
    TIM_ARRPreloadConfig(TIM3,ENABLE);
    TIM_Cmd(TIM3,ENABLE);
		
		CloseLimit;
}
