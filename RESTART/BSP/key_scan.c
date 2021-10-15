/*
 * @file       key_scan.c
 * @brief      ����ɨ�躯��ʵ�֣�״̬����
 * @author     ���˾�
 * @version    v1.0
 * @date       2017-10-10
 */

#include "main.h"


void key_init(void)
{
    GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_1 ;
	gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;//����
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
}





    key_states_e key_read()
{
	key_states_e key_states;
	/*****�ٴ˴���Ӷ�ȡ�����ĳ���****/
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) ==1 )
	{
		key_states = KEY_UP;
	}
	else
	{
		key_states = KEY_DOWN;
	}



	/*****��ӽ���****/
	return key_states;
}




key_msg_e key_scan(void)
{
		static key_msg_e  key_now_msg = KEY_S1;     //��ΰ�����״̬
		static key_msg_e  key_last_msg = KEY_S1;     //�ϴΰ�����״̬
		key_msg_e key_return_msg = KEY_S1;//��������
		static unsigned int  keytime = 0;
    switch (key_last_msg)
    {
      case KEY_S1:                 // ������ʼ̬
			if (KEY_DOWN == key_read()) //���������£��ȴ�ȷ��
			{
				key_now_msg = KEY_S2;
			}

	    break;

        case KEY_S2:                   // ����ȷ��̬
				if (KEY_DOWN == key_read())
				{
						keytime = keytime + SCAN_TIME;
						if(keytime >= DOWN_TIME)
						{
						    key_now_msg = KEY_S3;// ����̰�״̬
						}
						else
						{
							key_now_msg = KEY_S2;// ά�ֵ�ǰ״̬
						}

        }
				else if(KEY_UP == key_read())//�������̧��˵���󴥷�
				{
						key_now_msg = KEY_S1; // ������̧��ת����������ʼ̬
						keytime = 0;
				}
	  	break;

        case KEY_S3://�̰�״̬
            if (KEY_DOWN == key_read())
            {
				keytime = keytime + SCAN_TIME;
				if(keytime >= HOLD_TIME)
				{
					  key_now_msg = KEY_S4;// ���볤��״̬
				}
				else
				{
					key_now_msg = KEY_S3;// ά�ֵ�ǰ״̬
				}

            }
            else if(KEY_UP == key_read())//�������̧��˵���̰�һ�����
            {
				key_now_msg = KEY_S1; // ������̧��ת����������ʼ̬
				keytime = 0;
			}
			break;
		case KEY_S4://����״̬
			if (KEY_DOWN == key_read())
			{
                key_now_msg = KEY_S4;
				key_return_msg = KEY_S4;
			}
			else if(KEY_UP == key_read())//�������̧��˵���̰�һ�����
			{
				key_now_msg = KEY_S1; // ������̧��ת����������ʼ̬
				keytime = 0;
			}
	break;
   }//end switch
		
	 
		if(key_now_msg == KEY_S1)
		{
				if(key_last_msg == KEY_S3)
				key_return_msg = KEY_S3;//�̰��ɿ���ŷ��ض̰���Ϣ
		}
		key_last_msg = key_now_msg;
		return key_return_msg;                            //���ذ���ֵ
}

