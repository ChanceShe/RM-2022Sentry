#include <stm32f4xx.h>
#include "main.h"

RC_Ctl_t RC_CtrlData;
static InputMode_e inputmode = STOP;//REMOTE_INPUT;   //输入模式设定
int rotate_num_ture=0;		//掉头标志
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

  //RemoteControlProcess(&(RC_CtrlData.rc));

  switch(GetInputMode( ))
    {
    case REMOTE_INPUT:
    {
      //遥控器控制模式
      RemoteControlProcess(&(RC_CtrlData.rc));
      SetWorkState(NORMAL_STATE);
    }
    break;
    case KEY_MOUSE_INPUT:
    {
      //键盘控制模式
      MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);
      SetWorkState(NORMAL_STATE);
    }
    break;
    case STOP:
    {
      SetWorkState(PREPARE_STATE);
      //紧急停车
    }
    break;
    }
}
