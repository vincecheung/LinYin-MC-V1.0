//************************************************************************
//       取得06CPU用户参数区的设置数据[#70~#89][$FE8C~$FEB2]
//
//   06CPU 用户参数70开始为设定参数，开始地址FE8C~FEB2，预计使用20个
//   #70   过压报警门限             (设置) $FE8C
//   #71   转速换算PWM脉宽          (设置) $FE8E   (0.1Hz对应的脉冲数/s)*100(扩大100倍)取整;60 * F（0.1Hz）/极对数/60s * 每转脉冲数
//   #72   直流电流量程最大值       (设置) $FE90
//   #73   直流电压量程最大值       (设置) $FE92
//   #74   欠压报警门槛             (设置) $FE94
//   #75   电机过热报警门槛         (设置) $FE96
//   #76   控制器散热器过热报警门槛 (设置) $FE98
//   #77   NTC热敏电阻选择          (设置) $FE9A
//   #78   油门0速开度              (设置) $FE9C
//   #79   油门100%开度对应的频率   (设置) $FE9E
//   #80   电机运行频率限幅值       (设置) $FEA0
//   。。。。。#89（$FEB2)
//************************************************************************
#ifndef MITY_DATA_H
#define MITY_DATA_H

void fe_init(void);

#endif

