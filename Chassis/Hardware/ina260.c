/**********************************************************************************************************
 * @文件     ina260.c
 * @说明     输入输出电压/电流读取
 * @版本  	 V1.5
 * @作者     黄志雄
 * @日期     2020.1
**********************************************************************************************************/
#include "ina260.h"
INA260 INA260_1;//输入
INA260 INA260_2;//输出
#define WAITING_TIME 0.05 // us  发送数据后的接收时间
/**********************************************************************************************************
*函 数 名: INA_READ_Current
*功能说明: 读取输入输出电流
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void INA_READ_Current()
{
	float Cur = 0.0f;
	Cur = INA260_Read(INA260_1_ID << 1, REG_CURRENT);
	INA260_1.Current = LSB_CURRENT * Cur;
	Cur = INA260_Read(INA260_2_ID << 1, REG_CURRENT);
	INA260_2.Current = LSB_CURRENT * Cur;
}
/**********************************************************************************************************
*函 数 名: INA_READ_Vol
*功能说明: 读取输入输出电压
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void INA_READ_Vol()
{
	float Vol = 0.0f;
	Vol = INA260_Read(INA260_1_ID << 1, REG_VOLTAGE);
	INA260_1.Voltage = LSB_VOLTAGE * Vol;
	Vol = INA260_Read(INA260_2_ID << 1, REG_VOLTAGE);
	INA260_2.Voltage = LSB_VOLTAGE * Vol;
}
/**********************************************************************************************************
*函 数 名: INA_READ_Power
*功能说明: 读取输入输出功率
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void INA_READ_Power()
{
//	unsigned short power = 0;
//	power = INA260_Read(INA260_1_ID << 1, REG_POWER);
//	INA260_1.Power = LSB_POWER * power;

//	power = INA260_Read(INA260_2_ID << 1, REG_POWER);
//	INA260_2.Power = LSB_POWER * power;
	
	INA260_1.Power = ABS(INA260_1.Voltage * INA260_1.Current)/1000; //mw
	INA260_2.Power = ABS(INA260_2.Voltage * INA260_2.Current)/1000;
}
/**********************************************************************************************************
*函 数 名: INA260_Read
*功能说明: 读取i2c
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
short INA260_Read(u8 address, u8 reg)
{
	u8 templ = 0, temph = 0;
	short temp = 0;
	IIC_Start();
	IIC_Send_Byte(address); //发送低地址
	IIC_Wait_Ack();
	IIC_Send_Byte(reg); //发送低地址
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(address + 1); //进入接收模式
	IIC_Wait_Ack();
	delay_us_f(WAITING_TIME); //增加此代码通信成功！！！
	temph = IIC_Read_Byte();  //读寄存器 3
	IIC_Ack();
	templ = IIC_Read_Byte(); //读寄存器 3
	IIC_NAck();
	IIC_Stop(); //产生一个停止条件
	temp = (short)temph << 8 | templ;
	return temp;
}


