#include <stdint.h>
#include <math.h>

#define ABS(X)  (((X)>0)?(X):-(X))

typedef struct
{
	/*****安排过度过程*******/
	float x1;    //跟踪微分期状态量
	float x2;    //跟踪微分期状态量微分项
	float r;     //时间尺度
	float h;     //ADRC系统积分时间
	uint16_t N0; //跟踪微分器解决速度超调h0=N*h

	float h0;
	float fh;//最速微分加速度跟踪量

	/*****扩张状态观测器*******/
	/******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
	float z1;
	float z2;
	float z3; //根据控制对象输入与输出，提取的扰动信息
	float e;  //系统状态误差
	float y;  //系统输出量
	float fe1;
	float fe2;
	float beta_01;
	float beta_02;
	float beta_03;
	float zeta;   //线性段的区间长度

	/**********系统状态误差反馈率*********/
	float e0; //状态误差积分项
	float e1; //状态偏差
	float e2; //状态量微分项
	float u0; //非线性组合系统输出
	float u;  //带扰动补偿后的输出
	float b0; //扰动补偿


	/*********第一种组合形式*********/
	float k_0; //线性
	float k_1; //非线性组合参数
	float k_2; //u0=beta_1*e1+beta_2*e2+(beta_0*e0);

	/*********第二种组合形式*********/
	float alpha1; //u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
	float alpha2; //0<alpha1<1<alpha2
	

	/*********第三种组合形式*********/
	float h1;    //u0=-fhan(e1,e2,r,h1);
	uint16_t N1; //跟踪微分器解决速度超调h0=N*h

	/*********第四种组合形式*********/
	float c;    //u0=-fhan(e1,c*e2*e2,r,h1);
}Fhan_Data;

//const float ADRC_Unit[3][15] =
//{
//	// TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合
//	//  r     h      N                 beta_01   beta_02    beta_03     b0       beta_0  beta_1  beta_2   N1     C    alpha1  alpha2
//	{ 300000 ,0.005 , 2,               100,      1000,      2000,     0.001,    0.002,   1.0,      0.0005,    5,    5,    0.8,   1.5,    50 },
//	{ 300000 ,0.005 , 2,               100,      1000,      2000,     0.001,    0.002,   1.0,      0.0005,    5,    5,    0.8,   1.5,    50 },
//	{ 50000  ,0.005 , 30,              100,      2000,      10000,    5    ,    0.002,   10,        0.001,    5,    5,    0.5,   1.05,   50 },
//};


//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
const float ADRC_Unit[3][15] =
{
	//TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO          扰动补偿           非线性组合
	//  r      h      N                beta_01   beta_02    beta_03      b0         k_0      k_1    k_2       N1     C    alpha1  alpha2   zeta
	{ 75000 ,  0.02,  2,               25,        100,        100,       0.001,      0.002,   1.0,   0.0005,    5,    5,    0.8,    1.5,     50 },
	{ 75000 ,  0.02,  2,               100,      1000,       2000,       0.001,      0.002,   1.0,   0.0005,    5,    5,    0.8,    1.5,     50 },
	{ 50000 ,  0.005, 30,              100,      2000,      10000,           5,      0.002,   10,     0.001,    5,    5,    0.5,   1.05,     50 },
};

void ADRC_Init(Fhan_Data *fhan_Input1, Fhan_Data *fhan_Input2);

// sign符号函数
int16_t Sign_ADRC(float Input);

int16_t Fsg_ADRC(float x, float d);

//ADRC最速跟踪微分器TD，改进的算法fhan
void Fhan_ADRC(Fhan_Data *fhan_Input, float expect_ADRC);

float Fal_ADRC(float e, float alpha, float zeta);

void ESO_ADRC(Fhan_Data *fhan_Input);

float Constrain_Float(float amt, float low, float high);

void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input);

/*-- ADRC 控制的完整流程 --*/
float ADRC_Control(Fhan_Data *fhan_Input, float expect_ADRC, float feedback_ADRC);
