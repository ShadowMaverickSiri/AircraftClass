#pragma once

//导弹工具基类
#include<vector>
#include<string>
#include <time.h>
#include<array>
#include<Eigen/Dense>
#include<numeric>
#include<algorithm>
#include<iostream>
#include<fstream>
#include<sstream>
#include <cmath>



using namespace Eigen;

class SimTools
{
public:
	SimTools();
	//virtual void MissileStep()const = 0;
	virtual ~SimTools();
	//结构体(节点)
	typedef struct
	{
		double x;
		double y;
	}NODE_xy;
	
	//基础功能函数（内联）
	//符号函数
	template<class Type>
	static Type sign(Type x) {
		Type res;
		if (x > 0.0) { res = 1; }
		else if (x < 0.0) { res = -1; }
		else { res = 0; }
		return res;
	}
	
	//最大值函数，对比两个数
	template<class Type>
	static Type Max(Type a, Type b) {
		if (a >= b)
		{
			return a;
		}
		else
		return b;
	}

	//最小值函数
	template<class Type>
	static Type Min(Type a, Type b) {
		if (a <= b)
		{
			return a;
		}
		else
			return b;
	}
	
	//找出数组中的最大值,返回最大值和所在索引
	template<class Type>
	static Type FindAbsMax(std::vector<Type> INPUT, int &o){
		Type y = INPUT[0];
		int n, m;
		m = INPUT.size();
		o = 0;
		for (n = 0;n < m;n++)
		{
			if (abs(y) < abs(INPUT[n]))
			{
				y = INPUT[n];
				o = n;
			}

		}
		o = n + 1;
		return y;
	}

	template<class Type>
	static Type FindAbsMax(Type INPUT[], int &o) {
		Type y = INPUT[0];
		int n, m;
		m = INPUT.size();
		o = 0;
		for (n = 0;n < m;n++)
		{
			if (abs(y) < abs(INPUT[n]))
			{
				y = INPUT[n];
				o = n;
			}

		}
		o = n + 1;
		return y;
	}

	//求3维向量的二范数
	static double BoundNorm2(Vector3d Vec)
	{
		return sqrt(Vec[0] * Vec[0] + Vec[1] * Vec[1] + Vec[2] * Vec[2]);
	}
	static double BoundNorm2(double Vec[3])
	{
		return sqrt(Vec[0] * Vec[0] + Vec[1] * Vec[1] + Vec[2] * Vec[2]);
	}


	//3维向量归一化
	inline Vector3d  Normalize(Vector3d Vec)
	{
		Vector3d newVector;
		newVector(0) = Vec(0) / BoundNorm2(Vec);
		newVector(1) = Vec(1) / BoundNorm2(Vec);
		newVector(2) = Vec(2) / BoundNorm2(Vec);
		return newVector;
	}

	//数值计算函数：
	//插值相关
	static double Interp2(double x, std::vector<double>xx, std::vector<double>yy);//两点线性插值
	static double Interp2(double x, int n, double xx[], double yy[]);//两点线性插值
	static double Interp_L7(double x, std::vector<double>xx, std::vector<double>yy);//部分线性插值-前后7个点（容器）
	static double Interp_L7(double x, int n, double xx[], double yy[]);//部分线性插值-前后7个点（数组）
	static double Insert_EL(double x, std::vector<double>xx, std::vector<double>yy);//全局插值																			//三维插值函数
	static double Interp3(double x[], double y[], double z[], int n, int m, double u, double v);//名称：二维插值函数
	static double Regulate180(double angle);		 //将角度限制在[-π，π]中的函数
	//数值积分
	void RangKutta(double t, double y[], int n, double h, int k, double z[], double *ip);
	virtual void Equation(double t, double state[], double d[], double* ip);

	//坐标转换相关求解函数
	static void E2Gps(double PX,double PY,double PZ,double &longti,double &lati,double &h);//根据地心坐标求解经纬度（俊忠）
	static void Gps2E(double *Pe, double *gps);//由GPS转地心坐标(郑博士)
	
	static Vector3d Sitecompute_func(Vector3d Gps_m,double phi_angle,double Ht0,double R_r);//通过相对距离和角度来解算目标经纬度(待改)
	static void Sitecompute_func(Vector3d Gps_m, double phi_angle, double Ht0, double R_r, Vector3d &Gps_t);
	static double PhiL_Compute(const Vector3d &GPS_A, const  Vector3d & GPS_B);
	static double GPS_R_Compute(const Vector3d &GPS_A, const  Vector3d & GPS_B);

	//Eigen库相关功能函数(矩阵、向量计算)
	static void Gps2E(const Vector3d &GPS,Vector3d &PE);//由GPS转地心坐标
	static void E2Gps(const Vector3d &PE, Vector3d &GPS);//由地心坐标→GPS信息#2：球面几何法
	static void E2GPS(double pe[3],double gps[3]); //地心坐标→GPS坐标#2 (不出现奇异值)

	static Vector3d E2Gps_Newton(Vector3d E);//由地心坐标→GPS信息#1：牛顿法（速度慢）
	static Matrix3d CoordinateTransM3(double angle, int axis);//沿着三轴旋转生成的矩阵(三维) axis = 1:绕x轴转，axis = 2:绕y轴转，axis = 3:绕z轴转

	static void VnFromV(double theta, double phi_v, double V, Vector3d &VN); //根据初始绝对速度求解北天东三向速度（Eigen版本） //注：角度为弧度！
	static void VeFromV(double V,const Vector3d &GPS, Vector3d &VE); //根据经纬高和绝对速度求解地心三向速度						 
	static bool Ifpointraingle(double pot[2], NODE_xy A, NODE_xy B, NODE_xy C);
	static Matrix3d MakeE2NFromGps(double longti,double lati);  //从经纬度得到地心到地理系的转换矩阵
	static Matrix3d MakeN2EFromGps(double longti, double lati);  //从经纬度得到地理到地心系的转换矩阵
	static void PnFromPe(const Vector3d &Pxyz_e, Vector3d &Pxyz_n, double longti, double lati); //从地心相对位置得到某经纬度下的地理系坐标
	static void MakeGPSFromPn(const Vector3d &Gps_O,double Pz_n, double Px_n,double &longti_t,double &lati_t,double Ht0);//从地理系求解经纬度

	static void MakeXYZFromGPS0(const Vector3d& Gps0, Vector3d Gps,Vector3d &Pxyz); //根据某一个点的经纬度坐标，求解它在某个参考点当地地理系下的XYZ


	//随机数序列
	static double Rand_01();						//01区间均匀分布
	static double Rand_ab(double a, double b);		//ab区间均匀分布
	static double Rand_N01();						//标准正态分布N(0,1)
	static double Rand_N(double mu, double sigma2);//非标准正态分布，均值mu，方差sigma^2

	static double regulate(double psi)
	{
		double temp = psi - 360 * floor(psi / 360);
		if (fabs(temp) > 180)	psi = temp - 360 * sign(temp);
		else	psi = temp;
		return psi;
	}

	//文件操作相关函数
	static double **Input_data(char *p, int k);//从txt、dat文件中导入数据到数组中
	static int TxtRowcount(std::string txtname);//获取文本数据行数
	static int TxtRowcount(char *p);//获取文本数据行数

	static int Index1(double t, std::vector<double> idx_t);//插值算法.判断索引函数
	static int Index2(double i, std::vector<std::vector<double>> idx_t);//插值算法.判断索引函数(对二维数组)

	static  std::string NumberToString(int x);

	//相关功能函数
	static double Caculate_g(double h);//根据高度求重力加速度
	static double SoundSpeedFromH(double h);//高度与声速之间的函数关系
	static void AirParaFromH(double hm, double &Pa, double &g, double &soundspeed, double &rou);//根据高度求解相关大气参数
	static double RhoFromH(double h);//高度与声速之间的函数关系
	static double CalVFromMaH(double Ma, double hm);  //根据马赫数和高度解算速度
	static double BigCircle(double Lon_S, double Lat_S, double Lon_T, double Lat_T); //根据起始经纬度计算大圆线航程(看成球体)
	static double Vincenty_distance(double latitude1,double longtitude1,double latitude2,double longtitude2); //根据经纬度求解地球两点距离（Vincenty公式-直线）
	static void Vincenty_inverse(double L1, double B1, double L2, double B2, double &s,double &A12,double &A21);//文森特公式逆求解：根据两点的大地经纬度B1,L1 B2,L2计算大地弧长s和两个方位角
	static void Vincenty(double L1, double B1, double A12, double s, double &L2, double &B2 ,double &A21);//文森特公式正解：根据点1经纬度和大地方位角A12和大地线长s推求点2经纬度和大地方位角A21

	static double Angle60To10(double degree,double minute,double second);//经纬度 度分秒→10进制度
	static void Angle10To60(double value, int &degree, int &minute, double &second);//经纬度 10进制度→度分秒

	//矩阵相乘（郑佳琳）
	static void MatMul(double a[3][3], double b[3], double c[3])
	{
		int i, j;
		for (i = 0; i < 3; i++)
		{
			c[i] = 0;
			for (j = 0; j < 3; j++)
			{
				c[i] += a[i][j] * b[j];
			}
		}
	}
	//矩阵转置（郑佳琳）
	static	void MatTrans(double a[3][3], double b[3][3])
	{
		int i, j;
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				b[j][i] = a[i][j];
			}
		}
	}
	//矩阵乘法（郑佳琳）
	static void MatMul33(double a[3][3], double b[3][3], double c[3][3])
	{
		int i, j, k;
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				c[i][j] = 0;
				for (k = 0; k < 3; k++)
				{
					c[i][j] += a[i][k] * b[k][j];
				}
			}
		}
	}

};


