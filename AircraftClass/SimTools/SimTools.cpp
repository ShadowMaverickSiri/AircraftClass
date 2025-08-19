#include "SimTools.h"

//相关物理常数
constexpr double PI = 3.14159265358979323846;
constexpr double Rad = 180. / PI;
constexpr double Re = 6378137.0;
constexpr double EPS = 0.00000000001;
constexpr double E_earth = 1 / 298.257222101;  //地球扁率
constexpr double RE_LONG = 6378137.; //地球长半轴  CGCS2000
constexpr double RE_SHORT = 6356752.3141;//地球短半轴  CGCS2000
constexpr double ECCE1 = 0.081819191042816;//地球第一偏心率 CGCS2000
constexpr double ECCE2 = 0.082094438151917;//地球第二偏心率 CGCS2000

SimTools::SimTools()
{
}

SimTools::~SimTools()
{
}

double SimTools::Regulate180(double angle)
{
	double temp = angle - 2 * PI*floor(angle / 2 / PI);
	if (fabs(temp) > PI)	angle = temp - 2 * PI*sign(temp);
	else	angle = temp;
	return angle;
}

void SimTools::RangKutta(double t, double y[], int n, double h, int k, double z[], double * ip)
{
	double a[4], tt;
	double *b, *d;

	b = (double*)malloc(n*sizeof(double));
	d = (double*)malloc(n*sizeof(double));

	a[0] = h / 2.0; a[1] = a[0];
	a[2] = h;	  a[3] = h;

	for (int i = 0;i <= n - 1;i++)
		z[i*k] = y[i];
	for (int l = 1;l <= k - 1;l++)
	{
		Equation(t, y, d, ip);
		for (int i = 0;i <= n - 1;i++)
			b[i] = y[i];
		for (int j = 0;j <= 2;j++)
		{
			for (int i = 0;i <= n - 1;i++)
			{
				y[i] = z[i*k + l - 1] + a[j] * d[i];
				b[i] = b[i] + a[j + 1] * d[i] / 3.0;
			}
			tt = t + a[j];
			Equation(tt, y, d, ip);
		}
		for (int i = 0;i <= n - 1;i++)
			y[i] = b[i] + h*d[i] / 6.0;

		for (int i = 0;i <= n - 1;i++)
			z[i*k + l] = y[i];

		//t = t+h;
	}
	free(b); free(d);
}

void SimTools::Equation(double t, double state[], double d[], double * ip)
{

}

double SimTools::Interp2(double x, std::vector<double>xx, std::vector<double>yy)
{
	int i, j, k, m;
	double z, s;
	z = 0.0;
	int n = xx.size();
	if (n < 1)
		return z;
	if (n == 1)
	{
		z = yy[0];
		return z;
	}
	if (n == 2)
	{
		z = yy[0] + (x - xx[0])*(yy[1] - yy[0]) / (xx[1] - xx[0]);
		return z;
	}
	i = 0;
	if (xx[1]>xx[0])
		while ((xx[i] < x) && (i < n))
			i = i + 1;
	else
		while ((xx[i] > x) && (i < n))
			i = i + 1;
	if (i == 0)
		z = yy[0];
	else
		z = yy[i - 1] + (x - xx[i - 1])*(yy[i] - yy[i - 1]) / (xx[i] - xx[i - 1]);
	return z;
}

double SimTools::Interp2(double x, int n, double xx[], double yy[])
{
	int i, j, k, m;
	double z, s;
	z = 0.0;
	if (n < 1)
		return z;
	if (n == 1)
	{
		z = yy[0];
		return z;
	}
	if (n == 2)
	{
		z = yy[0] + (x - xx[0])*(yy[1] - yy[0]) / (xx[1] - xx[0]);
		return z;
	}
	i = 0;
	if (xx[1]>xx[0])
		while ((xx[i] < x) && (i < n))
			i = i + 1;
	else
		while ((xx[i] > x) && (i < n))
			i = i + 1;
	if (i == 0)
		z = yy[0];
	else
		z = yy[i - 1] + (x - xx[i - 1])*(yy[i] - yy[i - 1]) / (xx[i] - xx[i - 1]);
	return z;
}

double SimTools::Interp_L7(double x, std::vector<double> xx, std::vector<double> yy)
{
	int i, j, k, m;
	double z, s;
	z = 0.0;
	int n = xx.size();
	if (n < 1)
		return z;
	if (n == 1)
	{
		z = yy[0];
		return z;
	}
	if (n == 2)
	{
		z = yy[0] + (x - xx[0])*(yy[1] - yy[0]) / (xx[1] - xx[0]);
		return z;
	}
	i = 0;
	while ((xx[i] < x) && (i < n)) i = i + 1;
	k = i - 4;//往前取三个数
	if (k < 0) k = 0;
	m = i + 3;//往后取三个数
	if (m > n - 1) m = n - 1;
	//对索引k到m中间的数取插值
	for (i = k;i <= m;i++)
	{
		s = 1.0;
		for (j = k;j <= m;j++)
			if (j != i) s = s*(x - xx[j]) / (xx[i] - xx[j]);
		z = z + s*yy[i];
	}
	return z;
}

double SimTools::Interp_L7(double x, int n,double xx[], double yy[])
{
	int i, j, k, m;
	double z, s;
	z = 0.0;
	if (n < 1)
		return z;
	if (n == 1)
	{
		z = yy[0];
		return z;
	}
	if (n == 2)
	{
		z = yy[0] + (x - xx[0])*(yy[1] - yy[0]) / (xx[1] - xx[0]);
		return z;
	}
	i = 0;
	while ((xx[i] < x) && (i < n)) i = i + 1;
	k = i - 4;//往前取三个数
	if (k < 0) k = 0;
	m = i + 3;//往后取三个数
	if (m > n - 1) m = n - 1;
	//对索引k到m中间的数取插值
	for (i = k;i <= m;i++)
	{
		s = 1.0;
		for (j = k;j <= m;j++)
			if (j != i) s = s*(x - xx[j]) / (xx[i] - xx[j]);
		z = z + s*yy[i];
	}
	return z;
}

double SimTools::Insert_EL(double x, std::vector<double> xx, std::vector<double> yy)
{
	int i, j;
	double res = 0.;
	double l = 1.;
	int n = xx.size();
	for (i = 0;i < n;i++)
	{
		l = 1.;
		for (j = 0;j < n;j++)
		{
			if (i != j)l = l*(x - xx[j]) / (xx[i] - xx[j]);
		}
		res += yy[i] * l;
	}
	return res;
}

double SimTools::Interp3(double x[], double y[], double z[], int n, int m, double u, double v)
{
	int nn, mm, ip, iq, i, j, k, l;
	double b[3], h, w;
	nn = 3;
	if (n <= 3) { ip = 0;nn = n; }
	else if (u <= x[1]) ip = 0;
	else if (u > x[n - 2]) ip = n - 3;
	else
	{
		i = 1;j = n;
		while (((i - j) != 1) && ((i - j) != -1))
		{
			l = (i + j) / 2;
			if (v < y[l - 1]) j = 1;
			else i = 1;
		}
		if (fabs(v - y[i - 1]) < fabs(v - y[j - 1]))iq = i - 2;
		else iq = i - 1;
	}
	mm = 3;
	if (m <= 3) { iq = 0;mm = m; }
	else if (v <= y[1])iq = 0;
	else if (v >= y[m - 2])iq = m - 3;
	else
	{
		i = 1;j = m;
		while (((i - j) != 1) && ((i - j) != -1))
		{
			l = (i + j) / 2;
			if (v < y[l - 1])j = 1;
			else i = l;
		}
		if (fabs(v - y[i - 1]) < fabs(v - y[j - 1]))iq = i - 2;
		else iq = i - 1;
	}
	for (i = 0;i < nn;i++)
	{
		b[i] = 0;
		for (j = 0;j <= mm - 1;j++)
		{
			k = m*(ip + i) + (iq + j);
			h = z[k];
			for (k = 0;k < mm;k++)
				if (k != j)
					h = h*(v - y[iq + k]) / (y[iq + j] - y[iq + k]);
			b[i] = b[i] + h;
		}
	}
	w = 0;
	for (i = 0;i < nn;i++)
	{
		h = b[i];
		for (j = 0;j <= nn - 1;j++)
		{
			if (j != i)
				h = h*(u - x[ip + j]) / (x[ip + i] - x[ip + j]);
			w = w + h;
		}
	}
	return w;
}

double SimTools::Caculate_g(double h) {
	double g;
	int i;
	double ht, e, gn, r, hhb, ts;
	double vm2;
	static double w[30] =
	{
		95000.,	51000., 270.65, -0.002800,	   75.944,
		51000.,	47000., 270.65,  0.000000,	  110.906,
		47000.,	32000., 228.65,  0.002800,	  868.015,
		32000.,	20000., 216.65,  0.001000,	 5474.870,
		20000.,	11000., 216.65,  0.000000,	22632.000,
		11000.,		0., 288.15, -0.006500, 101325.000,
	};

	if (h <= 10)		// 2013-11-28 高度过低还是要计算的
	{
		h = 10;
	}
	else if (h > 51000.0)
	{
		h = 51000.0;
	}

	gn = 9.80665;
	r = 6356766.0;
	e = -1.0*gn / 287.05287;
	ht = r / (r + h)*h;
	i = 30;
	do
	{
		i = i - 5;
	} while ((ht > w[i]) && (w[i] < 95000.));
	g = 9.7858 - 0.00000094114*h;

	return g;
}

double SimTools::CalVFromMaH(double Ma, double hm) {
	double ht, e, gn, r, hhb, ts;
	double vm2;
	static double w[30] =
	{
		95000.,	51000., 270.65, -0.002800,	   75.944,
		51000.,	47000., 270.65,  0.000000,	  110.906,
		47000.,	32000., 228.65,  0.002800,	  868.015,
		32000.,	20000., 216.65,  0.001000,	 5474.870,
		20000.,	11000., 216.65,  0.000000,	22632.000,
		11000.,		0., 288.15, -0.006500, 101325.000,
	};

	if (hm <= 10)
	{
		hm = 10;
	}
	else if (hm > 51000.0)
	{
		hm = 51000.0;
	}

	gn = 9.80665;
	r = 6356766.0;
	e = -1.0*gn / 287.05287;
	ht = r / (r + hm)*hm;
	int i = 30;
	do
	{
		i = i - 5;
	} while ((ht > w[i]) && (w[i] < 95000.));
	double g = 9.7858 - 0.00000094114*hm;
	hhb = ht - w[i + 1];
	ts = w[i + 2] + w[i + 3] * hhb;
	double a = sqrt(1.4*287.05287*ts);

	return a*Ma;
}

double SimTools::BigCircle(double Lon_S, double Lat_S, double Lon_T, double Lat_T)
{
	double d;
	d = Re*acos(sin(Lat_S / Rad)*sin(Lat_T / Rad) + cos(Lat_S/Rad)*cos(Lat_T/Rad)*cos(fabs( Lon_T - Lon_S )/Rad)  );
	return d;
}

double SimTools::Vincenty_distance(double longtitude1, double latitude1,double longtitude2, double latitude2)
{
	double R = 6378137.0;//
	double deltaLatitude = (latitude2 - latitude1)/Rad;
	double deltaLongtitude = (longtitude2 - longtitude1) / Rad;
	latitude1 = latitude1 / Rad;
	latitude2 = latitude2 / Rad;
	double a = pow(sin(deltaLatitude / 2), 2) + cos(latitude1)*cos(latitude2)*pow(sin(deltaLongtitude / 2), 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	double d = R * c;
	return d;
}

void SimTools::Vincenty_inverse(double L1, double B1, double L2, double B2, double &s, double &A12, double &A21)
{
	//double xx = (L2 - L1) / Rad;
	B1 /= Rad; L1 /= Rad; B2 /= Rad; L2 /= Rad; //把经纬度转换为弧度
	double sin_u1, sin_u2;
	double cos_u1, cos_u2;
	double lamda, deltaL;
	double sin_sigma,sin2_sigma, cos_sigma, tan_sigma;
	double sin_alpha,cos_alpha;
	double cos_2sigma_m;
	double C;
	double sigma;
	lamda = deltaL = L2 - L1;

	double tan_u1 = sqrt(1 - ECCE1 * ECCE1)*tan(B1);
	double tan_u2 = sqrt(1 - ECCE1 * ECCE1)*tan(B2);
	cos_u1 = sqrt(1 / (1 + tan_u1 * tan_u1));
	cos_u2 = sqrt(1 / (1 + tan_u2 * tan_u2));
	sin_u1 = sqrt(1 - cos_u1 * cos_u1);
	sin_u2 = sqrt(1 - cos_u2 * cos_u2);
	double lamda_last = 0.;
	while (true)
	{
		sin2_sigma = pow((sin(lamda)*cos_u2), 2) + pow(cos_u1*sin_u2 - sin_u1 * cos_u2*cos(lamda), 2);
		sin_sigma = sqrt(sin2_sigma);
		cos_sigma = sin_u1 * sin_u2 + cos_u1 * cos_u2*cos(lamda);
		double x = sin_sigma * sin_sigma + cos_sigma * cos_sigma;
		tan_sigma = sin_sigma / cos_sigma;
		sin_alpha = cos_u1 * cos_u2*sin(lamda)/sin_sigma;
		cos_alpha = sqrt(1 - sin_alpha * sin_alpha);
		cos_2sigma_m = cos_sigma - 2 * sin_u1*sin_u2 / pow(cos_alpha, 2);
		C = E_earth / 16 * pow(cos_alpha, 2)*(4 + E_earth * (4 - 3 * pow(cos_alpha, 2)));
		sigma = atan(tan_sigma);
		lamda = deltaL + (1 - C)*E_earth*sin_alpha*(sigma + C * sin_sigma*(cos_2sigma_m + C * cos_sigma*(-1 + 2 * cos_2sigma_m*cos_2sigma_m)));
		if ( fabs(lamda - lamda_last)  < EPS &&  fabs(lamda_last) >EPS)
		{
			break;
		}
		lamda_last = lamda; //记录上一拍值
	}
	double k2 = ECCE2* ECCE2 * cos_alpha*cos_alpha;
	double A = 1 + k2 / 16384 * (4096 + k2 * (-768 + k2 * (320 - 175 * k2)));
	double B = k2 / 1024 * (256 + k2 * (-128 + k2 * (74 - 47 * k2)));
	double delta_sigma = B * sin_sigma*(cos_2sigma_m + B / 4 * (cos_sigma*(-1 + 2 * cos_2sigma_m*cos_2sigma_m) - B / 6 * cos_2sigma_m*(-3 + 4*sin_sigma * sin_sigma)*(-3 + 4*cos_2sigma_m*cos_2sigma_m)));
	s = RE_SHORT * A*(sigma - delta_sigma);
	double tan_A12 = sin(lamda)*cos_u2 / (sin_u2*cos_u1 - cos(lamda)*sin_u1*cos_u2);
	double tan_A21 = sin(lamda)*cos_u1 / (sin_u1*cos_u2 - sin_u2 * cos_u1*cos(lamda));
	//A12 = atan(tan_A12)*Rad;
	//A21 = atan(tan_A21)*Rad;
	A12 = atan2(tan_A12,1)*Rad;
	A21 = atan2(tan_A21,1)*Rad;

	if (A12 < 0.)
	{
		A12 = A12 + 180.;
	}
	if (A21 < 0.)
	{
		A21 = A21 + 180.;
	}
	A21 = 360. - A21;

	//if (A12<0.)
	//{
	//	A12 = A12 + 180;
	//}


}

void SimTools::Vincenty(double L1, double B1, double A12, double s, double & L2, double & B2, double &A21)
{
	L1 /= Rad; B1 /= Rad; A12 /= Rad;
	double tan_u1,sin_u1,cos_u1, tan_sigma1, sin_alpha, cos_alpha,tan_A21;
	double k2,A,B,C,sigma, sigma1;
	tan_u1 = sqrt(1 - ECCE1 * ECCE1)*tan(B1);
	cos_u1 = sqrt(1 / (1 + tan_u1 * tan_u1));
	sin_u1 = sqrt(1 - cos_u1 * cos_u1);
	tan_sigma1 = tan_u1 / cos(A12);
	sin_alpha = sin(A12)*cos_u1;
	cos_alpha = sqrt(1 - sin_alpha * sin_alpha);
	k2 = ECCE2 * ECCE2*pow(cos_alpha, 2);
	A = 1 + k2 / 16384 * (4096 + k2 * (-768 + k2 * (320 - 175 * k2)));
	B = k2 / 1024 * (256 + k2 * (-128 + k2 * (74 - 47 * k2)));
	sigma = s / RE_SHORT / A;
	sigma1 = atan2(tan_sigma1,1);


	//迭代
	double sigma_m,delta_sigma;
	double sigma_last = 0.;
	while (true)
	{
		sigma_m = (2 * sigma1 + sigma)/2;
		delta_sigma = B * sin(sigma)*(cos(2 * sigma_m) + B / 4 * (cos(sigma)*(-1 + 2 * cos(2 * sigma_m)*cos(2 * sigma_m)) - B / 6 * cos(2 * sigma_m)*(-3 + 4 * sin(sigma)*sin(sigma))*(-3 + 4 * cos(2 * sigma_m)*cos(2 * sigma_m) ) ) );
		sigma = s / RE_SHORT / A + delta_sigma;
		if (fabs(sigma - sigma_last) < EPS &&  fabs(sigma_last) > EPS)
		{
			break;
		}
		sigma_last = sigma; //记录上一拍值
	}
	double tan_B2,tan_lamda;
	tan_B2 = (sin_u1 * cos(sigma) + cos_u1 * sin(sigma)*cos(A12) )/ (1 - E_earth) / pow(sin_alpha*sin_alpha +   pow( (sin_u1*sin(sigma) - cos_u1 * cos(sigma)*cos(A12)),2 )  , 0.5);
	tan_lamda = sin(sigma)*sin(A12) / (cos_u1*cos(sigma) - sin_u1 * sin(sigma)*cos(A12));
	
	//double cos_lamda;
	double lamda = atan2(tan_lamda, 1);
	if (lamda<0.)
	{
		lamda += 180./Rad;
	}
	////辅助计算B2(三角函数辅助角公式)
	//double a = sin_u1;
	//double b = cos_u1 * cos(lamda);
	//double c = cos(sigma);
	//double phi = atan(b / a);
	//double u2 = asin(c / sqrt(a * a + b * b))-phi;
	//tan_B2 = tan(u2) / sqrt(1 - ECCE1* ECCE1);

	C = E_earth / 16 * cos_alpha*cos_alpha*(4 + E_earth * (4 - 3 * cos_alpha*cos_alpha));
	double deltaL;
	deltaL = lamda - (1 - C)*E_earth*sin_alpha*(sigma + C * sin(sigma)*(cos(2 * sigma_m) + C * cos(sigma)*(-1 + 2 * cos(2 * sigma_m)*cos(2 * sigma_m) ) ));
	L2 = L1 + deltaL;
	tan_A21 = sin_alpha / (sin_u1*sin(sigma) - cos_u1 * cos(sigma)*cos(A12) );
	A21 = atan2(tan_A21,1);
	A21 *= Rad;
	B2 = atan2(tan_B2, 1)*Rad;
	L2 *= Rad;
}

double SimTools::Angle60To10(double degree, double minute, double second)
{
	return degree + minute / 60. + second / 3600.;
}

void SimTools::Angle10To60(double value, int & degree, int & minute, double & second)
{
	degree = (int)value ;
	minute = (int)((value - degree)*3600. / 60.);
	second = (value - degree - double(minute) / 60)*3600 ;
}

double SimTools::SoundSpeedFromH(double h) {
	double T, P, Rou;
	if (h <= 11000)
	{
		T = 288.15 - 0.0065*h;
		P = 101325.2*T / 288.15*pow(T / 288.15, 4.25588);
		Rou = 1.22505*pow(T / 288.15, 4.25588);
	}
	else if (h <= 20000.0)
	{
		T = 216.65;
		P = 22632.04 / exp((h - 11000) / 6341.62);
		Rou = 0.3639176 / exp((h - 11000) / 6341.62);
	}
	else
	{
		T = 216.65;
		P = 5474.87*pow(1.0 + 0.001*(h - 20000) / T, -9.80665 / 0.001 / 287.05287);
		Rou = P / 287.05287 / (T + 0.001*(h - 20000.0));
	}
	double tmp = sqrt(T);
	return 20.1*sqrt(T);

}

void SimTools::AirParaFromH(double hm, double &Pa, double &g, double &soundspeed, double &rou)
{
	double g0 = 9.80665;
	double r0 = 6356.766;
	double ph0 = 101325;
	double rho0 = 1.225;

	double m_R = 287.05287;//专用气体常数
	double H;//H
	double Hb;//相应层下界的位势高度,m
	double Tb;//相应层下界的温度,K
	double L;//L
	double Pb;//相应层下界的压力,Pa
	double T;
	static double w[6][5] = {
		95000., 51000., 270.65, -0.002800,     75.944,
		51000., 47000., 270.65,  0.000000,    110.906,
		47000., 32000., 228.65,  0.002800,    868.015,
		32000., 20000., 216.65,  0.001000,   5474.870,
		20000., 11000., 216.65,  0.000000,  22632.000,
		11000.,     0., 288.15, -0.006500, 101325.000
	};

	if (hm > 95e3)//高度限幅
	{
		hm = 95e3;
	}

	H = hm * r0 * 1000 / (r0 * 1000 + hm);
	int i = 6;
	do
	{
		i = i - 1;
	} while ((H > w[i][0]) && (w[i][0] < 95000.));

	Hb = w[i][1];
	Tb = w[i][2];
	L = w[i][3];
	Pb = w[i][4];
	T = Tb + L * (H - Hb);
	if (w[i][3] != 0.0)
	{
		Pa = Pb * pow((1.0 + L / Tb * (H - Hb)), -g0 / (m_R*L));//大气压力
	}
	else
	{
		Pa = Pb * exp(-g0 / (m_R*T)*(H - Hb));//大气压力
	}
	rou = Pa / (m_R*T);
	g = g0 * r0 * 1000 * r0 * 1000 / ((r0 * 1000 + hm)*(r0 * 1000 + hm));
	soundspeed = sqrt(1.4*m_R*T);

}

double SimTools::RhoFromH(double h) {
	double T, P, Rou;
	if (h <= 11000)
	{
		T = 288.15 - 0.0065*h;
		P = 101325.2*T / 288.15*pow(T / 288.15, 4.25588);
		Rou = 1.22505*pow(T / 288.15, 4.25588);
	}
	else if (h <= 20000.0)
	{
		T = 216.65;
		P = 22632.04 / exp((h - 11000) / 6341.62);
		Rou = 0.3639176 / exp((h - 11000) / 6341.62);
	}
	else
	{
		T = 216.65;
		P = 5474.87*pow(1.0 + 0.001*(h - 20000) / T, -9.80665 / 0.001 / 287.05287);
		Rou = P / 287.05287 / (T + 0.001*(h - 20000.0));
	}
	double tmp = sqrt(T);
	return Rou;
}

void SimTools::E2Gps(double PX, double PY, double PZ, double & longti, double & lati, double & h)
{
	double Re = 6378137.;
	double Re1 = 6356752.3;
	double e0 = 0.0818191909289063;
	double nn = Re;
	double hh = sqrt(PX*PX + PY*PY + PZ*PZ) - sqrt(Re*Re1);
	double bb = atan(PZ / sqrt(PX*PX + PY*PY) / (1 - e0*e0*nn / (nn + hh)));

	for (int i = 0;i < 3;i++)
	{
		nn = Re / (sqrt(1 - e0*e0*sin(bb)*sin(bb)));
		hh = sqrt(PX*PX + PY*PY) / cos(bb) - nn;
		bb = atan2(PZ, sqrt(PX*PX + PY*PY)*(1 - e0*e0*nn / (nn + hh)));
	}
	longti = atan2(PY, PX)*Rad;
	lati = bb*Rad;
	h = hh;
}

void SimTools::Gps2E(double *Pe, double *gps)
{
	double log, lat, hm;
	double RAD = 180.0 / 3.14159265358979323846;
	double _Ecllipse_A_84 = 6378137.000;												//84下长轴
	double _Ecllipse_B_84 = 6356752.3142;											//84下短轴
	double _Ecllipse_E_84_1 =																//84下第一偏心率
		sqrt(pow(_Ecllipse_A_84, 2) - pow(_Ecllipse_B_84, 2)) / _Ecllipse_A_84;
	static const double _Ecllipse_E_84_2 =																//84下第二偏心率
		sqrt(pow(_Ecllipse_A_84, 2) - pow(_Ecllipse_B_84, 2)) / _Ecllipse_B_84;
	static const double _Ecllipse_EE_84_1 =																//84下第一偏心率平方
		(pow(_Ecllipse_A_84, 2) - pow(_Ecllipse_B_84, 2)) / pow(_Ecllipse_A_84, 2);
	static const double _Ecllipse_EE_84_2 =																//84下第二偏心率平方
		(pow(_Ecllipse_A_84, 2) - pow(_Ecllipse_B_84, 2)) / pow(_Ecllipse_B_84, 2);
	double n;
	log = gps[0];
	lat = gps[1];
	hm = gps[2];
	n = _Ecllipse_A_84 / sqrt(1 - _Ecllipse_EE_84_1 * pow(sin(lat / RAD), 2));			//该N值与ppt上的公式不一致，但从效果看，这个是对的
	Pe[0] = (n + hm) * cos(lat / RAD) * cos(log / RAD);
	Pe[1] = (n + hm) * cos(lat / RAD) * sin(log / RAD);
	Pe[2] = (n * (1 - _Ecllipse_EE_84_1) + hm) * sin(lat / RAD);
}

Vector3d SimTools::Sitecompute_func(Vector3d Gps_m, double phi_angle, double Ht0, double R_r)
{
	double longti, lati, hm0;
	Vector3d tDR, tDR_pe, Pe, Pte;
	Vector3d Gps_t;
	Matrix3d dTe, eTd;
	double Rm_now;

	tDR(0) = R_r * cos(-phi_angle / Rad);
	tDR(1) = 0.0;
	tDR(2) = R_r * sin(-phi_angle / Rad);
	longti = Gps_m(0);
	lati = Gps_m(1);
	hm0 = Gps_m(2);

	dTe << CoordinateTransM3(PI / 2, 2)*CoordinateTransM3(PI / 2 - longti / Rad, 1)*CoordinateTransM3(lati / Rad, 3);
	eTd = dTe.transpose();

	for (int i = 0;i < 10; i++)
	{
		Gps2E(Gps_m, Pe);
		/*Pe = Gps2E(Gps_m);*/
		tDR_pe = dTe*tDR;
		Pte = Pe + tDR_pe;
		E2Gps(Pte, Gps_t);

		Gps_t(2) = Ht0;
		Gps2E(Gps_t, Pte);
		//Pte = Gps2E(Gps_t);
		tDR_pe = Pte - Pe;
		tDR = eTd*tDR_pe;
		Rm_now = sqrt(tDR(0)*tDR(0) + tDR(1)*tDR(1) + tDR(2)*tDR(2));
		if (fabs(Rm_now - R_r) < 1.)
		{
			break;
		}
		tDR = tDR / Rm_now*R_r;
	}
	return Gps_t;
}

void SimTools::Sitecompute_func(Vector3d Gps_m, double phi_angle, double Ht0, double R_r, Vector3d & Gps_t)
{
	double longti, lati, hm0;
	Vector3d tDR, tDR_pe, Pe, Pte;
	Matrix3d dTe, eTd;
	double Rm_now;

	tDR(0) = R_r * cos(-phi_angle / Rad);
	tDR(1) = 0.0;
	tDR(2) = R_r * sin(-phi_angle / Rad);
	longti = Gps_m(0);
	lati = Gps_m(1);
	hm0 = Gps_m(2);

	dTe << CoordinateTransM3(PI / 2, 2)*CoordinateTransM3(PI / 2 - longti / Rad, 1)*CoordinateTransM3(lati / Rad, 3);
	eTd = dTe.transpose();

	for (int i = 0;i < 10; i++)
	{
		Gps2E(Gps_m, Pe);
		tDR_pe = dTe*tDR;
		Pte = Pe + tDR_pe;
		E2Gps(Pte, Gps_t);

		Gps_t(2) = Ht0;
		Gps2E(Gps_t, Pte);
		tDR_pe = Pte - Pe;
		tDR = eTd*tDR_pe;
		Rm_now = sqrt(tDR(0)*tDR(0) + tDR(1)*tDR(1) + tDR(2)*tDR(2));
		if (fabs(Rm_now - R_r) < 1.)
		{
			break;
		}
		tDR = tDR / Rm_now*R_r;
	}

}

double SimTools::PhiL_Compute(const Vector3d & GPS_A, const  Vector3d & GPS_B)
{
	double phi_l;
	double longti_A = GPS_A(0);double lati_A = GPS_A(1); double hm_A = GPS_A(2);
	double longti_B = GPS_B(0);double lati_B = GPS_B(1);double ht_B = GPS_B(2);
	double xd0, yd0, zd0;
	Matrix3d dTe_A, eTd_A;
	Vector3d Pm_eA, Pm_eB;
	Vector3d PB_dA; //BA在A点当地地理系下的投影
	double R;
	dTe_A << CoordinateTransM3(PI / 2, 2)*CoordinateTransM3(PI / 2 - longti_A / Rad, 1)*CoordinateTransM3(lati_A / Rad, 3);
	eTd_A = dTe_A.transpose();
	Gps2E(GPS_A, Pm_eA);
	Gps2E(GPS_B, Pm_eB);
	PB_dA = eTd_A * (Pm_eB - Pm_eA);
	R = BoundNorm2(PB_dA);
	phi_l = atan2(-PB_dA(2), PB_dA(0))*Rad;
	return phi_l;
}

double SimTools::GPS_R_Compute(const Vector3d & GPS_A, const Vector3d & GPS_B)
{
	
	double longti_A = GPS_A(0); double lati_A = GPS_A(1); double hm_A = GPS_A(2);
	double longti_B = GPS_B(0); double lati_B = GPS_B(1); double ht_B = GPS_B(2);
	double xd0, yd0, zd0;
	Matrix3d dTe_A, eTd_A;
	Vector3d Pm_eA, Pm_eB;
	Vector3d PB_dA; //BA在A点当地地理系下的投影
	double R;
	dTe_A << CoordinateTransM3(PI / 2, 2)*CoordinateTransM3(PI / 2 - longti_A / Rad, 1)*CoordinateTransM3(lati_A / Rad, 3);
	eTd_A = dTe_A.transpose();
	Gps2E(GPS_A, Pm_eA);
	Gps2E(GPS_B, Pm_eB);
	PB_dA = eTd_A * (Pm_eB - Pm_eA);
	//R = BoundNorm2(PB_dA);
	R = BoundNorm2(Pm_eB - Pm_eA);
	return R;
}

bool SimTools::Ifpointraingle(double pot[2], NODE_xy A, NODE_xy B, NODE_xy C)
{
	float signOfAB = (B.x - A.x) * (pot[1] - A.y) - (B.y - A.y) * (pot[0] - A.x);
	float signOfCA = (A.x - C.x) * (pot[1] - C.y) - (A.y - C.y) * (pot[0] - C.x);
	float signOfBC = (C.x - B.x) * (pot[1] - C.y) - (C.y - B.y) * (pot[0] - C.x);

	// 若同号，则点在边的同一侧，若等于零则点在边上
	if (signOfAB >= 0 && signOfBC >= 0 && signOfCA >= 0)
	{
		return true;
	}
	if (signOfAB <= 0 && signOfBC <= 0 && signOfCA <= 0)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

double SimTools::Rand_01()
{
	return rand() / (RAND_MAX*1.0);	//归一化
}

double SimTools::Rand_ab(double a, double b)
{
	double tmp = Rand_01();
	return a + tmp*(b - a);				//输入参数要求b>a
}

double SimTools::Rand_N01()
{
	double tmp = 0.0;
	for (int i = 0;i < 12;i++)
	{
		tmp = tmp + Rand_01();
	}
	return tmp - 6.0;
}

double SimTools::Rand_N(double mu, double sigma2)
{
	return Rand_N01()*sqrt(sigma2) + mu;
}

double** SimTools::Input_data(char * p, int k)
{
	char c;
	int n = 0;
	double **num;
	std::ifstream file(p);
	std::ifstream fin(p);
	while (fin.get(c))
	{
		if (c == '\n')
			n++;
	}
	num = (double **)malloc(n*k*sizeof(double *));
	for (int i = 0;i < n*k;i++)
		num[i] = (double *)malloc(2 * sizeof(double *));

	for (int i = 0;i < n;i++)
	{
		//if(i>400)
		//{		
		//	double mm;
		//	mm=1;
		//}
		for (int j = 0;j < k;j++)
			file >> num[i][j];

		double tttt;
		tttt = 1;
	}


	file.close();
	fin.close();
	return num;
}

int SimTools::TxtRowcount(std::string txtname)
{
	int Rownum = 0;
	std::ifstream data_mtnum;
	data_mtnum.open(txtname);
	if (!data_mtnum)
	{
		std::cout << std::flush << "\\n\\aERROR: Unable to open file: "<<"\\n\\a";
		std::cout << std::flush << "\\n\\tPress any key to exit...";
		std::cin.get();
		exit(0);
	}
	std::string tmp;
	while (getline(data_mtnum, tmp, '\n'))
	{
		Rownum++;
	}
	data_mtnum.close();
	return Rownum;
}

int SimTools::TxtRowcount(char * p)
{
	int n = 0;
	char c;
	std::ifstream file(p);
	std::ifstream fin(p);
	while (fin.get(c))
	{
		if (c == '\n')
			n++;
	}
	file.close();
	fin.close();
	return n;
}

int SimTools::Index1(double t, std::vector<double> idx_t)
{
	unsigned int i = 0;
	unsigned int rowMax = (unsigned int)idx_t.size();

	while (i < rowMax)
	{
		if (t < idx_t[i])	return i;//k-1, k
		else			i++;
	}

	if (i == rowMax)	i = rowMax - 1;

	return i;
}

int SimTools::Index2(double i, std::vector<std::vector<double>> idx_t)
{
	int rowmax = idx_t.size();

	unsigned int n = 0;
	unsigned int rowMax = (unsigned int)idx_t.size();

	while (n < rowMax)
	{
		if (i <= idx_t[n][0])
			return n;//k-1, k
		else
			n++;
	}

	if (n == rowMax)	n = rowMax - 1;

	return n;
}

std::string SimTools::NumberToString(int x)
{
	std::stringstream ss;
	ss << x;
	return ss.str();
}

Matrix3d SimTools::CoordinateTransM3(double angle, int axis) { //输入为弧度
															   //坐标转换矩阵（左乘，基于右手定则） axis=1,2,3分别对应X Y Z轴 逆时针旋转角度 angle(弧度)
	Matrix3d M_tran;
	if (axis == 1)
	{
		M_tran << 1, 0, 0,
			0, cos(angle), sin(angle),
			0, -sin(angle), cos(angle);
	}
	else if (axis == 2)
	{
		M_tran << cos(angle), 0, -sin(angle),
			0, 1, 0,
			sin(angle), 0, cos(angle);
	}
	else if (axis == 3)
	{
		M_tran << cos(angle), sin(angle), 0,
			-sin(angle), cos(angle), 0,
			0, 0, 1;
	}
	else
	{
		std::cout << "转换矩阵函数输入参数有误" << std::endl;
	}
	return M_tran;

}

void SimTools::VnFromV(double theta, double phi_v, double V,Vector3d &VN)
{
	VN(0) = V*cos(theta)*cos(phi_v);
	VN(1) = V*sin(theta);
	VN(2) = -V*cos(theta)*sin(phi_v);
}

void  SimTools::VeFromV(double V, const Vector3d & GPS, Vector3d & VE)
{
	double slog, clog, slat, clat, rtod;
	rtod = 180.0f / 3.14159265f;
	slog = sin(GPS[0] / rtod);
	clog = cos(GPS[0] / rtod);
	slat = sin(GPS[1] / rtod);
	clat = cos(GPS[1] / rtod);

	VE[0] = -slat*clog*V;
	VE[1] = -slat*slog*V;
	VE[2] = clat*V;

}

Matrix3d SimTools::MakeE2NFromGps(double longti, double lati)
{
	Matrix3d temp;
	temp<< CoordinateTransM3(PI / 2, 2)*CoordinateTransM3(PI / 2 - longti / Rad, 1)*CoordinateTransM3(lati / Rad, 3);
	return temp.transpose();
}

Matrix3d SimTools::MakeN2EFromGps(double longti, double lati)
{
	Matrix3d temp;
	temp<< CoordinateTransM3(PI / 2, 2)*CoordinateTransM3(PI / 2 - longti / Rad, 1)*CoordinateTransM3(lati / Rad, 3);
	return temp;
}

void SimTools::PnFromPe(const Vector3d & Pxyz_e, Vector3d & Pxyz_n, double longti, double lati)
{
	Matrix3d E2N = MakeE2NFromGps(longti, lati);
	Pxyz_n = E2N*Pxyz_e;
}

void SimTools::MakeGPSFromPn(const Vector3d &Gps_O, double Pz_n, double Px_n, double & longti_t, double &lati_t,double Ht0)
{
	double longti, lati, hm0;
	Vector3d tDR, tDR_pe, Pe, Pte;
	Vector3d Gps_t;
	Matrix3d dTe, eTd;
	double Rm_now;
	tDR(0) = Px_n;
	tDR(1) = 0.;
	tDR(2) = Pz_n;
	longti = Gps_O(0);
	lati = Gps_O(1);
	hm0 = Gps_O(2);
	double R_r = sqrt(Px_n*Px_n + Pz_n*Pz_n);

	dTe << CoordinateTransM3(PI / 2, 2)*CoordinateTransM3(PI / 2 - longti / Rad, 1)*CoordinateTransM3(lati / Rad, 3);
	eTd = dTe.transpose();

	for (int i = 0;i < 10; i++)
	{
		Gps2E(Gps_O, Pe);
		/*Pe = Gps2E(Gps_m);*/
		tDR_pe = dTe*tDR;
		Pte = Pe + tDR_pe;
		E2Gps(Pte, Gps_t);

		Gps_t(2) = Ht0;
		Gps2E(Gps_t, Pte);
		//Pte = Gps2E(Gps_t);
		tDR_pe = Pte - Pe;
		tDR = eTd*tDR_pe;
		Rm_now = sqrt(tDR(0)*tDR(0) + tDR(1)*tDR(1) + tDR(2)*tDR(2));
		if (fabs(Rm_now - R_r) < 1.)
		{
			break;
		}
		tDR = tDR / Rm_now*R_r;
	}
	longti_t = Gps_t(0);
	lati_t = Gps_t(1);

}

void SimTools::MakeXYZFromGPS0(const Vector3d& Gps0, Vector3d Gps, Vector3d& Pxyz)
{
	Vector3d gps0 = { Gps0[0], Gps0[1], 0 }; //初始发射点当地地理系原点
	Vector3d Pe0;
	Vector3d Pe, Pn;
	Gps2E(gps0, Pe0);
	SimTools::Gps2E(gps0, Pe0);		//求解原点的地心坐标
	Matrix3d E2N0 = MakeE2NFromGps(gps0[0], gps0[1]);      //地心系到当地地理系的转化矩阵
	Gps2E(Gps, Pe); //求出每个经纬度对应的地心坐标
	Pxyz = E2N0 * (Pe - Pe0);  //当地地理系坐标
}

void SimTools::Gps2E(const Vector3d &GPS, Vector3d &PE)
{
	double longti_x = GPS(0);
	double lati_x = GPS(1);
	double hh = GPS(2);
	double sin_lati = sin(lati_x / Rad);
	double cos_lati = cos(lati_x / Rad);
	double sin_longti = sin(longti_x / Rad);
	double cos_longti = cos(longti_x / Rad);

	PE(0) = (Re*(1 + E_earth*sin_lati*sin_lati) + hh)*cos_lati*cos_longti;
	PE(1) = (Re*(1 + E_earth*sin_lati*sin_lati) + hh)*cos_lati*sin_longti;
	PE(2) = (Re*(1 + E_earth*sin_lati*sin_lati)*(1 - E_earth)*(1 - E_earth) + hh)*sin_lati;

}

void SimTools::E2Gps(const Vector3d &PE, Vector3d &GPS)
{
	double  pex, pey, pez;
	double hm;
	pex = PE(0);  pey = PE(1);pez = PE(2);

	double pexy = sqrt(pex*pex + pey*pey);
	double log, lat, h0, Rnm, kh, clat, slat;

	if (pex >= 0.0)
	{

		log = atan(pey / pex);
	}
	else
	{
		if (pey >= 0.0)
		{
			log = atan(pey / pex) + PI;
		}
		else
		{
			log = atan(pey / pex) - PI;
		}
	}
	lat = atan((1 / ((1 - E_earth)*(1 - E_earth))) * (pez / pexy));
	if (E_earth > 0)
	{
		clat = cos(lat);
		slat = sin(lat);
		Rnm = Re * (1 + E_earth * slat * slat);
		h0 = pex / (cos(log)*clat) - Rnm;
		kh = 0.006655 * (1.0 - clat*clat) + 1.0;
		h0 = h0 / kh;
		lat = lat / (1 + clat * 0.0000009) - h0 / 2000000000.0;
	}
	else
	{
		clat = cos(lat);
		slat = sin(lat);
		Rnm = Re * (1 + E_earth * slat * slat);
		h0 = pex / (cos(log)*clat) - Rnm;
	}

	log = log * Rad;
	lat = lat * Rad;
	hm = h0;
	GPS << log, lat, hm;
}

void SimTools::E2GPS(double pe[3], double gps[3])
{
	double Re = 6378137.0;
	double f = 1.0 / 298.257223563;
	double pit = 3.14159265;
	double x = pe[0];
	double y = pe[1];
	double z = pe[2];

	double lat = std::atan2(z, std::sqrt(x * x + y * y));
	double sinLat, cosLat;
	double N;
	double h;
	double prevLat = lat;
	//迭代计算纬度和高度
	do {
		prevLat = lat;
		sinLat = std::sin(lat);
		cosLat = std::cos(lat);
		//计算卯酉圈曲率半径
		N = Re / std::sqrt(1 - (2 * f - f * f) * sinLat * sinLat);
		//计算高度
		h = std::sqrt(x * x + y * y) / cosLat - N;
		//更新纬度
		lat = std::atan2(z, std::sqrt(x * x + y * y) * (1 - (2 * f - f * f) * N / (N + h)));
	} while (std::abs(lat - prevLat) > 1e-8);
		//计算经度
	double lon = std::atan2(y, x);
	//弧度转化
	gps[0] = lon * 180. / pit;
	gps[1] = lat * 180. / pit;
	gps[2] = h;
}


Vector3d SimTools::E2Gps_Newton(Vector3d E)
{
	double xe = E(0);
	double ye = E(1);
	double ze = E(2);
	double longti_x = 30.;
	double lati_x = 30.;
	double hh = 0.;
	//*****牛顿法解非线性方程组*****//
	int N = 1000;//最大迭代次数
	int count = 0;//计数器
	double tol = 0.0001;//迭代精度
	double sig;//误差模
	Matrix3d J;//雅克比矩阵
	Vector3d F;//构造函数值向量
	Vector3d P;//经纬高坐标向量
	Vector3d PP;//迭代中间值
	P << longti_x, lati_x, hh;//初始值
	do
	{
		double sin_lati = sin(lati_x / Rad);
		double cos_lati = cos(lati_x / Rad);
		double sin_longti = sin(longti_x / Rad);
		double cos_longti = cos(longti_x / Rad);
		//计算雅克比矩阵
		J(0, 0) = -(Re*(1 + E_earth*sin_lati*sin_lati) + hh)*cos_lati*sin_longti;
		J(1, 0) = (Re*(1 + E_earth*sin_lati*sin_lati) + hh)*cos_lati*cos_longti;
		J(2, 0) = 0;
		J(0, 1) = E_earth*Re*sin(2 * lati_x / Rad)*cos_lati*cos_longti - (Re*(1 + E_earth*sin_lati*sin_lati) + hh)*cos_longti*sin_lati;
		J(1, 1) = E_earth*Re*sin(2 * lati_x / Rad)*cos_lati*sin_longti - (Re*(1 + E_earth*sin_lati*sin_lati) + hh)*sin_longti*sin_lati;
		J(2, 1) = E_earth*(1 - E_earth)*(1 - E_earth)*Re*sin(2 * lati_x / Rad)*sin_lati + (Re*(1 + E_earth*sin_lati*sin_lati)*(1 - E_earth)*(1 - E_earth) + hh)*cos_lati;
		J(0, 2) = cos_lati*cos_longti;
		J(1, 2) = cos_lati*sin_longti;
		J(2, 2) = sin_lati;
		//计算构造函数值
		F(0) = (Re*(1 + E_earth*sin_lati*sin_lati) + hh)*cos_lati*cos_longti - xe;
		F(1) = (Re*(1 + E_earth*sin_lati*sin_lati) + hh)*cos_lati*sin_longti - ye;
		F(2) = (Re*(1 + E_earth*sin_lati*sin_lati)*(1 - E_earth)*(1 - E_earth) + hh)*sin_lati - ze;

		PP = P + J.inverse()*-F;//循环迭代
								//更新经纬高信息
		sig = sqrt((PP(0) - P(0))*(PP(0) - P(0)) + (PP(1) - P(1))*(PP(1) - P(1)) + (PP(2) - P(2))*(PP(2) - P(2))) / sqrt(P(0)*P(0) + P(1)*P(1) + P(2)*P(2));

		//更新经纬高信息
		P = PP;
		longti_x = P(0);
		lati_x = P(1);
		hh = P(2);
		count += 1;
	} while (count<N || sig>tol);
	return P;
}

