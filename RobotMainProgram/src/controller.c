/*
* controller.c
*
* Created: 8/14/2015 12:19:25 PM
*  Author: QWA
*/

#include "controller.h"

double Vx , Vy , Wr ;

double x[7][1] , dx[7][1] ,xd[7][1] , du[4][1] , ud[4][1] , u[4][1], uobs[4][1], x_OB[7][1] , x_temp_1[7][1] , x_temp_2[7][1] ,x_temp_3[7][1] ;

double camera_d[3][1];

//Xdot = A*X + B*U
//Y = C*X + D*U

double Yd[7] ;// Y desired

double max_ocr = 16383 ;

double A [7][7] =
{{    -4.2613,   2.976e-16, -3.3331e-17,   0.0044376,   0.0037712,  -0.0037712,  -0.0044376},
 {  2.976e-16,     -2.8867,    0.045755,  -0.0029584,   0.0037712,   0.0037712,  -0.0029584},
 {  -3.58e-14,      49.145,     -54.173,    -0.48119,    -0.48119,    -0.48119,    -0.48119},
 {     442.74,     -295.16,     -44.697,     -7.9993,           0,           0,           0},
 {     376.26,      376.26,     -44.697,           0,     -7.9993,           0,           0},
 {    -376.26,      376.26,     -44.697,           0,           0,     -7.9993,           0},
 {    -442.74,     -295.16,     -44.697,           0,           0,           0,     -7.9993}};
	 	  
double B [7][4] =
{{      0,           0,           0,           0},
 {      0,           0,           0,           0},
 {      0,           0,           0,           0},
 { 562.44,           0,           0,           0},
 {      0,      562.44,           0,           0},
 {      0,           0,      562.44,           0},
 {      0,           0,           0,      562.44}};

// -inv(B'*B)*B'*A	= uFx
double uFx[4][7] =
{{-0.78717,     0.52478,    0.079469,    0.014222,           0,           0,           0},
 {-0.66897,    -0.66897,    0.079469,           0,    0.014222,           0,           0},
 { 0.66897,    -0.66897,    0.079469,           0,           0,    0.014222,           0},
 { 0.78717,     0.52478,    0.079469,           0,           0,           0,    0.014222}};

//LQR
double k_sf[4][7] =		
{{  0.34349,    -0.37807,  -0.0028347,   0.0033964,  0.00012475,  -0.0001388,   9.768e-05},
 {  0.29191,     0.38459,   -0.002202,  0.00012475,   0.0033561,  0.00014351,  -0.0001388},
 { -0.29191,     0.38459,   -0.002202,  -0.0001388,  0.00014351,   0.0033561,  0.00012475},
 { -0.34349,    -0.37807,  -0.0028347,   9.768e-05,  -0.0001388,  0.00012475,   0.0033964}};
	 
double G[7][7]	=
{{      2.0159, -3.6621e-18,  7.9508e-17,   0.0011207,  0.00095243, -0.00095243,  -0.0011207},
 { -3.6621e-18,      2.5065,    0.072809, -0.00094346,    0.001107,    0.001107, -0.00094346},
 {  1.9877e-15,      1.8202,      1.4314,   -0.012509,   -0.010969,   -0.010969,   -0.012509},
 {      93.393,     -78.621,     -41.696,      1.5376,    0.039508,   -0.039788,    0.010215},
 {      79.369,      92.254,     -36.562,    0.039508,      1.5367,    0.035259,   -0.039788},
 {     -79.369,      92.254,     -36.562,   -0.039788,    0.035259,      1.5367,    0.039508},
 {     -93.393,     -78.621,     -41.696,    0.010215,   -0.039788,    0.039508,      1.5376}};
float cycle_time_s, cycle_time_us, run_time_us;



void setpoint_generator ( void )
{
	//! Transferring vectors from camera's coordinate system to Robot coordinate system
	float cos_alpha = cos(Robot.alpha.full/1000.0);
	float sin_alpha = sin(Robot.alpha.full/1000.0);
	Vx = (  Robot.Vx_sp.full * cos_alpha + Robot.Vy_sp.full * sin_alpha ) / 1000.0;
	Vy = ( -Robot.Vx_sp.full * sin_alpha + Robot.Vy_sp.full * cos_alpha ) / 1000.0;
	Wr = Robot.Wr_sp.full / 1000.0;
	// 7 set points for system
	//kinematics rules that should be considered for specifying desired output
	
	// 	xd=(C'*C)\C'*Yd;%inv(C'*C)*C'*Yd; & C = I7x7		=>		xd = Yd
	xd[0][0] = Vx ;
	xd[1][0] = Vy ;
	xd[2][0] = Wr ;
	xd[3][0] = (Vx*sina1-Vy*cosa1-Wr*d)*b ;
	xd[4][0] = (Vx*sina2-Vy*cosa2-Wr*d)*b ;
	xd[5][0] = (Vx*sina3-Vy*cosa3-Wr*d)*b ;
	xd[6][0] = (Vx*sina4-Vy*cosa4-Wr*d)*b ;
	
	// data checking 1 : data is produced correctly (checked with model in MATLAB)
}

void state_generator ( void )
{
	//! Transferring vectors from camera's coordinate system to Robot coordinate system
	float cos_alpha = cos(Robot.alpha.full/1000.0);
	float sin_alpha = sin(Robot.alpha.full/1000.0);
	Vx = (  Robot.Vx.full * cos_alpha + Robot.Vy.full * sin_alpha ) / 1000.0;
	Vy = ( -Robot.Vx.full * sin_alpha + Robot.Vy.full * cos_alpha ) / 1000.0;
	Wr = Robot.Wr.full / 100.0;
	// 7 set points for system
	//kinematics rules that should be considered for specifying desired output
	
	// 	xd=(C'*C)\C'*Yd;%inv(C'*C)*C'*Yd; & C = I7x7		=>		xd = Yd
	x[0][0] = Vx ;
	x[1][0] = Vy ;
	x[2][0] = Wr ;
	x[3][0] = Robot.W0.full ;
	x[4][0] = Robot.W1.full ;
	x[5][0] = Robot.W2.full ;
	x[6][0] = Robot.W3.full ;
	
	// data checking 1 : data is produced correctly (checked with model in MATLAB)
}

void camera_data ( void )
{
	float cos_alpha = cos(Robot.alpha.full/1000.0);
	float sin_alpha = sin(Robot.alpha.full/1000.0);
	camera_d[0][0] = (x_OB[0][0]*1000 / sin_alpha - x_OB[1][0]*1000/ cos_alpha ) / (cos_alpha / sin_alpha  + sin_alpha/ cos_alpha )  ;
	
}

void state_feed_back ( void )
{
	// 	ud=-inv(B'*B)*B'*A*xd;
	
	for (int i=0 ; i < 4 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			ud [i][j] = 0 ;
			for (int k = 0 ; k < 7 ; k ++)
			{
				ud [i][j] += uFx [i][k] * xd [k][j];
			}
		}
	}
	
	// data checking 2 : data is produced correctly (checked with model in MATLAB)

	// 	du=-K*(x-xd);%for simulating controller with unnoisy data
	// 	%du=-K*(xl-xd);%for simulating controller and observer:(x-xd)|(xh-xd)|(xl-xd)
	
	for (int i=0 ; i < 7 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{			
			dx [i][j] = xd [i][j] - x_OB [i][j] ;  // minus is here <<<<<= // x_OB <<<
		}
	}
	
	
	for (int i=0 ; i < 4 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			du [i][j] = 0 ;
			for (int k = 0 ; k < 7 ; k ++)
			{
				du [i][j] += k_sf [i][k] * dx [k][j];
			}
		}
	}
	
	// 	u=ud+du;
	for (int i=0 ; i < 4 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			u [i][j] = (du [i][j] + ud [i][j]);
			uobs [i][j] = u [i][j];
			// 2.9 v is gained with experiment : it should be applied to confront resistive forces
			u [i][j] += sign(u [i][j]) * 2.9 ;
			
			//emitting saturation
			if (fabs(u[i][j]) > Robot.bat_v.full)
			{
				u[i][j] = sign(u[i][j]) * (Robot.bat_v.full *.997) ;// 0.997 since in transfer function of volt to OCR 100% occurs in 99.7% of full voltage
			}
		}
	}
	
}

void observer ( void )
{
	//xhp=Ah*xh+Bh*uh+G*(y-Ch*xh);
	
	//(y-Ch*xh)
	for (int i=0 ; i < 7 ; i ++)
	{
		x_temp_1 [i][0] = (x [i][0] - x_OB [i][0]) ;
	}
	//xhp+=G*(y-Ch*xh)
	for (int i=0 ; i < 7 ; i ++)
	{
		x_temp_2 [i][0] = 0;
		for (int k = 0 ; k < 7 ; k ++)
		{
			x_temp_2 [i][0] += G [i][k] * x_temp_1 [k][0]*cycle_time_s;
		}
	}
	
	//xhp+=Ah*xh
	for (int i=0 ; i < 7 ; i ++)
	{
		x_temp_3 [i][0] = 0;
		for (int k = 0 ; k < 7 ; k ++)
		{
			x_temp_3 [i][0] += A [i][k]* x_OB [k][0]*cycle_time_s*0;
		}
	}
	
	//xhp+=Bh*uh
	for (int i=0 ; i < 7 ; i ++)
	{
		for (int k = 0 ; k < 4 ; k ++)
		{
			x_OB [i][0] += B [i][k] * uobs [k][0]*cycle_time_s*0;
		}
		x_OB [i][0] += x_temp_2[i][0] + x_temp_3[i][0];

	}

}



//void RLS (void)
//{
//
//p=p/landa-(p*(xk'*xk)*p)/(landa+xk*p*xk')/landa
//q=q+xk'*yk;
////         %x1
////         %x2
////         %x3
////         %x4
////         %x5
////    q=q+[%x6]*[y1 y2 y3 y4 y5 y6 y7 y8 y9 y10 y11]
////         %x7
////         %x8
////         %x9
////         %x10
////         %x11
//
//theta(:,:)=p*q;
//}


double sign (double number)
{
	if (number > 0)
	{
		return 1 ;
	}
	else if (number < 0)
	{
		return -1 ;
	}
	else
	{
		return 0 ;
	}
}

inline void ocr_change(void)
{
	// TODO correct it
	Robot.orc_length=127;
	max_ocr = (Robot.orc_length << 8) - 1 ;
}