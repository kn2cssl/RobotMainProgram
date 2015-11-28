/*
* controller.c
*
* Created: 8/14/2015 12:19:25 PM
*  Author: QWA
*/

#include "controller.h"

double Vx , Vy , Wr ;

double x[7][1] , dx[7][1] ,xd[7][1] , du[4][1] , ud[4][1] , u[4][1] ,x_OB[7][1] , x_temp_1[7][1] , x_temp_2[7][1] ,x_temp_3[7][1] ;

//Xdot = A*X + B*U
//Y = C*X + D*U

double Yd[7] ;// Y desired


double max_ocr = 16383 ;


double A [7][7] =	{	{   -5.3266  , 3.7199e-16  ,-4.1663e-17  ,  -0.005547  ,  -0.004714   ,  0.004714  ,   0.005547 },
{ 3.7199e-16 ,     -3.6084 ,    0.057194 ,    0.003698 ,   -0.004714  ,  -0.004714 ,    0.003698},
{-3.255e-15  ,     4.4683  ,    -4.9254  ,    0.04375  ,    0.04375   ,   0.04375  ,    0.04375 },
{   -506.5   ,    337.67   ,    51.134   ,   -7.6844   ,         0    ,        0   ,         0  },
{   -430.44  ,    -430.44  ,     51.134  ,          0  ,    -7.6844   ,         0  ,          0 },
{   430.44   ,   -430.44   ,    51.134   ,         0   ,         0    ,  -7.6844   ,         0  },
{   506.5    ,   337.67    ,   51.134    ,        0    ,        0     ,       0    ,  -7.6844   }};
							   

double B [7][4] =	{	{	0			,0			,0			,0			}	,
{	0			,0			,0			,0			}	,
{	0			,0			,0			,0			}	,
{	514.7574	,0			,0			,0			}	,
{	0			,514.7574	,0			,0			}	,
{	0			,0			,514.7574	,0			}	,
{	0			,0			,0			,514.7574	}	}	;


//-inv(B'*B)*B'*A	= uFx
double uFx[4][7] =	
{{ 0.98396   ,  -0.65597   , -0.099336   ,  0.014928    ,        0    ,        0   ,         0  },
{ 0.83621   ,   0.83621   , -0.099336   ,         0    , 0.014928    ,        0   ,         0	},
{ -0.83621  ,    0.83621  ,  -0.099336  ,          0   ,         0   ,  0.014928  ,          0 },
{ -0.98396  ,   -0.65597  ,  -0.099336  ,          0   ,         0   ,         0  ,   0.014928	}};	
//! discretized
// double uFx[4][7] =
//    {{  0.97826  ,  -0.65388  ,  -0.098844 ,    0.014915 ,-6.4163e-06  , 3.6846e-06 , -1.4417e-06  },
//    {  0.83136  ,   0.83244  ,  -0.098751 , -6.4163e-06 ,   0.014915  ,-4.7394e-06 ,  3.6846e-06	 },
//    {  -0.83136 ,    0.83244 ,   -0.098751,   3.6846e-06, -4.7394e-06 ,    0.014915,  -6.4163e-06 },
//    {  -0.97826 ,   -0.65388 ,   -0.098844,  -1.4417e-06,  3.6846e-06 , -6.4163e-06,     0.014915 }};             


//k:state feed back
double k_sf[4][7] =   {{ -0.90533   ,    1.0294 ,    0.094414 ,   0.0058454 ,  0.00033879,  -0.00047563,  0.00028039},
{ -0.76938   ,   -1.0488 ,    0.071235 ,  0.00033879 ,   0.0057237,   0.00042486, -0.00047563},
{ 0.76938    ,  -1.0488  ,   0.071235  ,-0.00047563  , 0.00042486 ,   0.0057237 , 0.00033879	},
{ 0.90533    ,   1.0294  ,   0.094414  , 0.00028039  ,-0.00047563 ,  0.00033879 ,  0.0058454	}};

	
double G[7][7]	=  	{{   9.925    ,  1.4835e-15 ,  2.5183e-16 ,   -0.005482 ,  -0.0046588,    0.0046588,     0.005482},
{1.4835e-15  ,     11.037  ,    0.88378  ,  0.0041826  , -0.0051815 ,  -0.0051815 ,   0.0041826 },
{5.0367e-16  ,     1.7676  ,       6.73  ,  0.0021147  ,-0.00055901 , -0.00055901 ,   0.0021147 },
{   -219.28  ,     167.31  ,     42.295  ,    0.40771  ,   0.066417 ,    -0.38761 ,    -0.12654 },
{   -186.35  ,    -207.26  ,     -11.18  ,   0.066417  ,    0.40022 ,    0.014367 ,    -0.38761 },
{   186.35   ,   -207.26   ,    -11.18   ,  -0.38761   ,  0.014367  ,    0.40022  ,   0.066417	 },
{   219.28   ,    167.31   ,    42.295   ,  -0.12654   ,  -0.38761  ,   0.066417  ,    0.40771	 }};
	
float d_time;
	


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
	xd[3][0] = (-Vx*sina1+Vy*cosa1+Wr*d)*b ;
	xd[4][0] = (-Vx*sina2+Vy*cosa2+Wr*d)*b ;
	xd[5][0] = (-Vx*sina3+Vy*cosa3+Wr*d)*b ;
	xd[6][0] = (-Vx*sina4+Vy*cosa4+Wr*d)*b ;
	
	// data checking 1 : data is produced correctly (checked with model in MATLAB)
}

void state_generator ( void )
{
	//! Transferring vectors from camera's coordinate system to Robot coordinate system
	float cos_alpha = cos(Robot.alpha.full/1000.0);
	float sin_alpha = sin(Robot.alpha.full/1000.0);
	Vx = (  Robot.Vx.full * cos_alpha + Robot.Vy.full * sin_alpha ) / 1000.0;
	Vy = ( -Robot.Vx.full * sin_alpha + Robot.Vy.full * cos_alpha ) / 1000.0;
	Wr = Robot.Wr.full / 1000.0;
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
			
			dx [i][j] = xd [i][j] - x_OB [i][j] ;  // minus is here <<<<<=

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
			
			//emitting saturation
			if (fabs(u[i][j]) > Robot.bat_v.full)
			{
				u[i][j] = sign(u[i][j]) * Robot.bat_v.full ;
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
// 			for (int j = 0 ; j < 1 ; j ++)
// 			{
			int j = 0;
				x_temp_1 [i][j] = (x [i][j] - x_OB [i][j]) ;
/*			}*/
		}
		//xhp+=G*(y-Ch*xh)
		for (int i=0 ; i < 7 ; i ++)
		{
// 			for (int j = 0 ; j < 1 ; j ++)
// 			{
				int j = 0;
				x_temp_2 [i][j] = 0;
				for (int k = 0 ; k < 7 ; k ++)
				{
					x_temp_2 [i][j] += G [i][k] * x_temp_1 [k][j]*d_time;
				}
/*			}*/
		}
	
	//xhp+=Ah*xh
	for (int i=0 ; i < 7 ; i ++)
	{
// 		for (int j = 0 ; j < 1 ; j ++)
// 		{
			int j = 0;
			x_temp_3 [i][j] = 0;
			for (int k = 0 ; k < 7 ; k ++)
			{
				x_temp_3 [i][j] += A [i][k] * x_OB [k][j]*d_time;
			}
/*		}*/
	}
	
	//xhp+=Bh*uh
	for (int i=0 ; i < 7 ; i ++)
	{
// 		for (int j = 0 ; j < 1 ; j ++)
// 		{
			int j = 0;
			for (int k = 0 ; k < 4 ; k ++)
			{
				x_OB [i][j] += B [i][k] * ud [k][j]*d_time;
			}
			x_OB [i][j] += x_temp_2[i][j] + x_temp_3[i][j];
/*		}*/
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
	Robot.orc_length=127;
	max_ocr = (Robot.orc_length << 8) - 1 ;
}





