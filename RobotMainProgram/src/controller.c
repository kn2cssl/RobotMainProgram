/*
* controller.c
*
* Created: 8/14/2015 12:19:25 PM
*  Author: QWA
*/

#include "controller.h"

double Vx , Vy , Wr ;

double x[7][1] , dx[7][1] ,xd[7][1] , du[4][1] , ud[4][1] , u[4][1] ,x_OB[7][1] , x_temp[7][1] ;

//Xdot = A*X + B*U
//Y = C*X + D*U

double Yd[7] ;// Y desired


double max_ocr = 16383 ;


double A [7][7] =	{	{	-53.2664	,3.7199		,0			,-0.0554	,-0.0471	,0.0471		,0.0554		}	,
{	3.7199		,-36.0836	,0.6976		,0.0369		,-0.0471	,-0.0471	,0.0369		}	,
{	0			,54.5001	,-185.4827	,0.4110		,0.4375		,0.4375		,0.4110		}	,
{	-5065.0228	,3376.6732	,480.4730   ,-24.0355	,0			,0			,0			}	,
{	-4304.4383	,-4304.4383	,511.3411	,0          ,-24.0355	,0			,0			}	,
{	4304.4383	,-4304.4383 ,511.3411	,0			,0          ,-24.0355	,0			}	,
{	5065.0228	,3376.6732  ,480.4730	,0			,0			,0			,-24.0355	}	};

double B [7][4] =	{	{	0			,0			,0			,0			}	,
{	0			,0			,0			,0			}	,
{	0			,0			,0			,0			}	,
{	514.7574	,0			,0			,0			}	,
{	0			,514.7574	,0			,0			}	,
{	0			,0			,514.7574	,0			}	,
{	0			,0			,0			,514.7574	}	}	;

double C [7][7] =	{	{	1     ,0     ,0     ,0     ,0     ,0     ,0		}	,
{	0     ,1     ,0     ,0     ,0     ,0     ,0		}	,
{	0     ,0     ,1     ,0     ,0     ,0     ,0		}	,
{	0     ,0     ,0     ,1     ,0     ,0     ,0		}	,
{	0     ,0     ,0     ,0     ,1     ,0     ,0		}	,
{	0     ,0     ,0     ,0     ,0     ,1     ,0		}	,
{	0     ,0     ,0     ,0     ,0     ,0     ,1		}	};

 //-inv(B'*B)*B'*A	= uFx
double uFx[4][7] =	{	{ 0.98396   ,  -0.65597   , -0.099336   ,  0.014928    ,        0    ,        0   ,         0  },
{ 0.83621   ,   0.83621   , -0.099336   ,         0    , 0.014928    ,        0   ,         0	},
{ -0.83621  ,    0.83621  ,  -0.099336  ,          0   ,         0   ,  0.014928  ,          0 },
{ -0.98396  ,   -0.65597  ,  -0.099336  ,          0   ,         0   ,         0  ,   0.014928	}};	
	                    


//k:state feed back
double k_sf[4][7] =   {{-1.5261 ,     2.7119   ,    3.2208 ,   0.014061 ,   0.0076554 ,  0.0063174 ,   0.0078795},
{-1.2969 ,   -0.85194   ,    3.1084 ,  0.0076554 ,    0.013257 ,   0.007513 ,   0.0063174},
{1.2969  ,  -0.85194    ,   3.1084  , 0.0063174  ,   0.007513  ,  0.013257  ,  0.0076554 },
{1.5261  ,    2.7119    ,   3.2208  , 0.0078795  ,  0.0063174  , 0.0076554  ,   0.014061 }};
 // double G[7][7]	= {{157.15     , -2.9017e-14 , -5.6315e-13 , -0.00094799 , -0.00080563,   0.00080563,   0.00094799},
                   // {-2.9017e-14,      161.58 ,      282.68 ,  0.00062002 , -0.00078912,  -0.00078912,   0.00062002},
                   // {-2.2526e-14,      11.307 ,      926.91 ,  3.7007e-05 , -3.6456e-05,  -3.6456e-05,   3.7007e-05},
                   // {-3792      ,  2480.1     ,  3700.7     , 0.17665     ,  0.038248  ,   -0.17233  ,  -0.071139	 },
                   // {-3222.5    , -3156.5     , -3645.6     ,  0.038248   ,   0.17495  , -0.0040029  ,   -0.17233	 },
                   // {3222.5     ,-3156.5      , -3645.6     ,  -0.17233   , -0.0040029 ,     0.17495 ,    0.038248 },
                     // {3792       ,  2480.1     ,  3700.7     , -0.071139   ,  -0.17233  ,   0.038248  ,    0.17665	 }};
	
double G[7][7]	=  	{{	3.1137  ,-5.787e-17  ,-1.0079e-14   ,  -16.106   , -13.687  ,  13.687  ,  16.106   	},
{-5.787e-17 ,     3.4629 ,      2.0391  ,    13.213  ,  -16.711 ,  -16.711 ,   13.213	},
{-4.0315e-16,    0.081564,        92.65 ,     4.2955 ,    2.8705,    2.8705,    4.2955	},
{	-6.4422 ,     5.2851 ,      42.955  ,     113.6  ,   11.776 ,   -100.8 ,  -19.497	},
{	-5.4748 ,    -6.6843 ,      28.705  ,    11.776  ,   116.96 ,   20.665 ,   -100.8	},
{	5.4748  ,   -6.6843  ,     28.705   ,   -100.8   ,  20.665  ,  116.96  ,  11.776	},
{	6.4422  ,    5.2851  ,     42.955   ,  -19.497   ,  -100.8  ,  11.776  ,   113.6	}};
					 
	
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
				//du [i][j] += k_sf [i][k] * dx [k][j];
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
			for (int j = 0 ; j < 1 ; j ++)
			{
				x_temp [i][j] = (x [i][j] - x_OB [i][j]) ;
			}
		}
		//xhp+=G*(y-Ch*xh)
		for (int i=0 ; i < 7 ; i ++)
		{
			for (int j = 0 ; j < 1 ; j ++)
			{
				for (int k = 0 ; k < 7 ; k ++)
				{
					x_OB [i][j] += G [i][k] * x_temp [k][j]*d_time;
				}
			}
		}
	
	//xhp+=Ah*xh
	for (int i=0 ; i < 7 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			for (int k = 0 ; k < 7 ; k ++)
			{
				x_OB [i][j] += A [i][k] * x_OB [k][j]*d_time;
			}
		}
	}
	
	//xhp+=Bh*uh
	for (int i=0 ; i < 7 ; i ++)
	{
		for (int j = 0 ; j < 1 ; j ++)
		{
			for (int k = 0 ; k < 4 ; k ++)
			{
				x_OB [i][j] += B [i][k] * ud [k][j]*d_time;
			}
		}
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
	Robot.orc_length=128;
	max_ocr = (Robot.orc_length << 8) - 1 ;
}





