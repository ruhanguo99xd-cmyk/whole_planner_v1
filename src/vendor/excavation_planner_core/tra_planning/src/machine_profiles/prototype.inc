//------------------------------------------样机参数------------------------------------------//
// 1.机械参数
double w = 0.7 ; // 铲斗宽度
double Lb = 1.93; // 推杆长度
double Ld = 0.933; // 铲斗长度
double H = 1.449; // 鞍座回转中心到水平面距离

double EF = 0.471; // 铲斗尺寸相关
double PF = 0.377; // 铲斗提升点相关
double MN = 0.364;  // 铲底相关
double NF = 0.017;  // 铲底相关
double ED = 0.357; // 铲尖相关尺寸


double r_tianlun = 0.159; // 天轮半径
double Lbi = 2.128; // 鞍座中心到天轮齿心的距离
double Angle_db = 45 * M_PI / 180; //大臂与地面的夹角

double mb = 129.38; // 斗杆质量，kg
double md0 = 364.1; // 铲斗重量，kg
bool isempty;

double dOC = 1.645;  // 鞍座中心到回转中心的水平距离
int trajectory_number = 5; //挖掘轨迹截面，取斗齿数量

double AC = 0.466;  // 铲齿相关
double Relieving_angle = 26 * M_PI / 180; //齿尖角

// 2.土力学参数 
double phi = 42 * M_PI / 180;  //内摩擦角
double rou = 0.5;      //料堆滑裂面倾角
double midu = 1800;     //土壤密度
double deta = 40 * M_PI / 180; //外摩擦角
double g = 9.8;    //重力常数
double cc = 0.5;   //堆料内聚力

// 3.约束参数
double nominal_load_cap = 0.17; //单斗挖掘体积  0.17
double max_allowable_cap; 
double min_allowable_cap;
double max_allowable_OF = 1.754;  //最大允许OF 2.234(图纸) 但是有限位存在，到不了那么多
double max_allowable_v_gan = 0.110;   // 最大速度计算理论值 0.12
double max_allowable_v_rope = 0.215;   // 0.232
double min_allowable_y_M = 0.1;  //最小允许的y_M全局坐标
double min_allowable_tip_height = 1.65;  //结束点最小允许的齿尖高度   1.55!  
double min_d1 = 0.5;  //最小允许铲尖与料堆高度差  0.5!
double min_allowable_v_gan = -0.08;  //最小杆速 -0.1
double max_allowable_Fs = 47000; 
double max_allowable_Fg = 25000;
double max_allowable_Ps = 11; 
double max_allowable_Pg = 3;

// 4.优化参数
double lb[5] = { -50000, 0,       1,  1,   10 }; //8
double ub[5] = { 0,      50000,  4,  3,  15 }; //16
double seg3_x0[5] = {
        -18119.3,   // x[0] -> a6_x (×1e-9)
        5729.0,   // x[1] -> a6_y (×1e-9)
        2.4262,   // x[2] -> x_tf
        1.5978,   // x[3] -> y_tf，实际测量时的高度对应的是 这个值 + h0_seg01
        11.3      // x[4] -> tf (s)
};

// 5.轨迹路径相关
double dig_distance_min = 2.2;   // 最小挖掘距离
double dig_distance_max = 2.8;   // 最大挖掘距离

double s_lo = 2.15;     //交点搜索区间下限
double s_hi = 3.00;      //交点搜索区间上限
double ds = 0.05;        //交点搜索步长

double distip_x = 1.1;       //估计物料斜率用
double B0_deg_seg01 = 24.5;  // 第一段起始位置角度，度 23.8   23.5!  24.0
double h0_seg01  = 0.15;     // 第一段起始高度 0.2 0.12  0.04! 0.10
double v23x = 0.0;  // 二-三段连接速度
double dig_T2 = 1.6;   // 第二段挖掘时间
double dig_x2 = 0.14;   // 第二段挖掘距离
double init_tip_height = 0.15; // 第三段起始点高度
double B0 ; // 起始点角度
// 零初始化，实际以计算值为准

// 6.推压/提升编码值相关
// 这里只是附上一组默认参数，具体计算时应用的是launch文件中的参数
double L0_tianlun = 0.434;             // 提升零位（m）,要调整，0.139（L1,130度）+0.428（L2） 0.567  11578  0.42
double d_juantong  = 0.24922;        // 提升卷筒直径（m） 0.25 0.261056
double ratio_tuiya = 3.1136;          // 推压减速比（推压轴齿轮比推压二轴齿轮）,调整  2.9755  3.1136
double d_tuiya = 0.14;                // 推压齿轮直径（m）
double OE_zero  = 1.285;              // 推压零位（m），要调整,要调整系数需要调整4096  1.339（图纸） 4528 1.338
double cpr = 4096.0;                // 编码器每转脉冲数
