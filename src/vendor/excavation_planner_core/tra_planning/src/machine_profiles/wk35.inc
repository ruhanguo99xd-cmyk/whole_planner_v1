//------------------------------------------WK-35参数------------------------------------------//
//1.机械参数
double w = 4.724; // 铲斗宽度
double Lb = 12.1; // 推杆长度
double Ld = 4.882; // 铲斗长度
double H = 9.715; // 鞍座回转中心到水平面距离

double EF = 2.862; // 铲斗尺寸相关
double PF = 1.864; // 铲斗提升点相关
double MN = 2.137;  // 铲底相关
double NF = 0.33;  // 铲底相关
double ED = 1.841; // 铲尖相关尺寸

double r_tianlun = 1.2225; // 天轮半径
double Lbi = 10.668; // 鞍座中心到天轮齿心的距离
double Angle_db = 45 * M_PI / 180; //大臂与地面的夹角

double mb = 35000; // 斗杆质量，kg
double md0 = 57000; // 铲斗重量，kg
bool isempty;

double dOC = 8.462;  // 鞍座中心到回转中心的水平距离
int trajectory_number = 8; //挖掘轨迹截面，取斗齿数量

double AC = 2.32;  // 铲齿相关
double Relieving_angle = 21 * M_PI / 180; //齿尖角


//2.土力学参数 
double phi = 42 * M_PI / 180;  //内摩擦角
double rou = 0.5;      //料堆滑裂面倾角
double midu = 1800;     //土壤密度
double deta = 40 * M_PI / 180; //外摩擦角
double g = 9.8;    //重力常数
double cc = 0.5;   //堆料内聚力

//3.约束参数
double nominal_load_cap = 35; //单斗挖掘体积
double max_allowable_cap;   //最大容积
double min_allowable_cap;   //最小容积
double max_allowable_OF = 12.5;  //最大允许OF
double max_allowable_v_gan = 0.64;  //最大杆速
double max_allowable_v_rope = 1.58;  //最大绳速
double min_allowable_y_M = 0.4;  //最小允许的y_M全局坐标
double min_allowable_tip_height = 6.0;  //结束点最小允许的齿尖高度
double min_d1 = 1.0;  //最小允许铲尖与料堆高度差
double min_allowable_v_gan = -0.5; //最小杆速
double max_allowable_Fs = 2600000; 
double max_allowable_Fg = 1000000;
double max_allowable_Ps = 1700; 
double max_allowable_Pg = 625;

//4.优化参数
double lb[5] = { -100000, 0,        8,  6,  8 };
double ub[5] = { 0,       100000,  14, 18, 14 };
double seg3_x0[5] = {
    -18119.3,   // x[0] -> a6_x (×1e-9)
     17729.0,   // x[1] -> a6_y (×1e-9)
     10.4262,   // x[2] -> x_tf
     11.2978,   // x[3] -> y_tf
     11.3       // x[4] -> tf (s)
};

//5.轨迹路径相关
double dig_distance_min = 10.0;   // 最小挖掘距离
double dig_distance_max = 15.0;   // 最大挖掘距离

double s_lo = 9.0;     //交点搜索区间下限
double s_hi = 18.0;     //交点搜索区间上限
double ds = 0.2;        //交点搜索步长

double distip_x = 8; //估计物料斜率用

double B0_deg_seg01 = 10.0;   // 第一段起始位置角度，度
double h0_seg01   = 0.5;    // 第一段起始高度
double v23x = 1.0;  // 二-三段连接速度
double dig_T2 = 3.0;    // 第二段挖掘时间
double dig_x2 = 2.0;   // 第二段挖掘距离
double init_tip_height = 0.0 ; // 第三段起始点高度
double B0 ; // 起始点角度
// 零初始化，实际以计算值为准

//6.推压/提升编码值相关
double L0_tianlun = 4.45;                 // 提升零位（m）
double d_juantong  = 1.425;               // 提升卷筒直径（m）
double ratio_tuiya = 5.13;                // 推压减速比（WK-35为例，推压轴齿轮比推压二轴齿轮）
double d_tuiya = 0.68;                    // 推压齿轮直径（m）
double OE_zero  = 8.621;                  // 推压零位（m）
double cpr = 4096.0;                      // 编码器每转脉冲数
