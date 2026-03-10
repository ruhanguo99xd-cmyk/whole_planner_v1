#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <fstream>
#include <Eigen/Dense>
#include <sstream>
#include <string>
#include <point_cloud_processing_pkg/srv/wuliaoprocess.hpp>

using namespace std;
using namespace Eigen;
using Wuliaoprocess = point_cloud_processing_pkg::srv::Wuliaoprocess;

MatrixXd CreateGramianMatrix(MatrixXd x, int NbVariables, int PRSDegree)
{
	int NbPoints = x.cols();

	// create X
	MatrixXd X(x.rows(), 1);
	X.setOnes();

	if (PRSDegree > 0)
	{
		//X = [X x];
		X.conservativeResize(X.rows(), X.cols() + x.cols());
		X.rightCols(x.cols()) = x;

		MatrixXd X1(x.rows(), x.cols());
		X1 = x;
		int nc1 = NbVariables;
		MatrixXd n_loc(1, NbVariables + 1);//根据k+1
		for (int i = 0; i < NbVariables; i++)
		{
			n_loc(i) = i + 1;
		}
		n_loc(2) = 0;
		//cout << n_loc << endl;
		MatrixXd n_loc1(1, NbVariables + 1);
		n_loc1 = n_loc; //后面是直接对n_loc1进行赋值，故随便给它除第1个数以外的数一个值也没关系

						//MatrixXd X2(x.rows(), 1);
		for (int i = 2; i <= PRSDegree; i++)
		{

			int nr = X1.rows();
			int nc = X1.cols();
			MatrixXd X2(x.rows(), 1);
			int ctr = 0;  //col函数是从0开始的
			int k = 0;
			for (; k < NbVariables; k++)
			{
				
				int l_ctr = 0;
				int j = n_loc(k);
				for (; j <= nc; j++)
				{
					if (ctr > 0)
					{
						X2.conservativeResize(X2.rows(), X2.cols() + 1);
					}
					X2.col(ctr) = x.col(k).array() * X1.col(j - 1).array();  // X2(:,ctr) = x(:,k).*X1(:,j); j是从1开始的，k是从0开始的,col函数也是从0开始的
					ctr += 1;
					l_ctr += 1;
				}
				//这里n_loc和n_loc1，只关心它里面的值，只要位置对(k,k+1)，不用担心C++从0开始这样顺序的问题
				n_loc1(k + 1) = l_ctr + n_loc1(k);
			}
			nc1 = nc;

			//把X2列拼接到X的右边
			X.conservativeResize(X.rows(), X.cols() + X2.cols());
			X.rightCols(X2.cols()) = X2;

			//重新定义X1，并把X2赋值给X1，因为这里X2列的大小一直在变
			X1.conservativeResize(X1.rows(), X1.cols() + X2.cols() - X1.cols());
			//MatrixXd X1(X2.rows(), X2.cols());
			X1 = X2;

			//重新定义n_loc，并把n_loc1赋值给n_loc，因为这里n_loc1列的大小一直在变
			//MatrixXd n_loc(n_loc1.rows(), n_loc1.cols());
			n_loc = n_loc1;
		}
	}

	return X;
}

MatrixXd PRSFit(MatrixXd x_train, MatrixXd y_train, int prs_degree)
{
	int NbVariables = x_train.cols();
	int PRS_Degree = prs_degree;

	MatrixXd GramianMatrix;
	// std::cout << "1" << std::endl;
	GramianMatrix = CreateGramianMatrix(x_train, NbVariables, PRS_Degree);
	// std::cout << "2" << std::endl;

	//BDCSVD分解
	//time_stt = clock();
	MatrixXd beta;
	beta = GramianMatrix.bdcSvd(ComputeThinU | ComputeThinV).solve(y_train);
	/*cout << "BDCSVD分解得到的多项式系数：" << endl;
	cout << beta << endl;
	cout << "BDCSVD分解 time is:" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;
	cout << "-----------------------------------" << endl;*/

	MatrixXd PRS_Beta;
	PRS_Beta = beta;

	return PRS_Beta;
}


class PointCloudProcessingNode : public rclcpp::Node
{
public:
    PointCloudProcessingNode() : Node("prsdata_server")
    {
        // 创建服务
        server_ = create_service<Wuliaoprocess>(
            "process_pointcloud", 
            std::bind(&PointCloudProcessingNode::processServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(get_logger(), "服务已启动，等待客户端请求...");
        
        // 初始化数据
        initialize_data();
    }

private:
    rclcpp::Service<Wuliaoprocess>::SharedPtr server_;
    std::array<double, 28> prs_coefficients_;  
    std::array<double, 6> bounding_boxes_; 
    int PRSDegree = 6; // 专门存储有效范围（6个元素）
    int num_beta = ((1 + (PRSDegree+1)) * (PRSDegree+1)) / 2;

	// 新增：标记PRS数据是否初始化成功（核心新增）
    bool data_initialized_success_ = false;

    void initialize_data()
    {
		// // 1.直接用点云计算
        // std::string file_name = "/home/pipixuan/tra_planning1/ros2_ws/src/tra_planning/data/xiaoyangji-test.txt";
		// ifstream myfile(file_name);
    	// if (!myfile.is_open()) {  // 先检查文件是否能打开
        // RCLCPP_ERROR(this->get_logger(), "无法打开文件: %s", file_name.c_str());
        // return;
    	// }
		// std::string line_num;
		// int i = 0;
		// while (getline(myfile, line_num)) // line中不包括每行的换行符
		// {
		// 	i = i + 1;
		// }
		// //cout << "read line row : "<< i << endl;
		// RCLCPP_INFO(this->get_logger(), "read line row : %d", i); 
		// Eigen::MatrixXd dianyun_x_(1, i);
		// Eigen::MatrixXd dianyun_y_(1, i);
		// Eigen::MatrixXd dianyun_z_(1, i);
		// float temp_x = 0.0;
		// float temp_y = 0.0;
		// float temp_z = 0.0;
		// ifstream myfile_2(file_name);
		// std::string line;
		// int j = 0;
		// while (getline(myfile_2, line)) // line中不包括每行的换行符
		// {
		// 	istringstream ss(line);
		// 	std::vector<float> v;
		// 	float temp;
		// 	if (ss >> temp)
		// 	{
		// 		temp_x = temp;
		// 	}
		// 	if (ss >> temp)
		// 	{
		// 		temp_y = temp;
		// 	}
		// 	if (ss >> temp)
		// 	{
		// 		temp_z = temp;
		// 	}
		// 	dianyun_x_(j) = temp_x-0.3;
		// 	dianyun_y_(j) = temp_y;
		// 	dianyun_z_(j) = temp_z;
		// 	//cout << temp_x << temp_y << temp_z << endl;
		// 	j = j + 1;
		// }
		// Eigen::MatrixXd dianyun(3, i);
		// dianyun.row(0) = dianyun_x_;
		// dianyun.row(1) = dianyun_y_;
		// dianyun.row(2) = dianyun_z_;
		// //std::cout << "read MatrixXd row : " << dianyun.rows() << ", cols: " << dianyun.cols()<< std::endl;
		// // RCLCPP_INFO(node->get_logger(), "read MatrixXd row : %d, cols: %d", dianyun.rows(), dianyun.cols());
		// // ------------------------------------fitting---------------------------------------
		// int dianyun_num = dianyun.cols();
		// //矩阵的样子是为了满足拟合的需要
		// Eigen::MatrixXd duiliao_x_train(dianyun_num, 2);
		// duiliao_x_train.col(0) = dianyun.row(0);
		// duiliao_x_train.col(1) = dianyun.row(1);
		// Eigen::MatrixXd duiliao_y_train(dianyun_num, 1);
		// duiliao_y_train.col(0) = dianyun.row(2);
		// // std::cout << "duiliao_x_train row : " << duiliao_x_train.rows() << ", cols: " << duiliao_x_train.cols()<< std::endl;
		// // std::cout << "duiliao_y_train row : " << duiliao_y_train.rows() << ", cols: " << duiliao_y_train.cols()<< std::endl;
		// Eigen::MatrixXd PRS_Beta; // 用于存储点云堆料面的轨迹系数
		// // PRSDegree=5; //拟合物料面的系数
		// PRS_Beta = PRSFit(duiliao_x_train, duiliao_y_train, PRSDegree);

        // for (int i = 0; i < PRS_Beta.size(); ++i) {
        //     prs_coefficients_[i] = PRS_Beta(i);  // 将Eigen矩阵元素复制到vector
        // }

        // std::cout << "物料面拟合参数：" <<  std::endl;

		// for (int i = 0; i < num_beta; i++)
		// {
		// 	std::cout << "  " << PRS_Beta(i) <<","<<  std::endl;
		// }

		// 无点云数据时用默认值
    	bounding_boxes_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        //2.测试，模拟感知提供的PRS数据
        prs_coefficients_ = {
			4.84569,
			8.97557,
			0.629824,
			5.8445,
			0.430012,
			0.00231979,
			1.86044,
			0.0890789,
			0.0133216,
			-0.0595269,
			0.32589,
			0.00570019,
			0.0159389,
			-0.0298727,
			0.00256462,
			0.0294345,
			-2.77559e-05,
			0.00378471,
			-0.00625137,
			0.0013921,
			0.00123289,
			0.00106127,
			-2.02607e-06,
			0.000249356,
			-0.000438719,
			0.000107564,
			0.000134273,
			8.73145e-05,
        };
    }

    void processServiceCallback(
        const std::shared_ptr<Wuliaoprocess::Request> request,
        std::shared_ptr<Wuliaoprocess::Response> response)
    {
        (void)request; // 未使用请求参数
		
        response->issuccess = true;  // 必须添加这一行！
        // 将数据复制到响应中
        response->prs_coefficients = prs_coefficients_;
        response->bounding_boxes = bounding_boxes_;     

        RCLCPP_INFO(get_logger(), "发送PRS系数（%zu个）和边界框（%zu个）",prs_coefficients_.size(), bounding_boxes_.size());

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}    