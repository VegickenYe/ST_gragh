#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

double initial_yaw1;
double car_d_cr_,distance_resolution_,car_width_,time_resolution_,trajectory2_x_,trajectory2_y_,trajectory3_x_,trajectory3_y_;
int cars_num_,car_id_,time_samples_;
std::vector<Eigen::Vector2d> trajectory1;
std::vector<Eigen::Vector2d> trajectory2;
std::vector<Eigen::Vector2d> trajectory3;
std::vector<double> yaw1;
std::vector<double> yaw2;
std::vector<double> yaw3;
std::vector<double> t1;
std::vector<double> t2;
std::vector<double> t3;
double bias1_;
double bias2_;
double bias3_;

struct TrajectoryData {
    std::vector<Eigen::Vector2d> trajectory[3];
    std::vector<double> yaw[3];
    std::vector<double> timestamp[3];
};

struct Graph {
    int s_voxels_num;
    int t_voxels_num;
    double time_resolution = time_resolution_;
    double distance_resolution = distance_resolution_;
    std::vector<Eigen::Vector3i> s_t_graph;
};

std::vector<Eigen::Vector2d> readTrajectory(const std::string& file_name) {
    std::ifstream file(file_name);
    std::vector<Eigen::Vector2d> trajectory;
    trajectory.clear();
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << file_name << std::endl;
        return trajectory;  // 返回空向量
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        Eigen::Vector2d point;

        std::getline(ss, cell, ',');
        point[0]=std::stod(cell);
        std::getline(ss, cell, ',');
        point[1]=std::stod(cell);

        trajectory.push_back(point);
    }

    file.close();
    return trajectory;
}

std::vector<double> readYaw(const std::string& file_name) {
    std::ifstream file(file_name);
    std::vector<double> yaw;
    yaw.clear();
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << file_name << std::endl;
        return yaw;  // 返回空向量
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        double each_yaw;

        // 跳过前两个元素
        std::getline(ss, cell, ',');
        std::getline(ss, cell, ',');

        // 读取第三个元素
        std::getline(ss, cell, ',');
        each_yaw=std::stod(cell);

        yaw.push_back(each_yaw);
    }

    file.close();
    return yaw;
}


double readInitialYaw(const std::string& file_name) {
    std::ifstream file(file_name);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << file_name << std::endl;
        return -1.0;  // 返回一个错误值，表示函数执行失败
    }

    std::string line;
    std::getline(file, line);  // 读取第一行
    file.close();

    std::stringstream ss(line);
    std::string cell;

    // 跳过前两个元素
    std::getline(ss, cell, ',');
    std::getline(ss, cell, ',');

    // 读取第三个元素
    std::getline(ss, cell, ',');

    try {
        return std::stod(cell);  // 将字符串转换为double
    } catch (const std::exception& e) {
        std::cerr << "Error parsing yaw: " << e.what() << std::endl;
        return -1.0;  // 返回一个错误值，表示函数执行失败
    }
}

const double kPi = acos(-1.0);
double normalize_angle(const double& theta)
  {
    double theta_tmp = theta;
    theta_tmp -= (theta >= kPi) * 2 * kPi;
    theta_tmp += (theta < -kPi) * 2 * kPi;
    return theta_tmp;
  }

Eigen::Vector2d* getPos(TrajectoryData data, int sur_id, double pt_time) {
    Eigen::Vector2d* piont = nullptr; // 初始化为 nullptr 表示空指针
    for (int i = 0; i < time_samples_; i++) {
        if ((data.timestamp[sur_id][i]-pt_time<=1e-3)&&(data.timestamp[sur_id][i]-pt_time>=-1e-3)) {
            if((sur_id==2)&&((data.trajectory[sur_id][i]).norm()<=1e-3)){
                std::cout<<"第"<<i<<"点"<<std::endl;
                std::cout<<"data.trajectory[sur_id][i] "<<data.trajectory[sur_id][i]<<std::endl;
                std::cout<<data.timestamp[sur_id][i]<<std::endl;  
            }
            piont = new Eigen::Vector2d(data.trajectory[sur_id][i]); // 找到匹配项时，返回指向该位置的指针
            // piont = &(data.trajectory[sur_id][i]); 
            break; // 找到匹配项后，退出循环
        }
    }
    return piont; // 返回指针，可以为 nullptr 表示未找到匹配项
}

double* getYaw(TrajectoryData data, int sur_id, double pt_time) {
    double* yaw = nullptr; // 初始化为 nullptr 表示空指针
    for (int i = 0; i < time_samples_; i++) {
        if ((data.timestamp[sur_id][i]-pt_time <=1e-3)&&(data.timestamp[sur_id][i]-pt_time>=-1e-3)) {
            // if((sur_id==2)&&(pt_time==2.02)){
            //     std::cout<<"第"<<i<<"点"<<std::endl;
            //     std::cout<<data.timestamp[sur_id][i]<<std::endl;
            //     std::cout<<pt_time<<std::endl;
            //     std::cout<<1e-3<<std::endl;
            //     std::cout<<data.timestamp[sur_id][i]-pt_time<<std::endl;
            //     std::cout<<data.trajectory[sur_id][i]<<std::endl;
            // }
            yaw = new double(data.yaw[sur_id][i]);// 找到匹配项时，返回指向该位置的指针
            // yaw = &(data.yaw[sur_id][i]); 
            break; // 找到匹配项后，退出循环
        }
    }
    return yaw; // 返回指针，可以为 nullptr 表示未找到匹配项
}

// bool searchTime(std::vector<std::vector<double>> &flat_trajs, double &start_world_time)
Graph searchTime(TrajectoryData data,double &start_world_time)
{
    std::cout<<"世界开始时间(以轨迹1为参考时间)"<<start_world_time<<std::endl;
    /*-------Preliminaries: Judge the cars' moving direction------------*/
    int start_direction;
    double init_yaw = normalize_angle(initial_yaw1);
    Eigen::Vector2d initAngleDir(cos(init_yaw), sin(init_yaw));
    Eigen::Vector2d initdir = data.trajectory[car_id_][1] - data.trajectory[car_id_][0];
    if(initAngleDir.dot(initdir) >= 0)
        start_direction = 1;
    else
        start_direction = -1;
    /*------------------------------------------------------------------*/
    /*build s-t graph step1: discrete every point from the position list*/
    std::vector<double> distance2start;
    std::vector<Eigen::Vector2d> discrete_positionList;
    int end_index = time_samples_-1;
    double distance = 0.0;
    distance2start.push_back(distance);
    for(int i = 1; i <=end_index; i++)
    {
        Eigen::Vector2d start_pos = data.trajectory[car_id_][i-1];
        Eigen::Vector2d end_pos = data.trajectory[car_id_][i];
        double node_distance = (end_pos - start_pos).norm();
        distance += node_distance;
        distance2start.push_back(distance);
    }
    double total_s_ = distance;
    std::cout<<"总长度为"<<total_s_<<std::endl;
    discrete_positionList.push_back(data.trajectory[car_id_][0]);
    for(double dis = distance_resolution_; dis < total_s_; dis += distance_resolution_)
    {
        int locate_idx = 0;
        for(int i = 1; i <= end_index; i++)
        {
            if(dis > distance2start[i-1] && dis <= distance2start[i])
            {
                locate_idx = i;
            }
        }
        double d1 = distance2start[locate_idx-1];
        double deltad = dis - d1;
        Eigen::Vector2d pos1 = data.trajectory[car_id_][locate_idx - 1];
        Eigen::Vector2d pos2 = data.trajectory[car_id_][locate_idx];
        Eigen::Vector2d pos_direction = pos2 - pos1;
        double node_dis = (pos2 - pos1).norm();
        Eigen::Vector2d node_direction = pos2 - pos1;
        Eigen::Vector2d discrete_point = pos1 + deltad / node_dis * node_direction;
        double yaw_angle = atan2(start_direction * pos_direction(1), start_direction * pos_direction(0));
        Eigen::Matrix2d Rot_mat;
        Rot_mat << cos(yaw_angle), -sin(yaw_angle),
                    sin(yaw_angle), cos(yaw_angle);          
        discrete_point = discrete_point + Rot_mat * Eigen::Vector2d(car_d_cr_, 0.0);
        discrete_positionList.push_back(discrete_point);
    }
    discrete_positionList.push_back(data.trajectory[car_id_][end_index]);
    std::cout<<"轨迹1离散点个数为"<<discrete_positionList.size()<<std::endl;

    /*------------------------------------------------------------------*/
    
    /*build s-t graph step2: enumurate every point of other car's trajectory and check collision*/
    
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    double collision_distance = car_width_;
    
    
    
    std::vector<Eigen::Vector3i> s_t_graph;
    int max_int_time = 0;
    for(int sur_id = 0; sur_id < cars_num_; sur_id++)
    {
        if(sur_id == car_id_)
            continue;

        double traj_start_time = data.timestamp[sur_id][0];
        double traj_end_time = data.timestamp[sur_id][end_index];
        for(double world_t = start_world_time; world_t <= traj_end_time + 1e-3; world_t += time_resolution_)
        {
            double pt_time;
            if(world_t < traj_start_time)
            {
                pt_time = traj_start_time;
            }
            else if(world_t >= traj_start_time && world_t <= traj_end_time)
            {
                pt_time = world_t;
            }
            else
            {
                pt_time = traj_end_time;
            }
            int point_id=world_t/time_resolution_;
            int time_int = floor((world_t - start_world_time) / time_resolution_);
            if(time_int > max_int_time)
            {
                max_int_time = time_int;
            }
            // Eigen::Vector2d surround_p = data.trajectory[sur_id][point_id];  //读取第sur_id条轨迹的第point_id个点
            // std::cout<<"sur_id:"<<sur_id<<std::endl;
            // std::cout<<"world_t:"<<world_t<<std::endl;
            // std::cout<<"traj_start_time:"<<traj_start_time<<std::endl;
            // std::cout<<"traj_end_time:"<<traj_end_time<<std::endl;
            // std::cout<<"pt_time:"<<pt_time<<std::endl;
            Eigen::Vector2d* surround_ptr = getPos(data,sur_id,pt_time);
            Eigen::Vector2d surround_p = *surround_ptr;
            // if((sur_id==2)&&(surround_p.norm()<=1e-3)){
            //     std::cout<<"surround_p "<<surround_p<<std::endl; 
            // }
            delete surround_ptr;
            // std::cout<<surround_p<<std::endl;
            // if (surround_ptr != nullptr) {
            //     // 找到了匹配项，result 指向匹配项的位置
            //     Eigen::Vector2d surround_p = *surround_ptr;
            // } else {
            //     // 未找到匹配项，处理未找到的情况
            //     std::cout << "No matching point found." << std::endl;
            // }
            double* surround_angleptr = getYaw(data,sur_id,pt_time);
            // std::cout<<"1"<<std::endl;
            double surround_angle = *surround_angleptr;
            delete surround_angleptr;
            // if (surround_angleptr != nullptr) {
            //     // 找到了匹配项，result 指向匹配项的位置
            //     double surround_angle = *surround_angleptr;
            // } else {
            //     // 未找到匹配项，处理未找到的情况
            //     std::cout << "No matching yaw found." << std::endl;
            // }
            // std::cout<<"1"<<std::endl;
            Eigen::Matrix2d Rot_mat;
            Rot_mat << cos(surround_angle), -sin(surround_angle),
                        sin(surround_angle), cos(surround_angle);
            surround_p = surround_p + Rot_mat * Eigen::Vector2d(car_d_cr_, 0.0);
            // if there are only 2 discrete points, no collision check is needed
            for(int s_int = 0; s_int < discrete_positionList.size() - 1; s_int++)
            {
                if((surround_p - discrete_positionList[s_int]).norm() < collision_distance)
                {
                    // if ((s_int<=10)&&(sur_id<=2))
                    // {
                    //     std::cout<<"surround_p "<<surround_p<<std::endl;
                    //     std::cout<<"discrete_positionList[s_int] "<<s_int<<" "<<discrete_positionList[s_int]<<std::endl;
                    // }
                    
                    // char ch;
                    // std::cout << "Press 'q' and then Enter to continue: ";
                    // std::cin >> ch;

                    // while (ch != 'q') {
                    //     std::cout << "You pressed '" << ch << "'. Press 'q' and then Enter to continue: ";
                    //     std::cin >> ch;
                    // }
                    Eigen::Vector3i s_t_collision(time_int, s_int, sur_id);
                    // std::cout<<"sur_id "<<sur_id<<std::endl;
                    // std::cout<<"point_id "<<point_id<<std::endl;
                    // std::cout<<surround_p[0]<<" "<<surround_p[1]<<std::endl;
                    // std::cout<<discrete_positionList[s_int][0]<<" "<<discrete_positionList[s_int][1]<<std::endl;
                    // std::cout<<(surround_p - discrete_positionList[s_int]).norm()<<std::endl;
                    // std::cout<<s_t_collision<<std::endl;
                    s_t_graph.push_back(s_t_collision);
                }
            }
        }
    }
    std::cout << "The number of s-t-collision parts are: " << s_t_graph.size() << std::endl;
    /*------------------------------------------------------------------------------------------*/
    /*Build s-t graph step3: allocate memeory for the s-t graph*/
    int max_int_s = discrete_positionList.size() - 2;
    int max_int_t = max_int_time;
    int s_voxels_num = max_int_s + 1;
    int t_voxels_num = max_int_t + 1;
    int max_voxels = s_voxels_num * t_voxels_num;
    std::vector<int> s_t_graph_vector;
    s_t_graph_vector.resize(max_voxels);
    fill(s_t_graph_vector.begin(), s_t_graph_vector.end(), 0);
    
    for(int i = 0; i < s_t_graph.size(); i++)
    {
        int time_int = s_t_graph.at(i)(0);
        int distance_int = s_t_graph.at(i)(1);
        int time_distance_int = time_int + distance_int * t_voxels_num;
        s_t_graph_vector[time_distance_int] = 1;
    }
    Graph gragh;
    gragh.s_voxels_num = s_voxels_num;
    gragh.t_voxels_num = t_voxels_num;
    gragh.s_t_graph = s_t_graph;
    /*----------------------------------------------------------*/
    return gragh;
}
    
void writeGraphToFile(const Graph& g, const std::string& filename) {
    std::ofstream file(filename);
    
    // 写入基本属性
    file << g.s_voxels_num << " " << g.t_voxels_num << " ";
    file << g.time_resolution << " " << g.distance_resolution << "\n";

    // 写入所有点的坐标
    for (const auto& point : g.s_t_graph) {
        file << point[0] << " " << point[1] << " " << point[2] << "\n";
    }
}

std::vector<double> setTimestamp(const double bias){
    std::vector<double> timestamp;
    for (int i = 1; i < time_samples_+1; ++i) {

        double t = i*time_resolution_+bias;
        timestamp.push_back(t);
    }
    return timestamp;
}

int main() {
    //常量设置
    car_d_cr_ = -0.04; //雷达到中心点位置(前置为负数，后置为正数)
    distance_resolution_ = 0.01; //离散单位位置
    car_width_=0.3; 
    cars_num_ =3;
    car_id_=0; 
    time_resolution_ = 0.02; //离散单位时间
    time_samples_= 500; 

    //时间异步（以画st图的轨迹为参考时间，影响上下移动）
    bias1_=0.0;
    bias2_=-2.0;
    bias3_=2.0;

    //轨迹的x和y的增长和缩小比例（影响斜率）
    trajectory2_x_ = 1.0; 
    trajectory2_y_ = 1.0;
    trajectory3_x_ = 1.0;
    trajectory3_y_ = 1.0;

    // 读取三个CSV文件
    trajectory1 = readTrajectory("trajectory1.csv");
    trajectory2 = readTrajectory("trajectory2.csv");
    trajectory3 = readTrajectory("trajectory3.csv");
    yaw1 = readYaw("trajectory1.csv");
    yaw2 = readYaw("trajectory2.csv");
    yaw3 = readYaw("trajectory3.csv");
    //调整轨迹
    for (Eigen::Vector2d& point : trajectory2) {
        point[0] *=trajectory2_x_;
        point[1] *=trajectory2_y_;
    }
    for (Eigen::Vector2d& point : trajectory3) {
        point[0] *=trajectory3_x_;
        point[1] *=trajectory3_y_;
    }
  
    t1 = setTimestamp(bias1_);
    t2 = setTimestamp(bias2_);
    t3 = setTimestamp(bias3_);
    
    TrajectoryData data;

    data.trajectory[0]=trajectory1; // 添加到 trajectory1
    data.trajectory[1]=trajectory2; // 添加到 trajectory2
    data.trajectory[2]=trajectory3; // 添加到 trajectory3

    data.yaw[0]=yaw1; // 添加到 yaw1
    data.yaw[1]=yaw2; // 添加到 yaw2
    data.yaw[2]=yaw3; // 添加到 yaw3

    data.timestamp[0]=t1; // 添加到 timestamp1
    data.timestamp[1]=t2; // 添加到 timestamp2
    data.timestamp[2]=t3; // 添加到 timestamp3

    
    // for (const auto& t : t2) {
    //     std::cout << "t: " << t<< std::endl;
    // }
    std::cout << "成功保存了轨迹"<< std::endl;
        double initial_yaw1 = readInitialYaw("trajectory1.csv");

    if (initial_yaw1 != -1.0) {
        std::cout << "Initial yaw: " << initial_yaw1 << std::endl;
    } else {
        std::cout << "Failed to read initial yaw." << std::endl;
    }
    Graph graph;
    graph = searchTime(data,t1[0]);
    std::string filename="s_t_graph.txt";
    writeGraphToFile(graph,filename);
    return 0;
}
