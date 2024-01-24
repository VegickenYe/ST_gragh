#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <set>
#include <unordered_map>
#include <eigen3/Eigen/Dense>
#include "save_traj.h"
#include <chrono>

void writevectorToFile(const std::vector<Eigen::Vector2i>& vec, const std::string& filename) {
    std::ofstream file(filename);

    if (file.is_open()) {
        for (const auto& item : vec) {
            file << item.x() << " " << item.y() << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
    }
}

void saveToCSV(const TrajectoryData& data, int index, const std::string& filename) {
    std::ofstream file(filename);

    if (file.is_open()) {
        // 写入表头
        file << "X,Y,Yaw,Timestamp\n";

        // 获取当前索引下的数据大小
        size_t dataSize = data.trajectory[index].size();

        // 写入数据
        for (size_t i = 0; i < dataSize; ++i) {
            file << data.trajectory[index][i].x() << "," 
                 << data.trajectory[index][i].y() << ","
                 << data.yaw[index][i] << "," 
                 << data.timestamp[index][i] << "\n";
        }

        file.close();
    } else {
        std::cerr << "Unable to open file " << filename << std::endl;
    }
}


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
Graph searchTime(TrajectoryData& data,double &start_world_time,KinoTrajData &flat_trajs,std::vector<Eigen::Vector2i> &Find_s_t)
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
    total_s_ = distance;
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
        std::cout<<"1"<<std::endl;
        double traj_start_time = data.timestamp[sur_id][0];
        double traj_end_time = data.timestamp[sur_id].back();
        std::cout<<"traj_end_time: "<<traj_end_time<<std::endl;
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
    std::cout << "max_int_time: " << max_int_time << std::endl;
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

    /*-----------------------start to search!-------------------*/
    middle_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; i++)
    {
        middle_node_pool_[i] = new MiddleNode;
    }
    for(int i = 0; i < use_time_node_num_; i++)
    {
        MiddleNodePtr node = middle_node_pool_[i];
        node->parent = NULL;
        // std::cout << "2" << std::endl;
    }


    std::cout << "start to search!" << std::endl;
    expanded_middle_nodes_.clear();
    middle_nodes_.clear();
    use_time_node_num_ = 0;
    openSet_.clear();    
    double plan_start_vel = initial_vel_; //目前设定为0，之后需要改成生成新轨迹时的初始速度

    MiddleNodePtr start_node;
    start_node = middle_node_pool_[0];
    start_node->start_s = 0.0;
    start_node->end_s = 0.0;
    start_node->start_vel = plan_start_vel;
    start_node->end_vel = plan_start_vel;
    start_node->start_t = 0.0;
    start_node->end_t = 0.0;
    start_node->s_t_v_idx = Eigen::Vector3i(t2idx(0.0), s2idx(0.0), v2idx(plan_start_vel));
    start_node->length = 0.0;
    start_node->distance2end = total_s_;
    start_node->acc = 0.0;
    start_node->g_score = 0.0;
    start_node->f_score = 1 * calculateMiniTime2singulPoint(plan_start_vel, total_s_);
    start_node->node_state = IN_OPEN_SET;
    start_node->parent = NULL;
    use_time_node_num_++;

    openSet_.insert(std::make_pair(start_node->f_score, start_node));
    expanded_middle_nodes_.insert(start_node->s_t_v_idx, start_node);

    MiddleNodePtr cur_node = NULL;
    double max_input = max_acc_;
    //初始速度太快了，刹不住车
    if(plan_start_vel * plan_start_vel > 2 * max_input * total_s_)
    {
        max_input = plan_start_vel * plan_start_vel / (2 * total_s_) + 0.1;
        std::cout<<"max_acc_变大了"<<std::endl;
    }

    // for debugging //
    std::vector<MiddleNodePtr> debug_middle_nodes;
    debug_middle_nodes.push_back(start_node);
    //**************//
    std::cout << "配置结束进入循环"<< std::endl;
    while(!openSet_.empty())
    {
        static int i=0;
        i++;
        // std::cout << "配置结束进入循环"<< i << std::endl;
        int j=0;
        // for (const auto& element : openSet_) {
        //     j++;
        //     std::cout << "点"<< j << std::endl;
        //     std::cout << s2idx(element.second->end_s) << std::endl;
        //     std::cout << t2idx(element.second->end_t) << std::endl;
        //     std::cout << v2idx(element.second->end_vel) << std::endl;
        // }

        cur_node = openSet_.begin()->second; //读取openSet_中f_score最小的点指针
        openSet_.erase(openSet_.begin());
        cur_node->node_state = IN_CLOSE_SET;

        double pro_start_t = cur_node->end_t;
        double pro_start_s = cur_node->end_s;
        double pro_start_vel = cur_node->end_vel;       
        double pro_start2singuldistance = cur_node->distance2end;  

        /*---------one shot, which needs the end velcity and distance to singul point---------*/
        bool one_shot_success = true;
        double shot_duration = 0.0;  //one shot 花费的总时间
        std::vector<double> oneShot_trans_t;
        double d1 = (max_vel_ * max_vel_ - pro_start_vel * pro_start_vel) / (2 * max_input);
        double d2 = max_vel_ * max_vel_ / (2 * max_input);
        if(pro_start2singuldistance >= d1 + d2)
        {
            double t1 = (max_vel_ - pro_start_vel) / max_input;
            double t2 = (pro_start2singuldistance - d2 - d1) / max_vel_;
            double t3 = max_vel_ / max_input;
            shot_duration = t1 + t2 + t3;
            oneShot_trans_t.resize(3);
            oneShot_trans_t.at(0) = t1; oneShot_trans_t.at(1) = t1 + t2; oneShot_trans_t.at(2) = t1 + t2 + t3;
        }
        double d3 = pro_start_vel * pro_start_vel / (2 * max_input);
        if(pro_start2singuldistance >= d3 && pro_start2singuldistance < (d1 + d2))
        {
            double max_v = sqrt((2 * max_input * pro_start2singuldistance + pro_start_vel * pro_start_vel) / 2);
            double t1 = (max_v - pro_start_vel) / max_input;
            double t2 = max_v / max_input;
            shot_duration = t1 + t2;
            oneShot_trans_t.resize(2);
            oneShot_trans_t.at(0) = t1; oneShot_trans_t.at(1) = t1 + t2;
        }
        else if(pro_start2singuldistance < d3)
        {
            std::cerr<<"Too fast init speed, not possible to stop!"<<std::endl;
            continue;
            // return false;
        }

        for(double shot_check_t = time_resolution_; shot_check_t <= shot_duration + 1e-3; shot_check_t += time_resolution_)
        {
            double cur_t, cur_s; //遍历每个单位时间的达到距离
            cur_t = pro_start_t + shot_check_t;
            if(oneShot_trans_t.size() == 3)
            {
                if(shot_check_t < oneShot_trans_t.at(0))
                {
                    cur_s = pro_start_s + pro_start_vel * shot_check_t + 0.5 * max_input * shot_check_t * shot_check_t;
                }
                else if(shot_check_t >= oneShot_trans_t.at(0) && shot_check_t < oneShot_trans_t.at(1))
                {
                    double s1 = (max_vel_ * max_vel_ - pro_start_vel * pro_start_vel) / (2 * max_input);
                    double s2 = max_vel_ * (shot_check_t - oneShot_trans_t.at(0));
                    cur_s = pro_start_s + s1 + s2;
                }
                else
                {
                    double s1 = (max_vel_ * max_vel_ - pro_start_vel * pro_start_vel) / (2 * max_input);
                    double s2 = max_vel_ * (oneShot_trans_t.at(1) - oneShot_trans_t.at(0));
                    double s3 = max_vel_ * (shot_check_t - oneShot_trans_t.at(1)) - 0.5 * max_input * pow(shot_check_t - oneShot_trans_t.at(1), 2);
                    cur_s = pro_start_s + s1 + s2 + s3;
                }

            }
            else if(oneShot_trans_t.size() == 2)
            {
                if(shot_check_t < oneShot_trans_t.at(0))
                {
                    cur_s = pro_start_s + pro_start_vel * shot_check_t + 0.5 * max_input * shot_check_t * shot_check_t;
                }
                else
                {
                    double max_v = pro_start_vel + max_input * oneShot_trans_t.at(0);
                    double s1 = pro_start_vel * oneShot_trans_t.at(0) + 0.5 * max_input * pow(oneShot_trans_t.at(0), 2);
                    double s2 = max_v * (shot_check_t - oneShot_trans_t.at(0)) - 0.5 * max_input * pow(shot_check_t - oneShot_trans_t.at(0), 2);
                    cur_s = pro_start_s + s1 + s2;
                }
            }
            int cur_t_int = t2idx(cur_t);
            if(cur_t_int >= max_int_t)  //如果没有碰撞走完全程，说明可以直接one_shot
            {
                one_shot_success = true;
                break;
            }
            int cur_s_int = s2idx(cur_s);
            int cur_s_t_int = cur_t_int + cur_s_int * t_voxels_num;
            if(s_t_graph_vector.at(cur_s_t_int) > 0)
            {
                one_shot_success = false;
                std::cout << "oneshot失败,碰撞点" <<cur_t_int<<","<<cur_s_int<< std::endl;
                break;
            }              
        }
        if(one_shot_success)
        {
            double shot_start_vel = cur_node->end_vel;
            double shot_start_s = cur_node->end_s;
            double shot_start_t = cur_node->end_t;              
            
            std::cout << "The number of expanded nodes: " << debug_middle_nodes.size() << std::endl;
            std::cout << "speed planning success!!!!!!" << std::endl;

            middle_nodes_.clear();
            middle_nodes_.push_back(cur_node);
            while(cur_node->parent != NULL)
            {
                cur_node = cur_node->parent;
                middle_nodes_.push_back(cur_node);
            }
            std::reverse(middle_nodes_.begin(), middle_nodes_.end()); // middle_nodes_ include the first no distance node

            Find_s_t.clear();
            for (const auto& element : middle_nodes_) {
                Eigen::Vector2i find_s_t(s2idx(element->start_s),t2idx(element->start_t));
                Find_s_t.push_back(find_s_t);
            }
            
            
            for(double t_bar=shot_start_t;t_bar<shot_start_t+shot_duration+1e-3;t_bar+=time_resolution_){
                double shot_start_t = middle_nodes_.back()->end_t;
                double shot_start_s = middle_nodes_.back()->end_s;
                double shot_start_v = middle_nodes_.back()->end_vel;
                double shot_length = total_s_ - shot_start_s;
                double cur_s;
                double shot_t_bar = t_bar - shot_start_t;
                double d1 = (max_vel_ * max_vel_ - shot_start_v * shot_start_v) / (2 * max_acc_);
                double d2 = max_vel_ * max_vel_ / (2 * max_acc_);
                double d3 = shot_start_v * shot_start_v / (2 * max_acc_);
                if(shot_length > (d1 + d2))
                {
                    double t1 = (max_vel_ - shot_start_v) / max_acc_;
                    double t2 = (shot_length - d2 - d1) / max_vel_;
                    double t3 = max_vel_ / max_acc_;
                    if(shot_t_bar < t1)
                    {
                        cur_s = shot_start_s + shot_start_v * shot_t_bar + 0.5 * max_acc_ * pow(shot_t_bar, 2);
                    }
                    else if(shot_t_bar >= t1 && shot_t_bar < (t1 + t2))
                    {
                        cur_s = shot_start_s + d1 + max_vel_ * (shot_t_bar - t1);
                    }
                    else
                    {
                        cur_s = shot_start_s + d1 + max_vel_ * t2 + max_vel_ * (shot_t_bar - t1 - t2) - 0.5 * max_acc_ * pow(shot_t_bar - t1 - t2, 2);
                    }
                }
                else if(shot_length <= d1 + d2 && shot_length > (d3 - 1e-3))
                {
                    double max_v = sqrt((2 * max_acc_ * shot_length + shot_start_v * shot_start_v) / 2);
                    double t1 = (max_v - shot_start_v) / max_acc_;
                    double t2 = max_v / max_acc_;
                    if(shot_t_bar < t1)
                    {
                        cur_s = shot_start_s + shot_start_v * shot_t_bar + 0.5 * max_acc_ * shot_t_bar * shot_t_bar;
                    }
                    else
                    {
                        cur_s = shot_start_s + (max_v * max_v - shot_start_v * shot_start_v) / (2 * max_acc_) + max_v * (shot_t_bar - t1) - 0.5 * max_acc_ * pow(shot_t_bar - t1, 2);
                    }
                }
                else
                {
                    // ROS_ERROR("Wrong! Cannot calculate the init pos!");
                    double max_input = shot_start_v * shot_start_v / (2 * shot_length);
                    cur_s = shot_start_s + shot_start_v * shot_t_bar - 0.5 * max_input * pow(shot_t_bar, 2);
                }
                Eigen::Vector2i find_s_t(s2idx(cur_s),t2idx(t_bar));
                Find_s_t.push_back(find_s_t);
            }
            std::cout<<"总用时 "<<shot_start_t+shot_duration<<std::endl;
            flat_trajs.clear();

            Eigen::Vector2d last_last_pos = data.trajectory[car_id_].at(end_index - 1);
            Eigen::Vector2d last_pos = data.trajectory[car_id_].at(end_index);
            Eigen::Vector4d segment_start_state, segment_end_state;
            segment_start_state << data.trajectory[car_id_][0],data.yaw[car_id_][0],initial_vel_;

            //middle_nodes_.back()->end_t是前面搜索的点，不能用one_shot直接生成的点
            double segment_duration = shot_start_t + shot_duration;
            double sampletime = 0.02;
            if(sampletime > segment_duration)
            {
                sampletime = segment_duration / 2.0;
                std::cout << "太小了!!!!!!!!!!!!!!!!" << std::endl;
            }
            std::vector<Eigen::Vector3d> traj_pts;
            std::vector<double> thetas;
            double samplet;
            TrajectoryData newdata;
            for(samplet = sampletime; samplet < segment_duration; samplet += sampletime)
            {
                double roundedTime = roundToNearestMultiple(samplet,time_resolution_);
                Eigen::Vector3d round_pos = CalculateInitPos(roundedTime, start_direction,data); 
                Eigen::Vector2d point=round_pos.head(2);
                newdata.trajectory[car_id_].push_back(point);
                newdata.timestamp[car_id_].push_back(roundedTime+data.timestamp[car_id_].at(0)-sampletime);
                newdata.yaw[car_id_].push_back(round_pos(2));
                
                // std::cout<<"roundedTime"<<roundedTime+data.timestamp[car_id_].at(0)-sampletime<<std::endl;
                // std::cout<<"point"<<point<<std::endl;
                // std::cout<<"yaw"<<round_pos(2)<<std::endl;
                // char ch;
                // std::cout << "Press 'q' and then Enter to continue: ";
                // std::cin >> ch;

                // while (ch != 'q') {
                //     std::cout << "You pressed '" << ch << "'. Press 'q' and then Enter to continue: ";
                //     std::cin >> ch;
                // }

                Eigen::Vector3d sample_pos = CalculateInitPos(samplet, start_direction,data);
                Eigen::Vector2d traj_pt_pos = sample_pos.head(2);
                Eigen::Vector3d traj_pt; traj_pt << traj_pt_pos, sampletime;
                traj_pts.push_back(traj_pt);
                thetas.push_back(sample_pos(2));
            }
            // Eigen::Vector2d last_last_pos = data.trajectory[car_id_].at(end_index - 1);
            // Eigen::Vector2d last_pos = data.trajectory[car_id_].at(end_index);
            Eigen::Vector2d last_direction = last_pos - last_last_pos;
            double segment_end_yaw = atan2(start_direction * last_direction(1), start_direction * last_direction(0));
            double segment_end_vel = start_direction > 0 ? non_siguav : -non_siguav;
            traj_pts.push_back(Eigen::Vector3d(last_pos(0), last_pos(1), segment_duration - samplet));
            thetas.push_back(normalize_angle(segment_end_yaw));

            double roundedTime = roundToNearestMultiple(samplet,time_resolution_);
            newdata.trajectory[car_id_].push_back(Eigen::Vector2d(last_pos(0), last_pos(1)));
            newdata.timestamp[car_id_].push_back(roundedTime+data.timestamp[car_id_].at(0)-sampletime);
            newdata.yaw[car_id_].push_back(normalize_angle(segment_end_yaw));  

            data.trajectory[car_id_].clear();
            data.timestamp[car_id_].clear();
            data.yaw[car_id_].clear();
            data.trajectory[car_id_]=newdata.trajectory[car_id_];
            data.timestamp[car_id_]=newdata.timestamp[car_id_];
            data.yaw[car_id_]=newdata.yaw[car_id_];
            std::cout<<"更新轨迹结束"<<std::endl;

            // Eigen::Vector4d segment_start_state, segment_end_state;
            // segment_start_state << data.trajectory[car_id_][0],data.yaw[car_id_][0],initial_vel_;
            segment_end_state << last_pos, segment_end_yaw, segment_end_vel;
            FlatTrajData flat_traj;
            Eigen::MatrixXd startS, endS;
            Eigen::Vector2d Initctrlinput(0, 0), Finctrlinput(0, 0);
            // Initctrlinput = start_ctrl_;
            getFlatState(segment_start_state, Initctrlinput, startS, start_direction);
            getFlatState(segment_end_state, Finctrlinput, endS, start_direction);   
            flat_traj.traj_pts = traj_pts;
            flat_traj.thetas = thetas;
            flat_traj.start_state = startS;
            flat_traj.final_state = endS;
            flat_traj.singul = start_direction;
            flat_trajs.push_back(flat_traj); 
            break;    

        }
        /*-----------------------------------------------------------------*/

        std::vector<double> ctrl_inputs, durations;
        double res = 1.0;
        for(double input = -max_input; input <= max_input + 1e-3; input += res * max_input)
        {
            ctrl_inputs.push_back(input);
        }
        durations.push_back(2 * time_resolution_);
        durations.push_back(4 * time_resolution_);
        durations.push_back(8 * time_resolution_);
        // for(double tau = 2 * time_resolution_; tau <= 3 * 2 * time_resolution_ + 1e-3; tau += 2 * time_resolution_)
        // {
        //     durations.push_back(tau);
        // }

        std::vector<MiddleNodePtr> expanded_nodes_from_one_node;
        expanded_nodes_from_one_node.clear();
        for(int i = 0; i < ctrl_inputs.size(); i++)
        {
            for(int j = 0; j < durations.size(); j++)
            {
                double input = ctrl_inputs[i];
                double duration = durations[j];

                double pro_end_t = pro_start_t + duration;
                double pro_end_s = pro_start_s + pro_start_vel * duration + 0.5 * input * duration * duration;
                double pro_end_vel = pro_start_vel + input * duration;
                double pro_end2singuldistance = total_s_ - pro_end_s;
                Eigen::Vector3i pro_end_stv_idx(t2idx(pro_end_t), s2idx(pro_end_s), v2idx(pro_end_vel));
                
                if(pro_end_vel > max_vel_ || pro_end_vel < non_siguav)
                    continue;
                if(pro_end_vel * pro_end_vel / (2 * max_input) > (pro_end2singuldistance - 1e-3))
                    continue;
                if(pro_end_s > total_s_ || pro_end_s < pro_start_s)
                    continue;

                MiddleNodePtr pro_node = expanded_middle_nodes_.find(pro_end_stv_idx);
                if(pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
                    continue;

                // check collision and dynamics
                bool collision = false;
                for(double check_t = pro_end_t - pro_start_t; check_t >= 1e-3; check_t -= time_resolution_)
                {
                    double check_s = pro_start_s + pro_start_vel * check_t + 0.5 * input * check_t * check_t;
                    if(check_s > total_s_ || check_s < pro_start_s)
                    {
                        collision = true;
                        if (input>=0)
                        {
                            std::cout << "差1"<<pro_start_vel * check_t ;
                            std::cout << "差2"<<0.5 * input * check_t * check_t;
                            std::cout << "差"<<pro_start_vel * check_t + 0.5 * input * check_t * check_t;
                            std::cout << "pro_start_vel"<<pro_start_vel<<std::endl;
                            if(check_s > total_s_ )
                            std::cout << "生成的新点太大了"<<t2idx(pro_end_t)<<","<<s2idx(pro_end_s)<< std::endl;
                            else 
                            std::cout << "生成的新点太小了"<<t2idx(pro_end_t)<<","<<s2idx(pro_end_s)<< std::endl;
                        }

                        break;
                    }
                    // no need to check velocity, only check end velocity is enough. But it's necessary to check s.
                    // if t is beyond the max t, then no need to check any more, because there will not be collision
                    int check_int_t = t2idx(check_t + pro_start_t);
                    if(check_int_t >= max_int_t)
                        continue;
            
                    int check_int_s = s2idx(check_s);
                    int check_s_t_idx = check_int_t + t_voxels_num * check_int_s;

                    if(s_t_graph_vector.at(check_s_t_idx) > 0)
                    {
                        collision = true;
                        std::cout << "生成的新点是碰撞点"<<t2idx(pro_end_t)<<","<<s2idx(pro_end_s)<< std::endl;
                        break;
                    }
                        
                }
                if(collision)
                    break;

                double tmp_g_score = pro_end_t;
                double tmp_h_score = 1 * calculateMiniTime2singulPoint(pro_end_vel, pro_end2singuldistance);
                double tmp_f_score = tmp_g_score + tmp_h_score;
                bool prune = false;
                for(int k = 0; k < expanded_nodes_from_one_node.size(); k++)
                {
                    MiddleNodePtr node_from_cur_node = expanded_nodes_from_one_node.at(k);
                    if(pro_end_stv_idx == node_from_cur_node->s_t_v_idx)
                    {
                        prune = true;
                        if(node_from_cur_node->f_score > tmp_f_score)
                        {
                            node_from_cur_node->end_s = pro_end_s;
                            node_from_cur_node->end_t = pro_end_t;
                            node_from_cur_node->end_vel = pro_end_vel;
                            node_from_cur_node->length = pro_start_vel * duration + 0.5 * input * duration * duration;
                            node_from_cur_node->distance2end = pro_end2singuldistance;
                            node_from_cur_node->acc = input;
                            node_from_cur_node->g_score = tmp_g_score;
                            node_from_cur_node->f_score = tmp_f_score;
                        }  
                        break;
                    }
                }

                if(!prune)
                {
                    if(pro_node == NULL)
                    {
                        pro_node = middle_node_pool_[use_time_node_num_];
                        use_time_node_num_++;
                        pro_node->start_s = pro_start_s;
                        pro_node->start_t = pro_start_t;
                        pro_node->start_vel = pro_start_vel;
                        pro_node->end_s = pro_end_s;
                        pro_node->end_t = pro_end_t;
                        pro_node->end_vel = pro_end_vel;
                        pro_node->s_t_v_idx = pro_end_stv_idx;
                        pro_node->length = pro_start_vel * duration + 0.5 * input * duration * duration;
                        pro_node->distance2end = total_s_ - pro_end_s;
                        pro_node->acc = input;
                        pro_node->g_score = tmp_g_score;
                        pro_node->f_score = tmp_f_score;
                        pro_node->node_state = IN_OPEN_SET;
                        pro_node->parent = cur_node;

                        if(use_time_node_num_ > 20000)
                        {
                            std::cerr<<"Too many nodes! Middle node search fails!"<<std::endl;
                            // return false;
                        }

                        debug_middle_nodes.push_back(pro_node);

                        openSet_.insert(std::make_pair(pro_node->f_score, pro_node));
                        expanded_middle_nodes_.insert(pro_node->s_t_v_idx, pro_node);
                        expanded_nodes_from_one_node.push_back(pro_node);
                    }
                    else if(pro_node->node_state == IN_OPEN_SET && pro_node->f_score > tmp_f_score)
                    {
                        pro_node->start_s = pro_start_s;
                        pro_node->start_t = pro_start_t;
                        pro_node->start_vel = pro_start_vel;
                        pro_node->end_s = pro_end_s;
                        pro_node->end_t = pro_end_t;
                        pro_node->end_vel = pro_end_vel;
                        pro_node->length = pro_start_vel * duration + 0.5 * input * duration * duration;
                        pro_node->distance2end = total_s_ - pro_end_s;
                        pro_node->acc = input;
                        pro_node->g_score = tmp_g_score;
                        pro_node->f_score = tmp_f_score;
                        pro_node->parent = cur_node;
                    }
                }
            }
        }    
    }
    std::cerr<<"MiddleNode open set empty! Search fails!"<<std::endl;
    std::cout << "Number of expanded nodes: " << debug_middle_nodes.size() << std::endl;
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

//计算到指定距离所花费的时间
//第一种情况：加速到max_vel后保持匀速，在减速到0；
//第二种情况：先加速后减速到0；
//第三种情况：直接加速到0；
double calculateMiniTime2singulPoint(double initspeed, double distance2end)
{
    double d1 = (max_vel_ * max_vel_ - initspeed * initspeed) / (2 * max_acc_) + max_vel_ * max_vel_ / (2 * max_acc_);
    double d2 = distance2end - d1;
    if(d2 > 0)
    {
        double t1 = (max_vel_ - initspeed) / max_acc_;
        double t2 = d2 / max_vel_;
        double t3 = max_vel_ / max_acc_;
        return t1 + t2 + t3;
    }

    double d3 = initspeed * initspeed / (2 * max_acc_);
    if(distance2end >= d3)
    {
        double vmax = sqrt((2 * max_acc_ * distance2end + initspeed * initspeed) / 2.0);
        double t1 = (vmax - initspeed) / max_acc_;
        double t2 = vmax / max_acc_;
        return t1 + t2;
    }
    else
    {
        return 2 * distance2end / initspeed;
    }
}

Eigen::Vector3d CalculateInitPos(double& t, int& singul,TrajectoryData data)
{
    double t_bar = t;
    double shot_start_t = middle_nodes_.back()->end_t;
    double shot_start_s = middle_nodes_.back()->end_s;
    double shot_start_v = middle_nodes_.back()->end_vel;
    double shot_length = total_s_ - shot_start_s;
    double cur_s;
    if(t_bar < shot_start_t - 1e-3)
    {
        for(MiddleNodePtr middle_node : middle_nodes_)
        {
            if(t_bar >= (middle_node->start_t - 1e-5) && t_bar < middle_node->end_t)
            {
                double node_t_bar = t_bar - middle_node->start_t;
                double node_start_v = middle_node->start_vel;
                double node_acc = middle_node->acc;
                cur_s = middle_node->start_s + node_start_v * node_t_bar + 0.5 * node_acc * pow(node_t_bar, 2);
                break;
            }
        }
    }
    else
    {
        double shot_t_bar = t_bar - shot_start_t;
        double d1 = (max_vel_ * max_vel_ - shot_start_v * shot_start_v) / (2 * max_acc_);
        double d2 = max_vel_ * max_vel_ / (2 * max_acc_);
        double d3 = shot_start_v * shot_start_v / (2 * max_acc_);
        if(shot_length > (d1 + d2))
        {
            double t1 = (max_vel_ - shot_start_v) / max_acc_;
            double t2 = (shot_length - d2 - d1) / max_vel_;
            double t3 = max_vel_ / max_acc_;
            if(shot_t_bar < t1)
            {
                cur_s = shot_start_s + shot_start_v * shot_t_bar + 0.5 * max_acc_ * pow(shot_t_bar, 2);
            }
            else if(shot_t_bar >= t1 && shot_t_bar < (t1 + t2))
            {
                cur_s = shot_start_s + d1 + max_vel_ * (shot_t_bar - t1);
            }
            else
            {
                cur_s = shot_start_s + d1 + max_vel_ * t2 + max_vel_ * (shot_t_bar - t1 - t2) - 0.5 * max_acc_ * pow(shot_t_bar - t1 - t2, 2);
            }
        }
        else if(shot_length <= d1 + d2 && shot_length > (d3 - 1e-3))
        {
            double max_v = sqrt((2 * max_acc_ * shot_length + shot_start_v * shot_start_v) / 2);
            double t1 = (max_v - shot_start_v) / max_acc_;
            double t2 = max_v / max_acc_;
            if(shot_t_bar < t1)
            {
                cur_s = shot_start_s + shot_start_v * shot_t_bar + 0.5 * max_acc_ * shot_t_bar * shot_t_bar;
            }
            else
            {
                cur_s = shot_start_s + (max_v * max_v - shot_start_v * shot_start_v) / (2 * max_acc_) + max_v * (shot_t_bar - t1) - 0.5 * max_acc_ * pow(shot_t_bar - t1, 2);
            }
        }
        else
        {
            // ROS_ERROR("Wrong! Cannot calculate the init pos!");
            double max_input = shot_start_v * shot_start_v / (2 * shot_length);
            cur_s = shot_start_s + shot_start_v * shot_t_bar - 0.5 * max_input * pow(shot_t_bar, 2);
        }
    }
    double path_distance = 0.0;
    Eigen::Vector2d path_node_start_pos, path_node_end_pos;
    for(int i = 0; i < data.trajectory[car_id_].size() - 1; i++)
    {
        double cur_path_distance = (data.trajectory[car_id_].at(i+1) - data.trajectory[car_id_].at(i)).norm();
        if(cur_s >= path_distance - 1e-3 && cur_s < path_distance + cur_path_distance)
        {
            path_node_start_pos = data.trajectory[car_id_].at(i);
            path_node_end_pos = data.trajectory[car_id_].at(i+1);
            break;
        }
        path_distance += cur_path_distance;
    }
    double s_bar = cur_s - path_distance;
    Eigen::Vector2d path_node_direction = path_node_end_pos - path_node_start_pos;
    double path_node_dis = path_node_direction.norm();
    Eigen::Vector2d sample_pos = path_node_start_pos + s_bar / path_node_dis * path_node_direction;
    double sample_yaw = atan2(singul * path_node_direction(1), singul * path_node_direction(0));
    
    return Eigen::Vector3d(sample_pos(0), sample_pos(1), normalize_angle(sample_yaw));
} 

/*--------------speed planning-----------------*/
int s2idx(double s)
{
    return floor((s + 1e-3) / distance_resolution_);
}

int t2idx(double t)
{
    return floor((t + 1e-3) / time_resolution_);
}

int v2idx(double v)
{
    return floor(v / velocity_resolution_);
}

void getFlatState(Eigen::Vector4d state, Eigen::Vector2d control_input,
                                  Eigen::MatrixXd &flat_state, int singul)
{

    flat_state.resize(2, 3);

    double angle = state(2);
    double vel   = state(3); // vel > 0 

    Eigen::Matrix2d init_R;
    init_R << cos(angle),  -sin(angle),
            sin(angle),   cos(angle);

    // if (abs(vel) <= non_siguav){
    //   vel = singul * non_siguav;
    // }
    // else{
    //   vel = singul * vel;
    // }
    
    flat_state << state.head(2), init_R*Eigen::Vector2d(vel, 0.0), 
                init_R*Eigen::Vector2d(control_input(1), std::tan(control_input(0)) / car_wheelbase_ * std::pow(vel, 2));

}

double roundToNearestMultiple(double value, double multiple) {
    return std::round(value / multiple) * multiple;
}


int main() {
    //常量设置
    car_d_cr_ = -0.04; //雷达到中心点位置(前置为负数，后置为正数)
    distance_resolution_ = 0.01; //离散单位位置
    car_width_=0.36; 
    cars_num_ =3;
    // car_id_=2; 
    time_resolution_ = 0.02; //离散单位时间
    time_samples_= 500; 
    velocity_resolution_ = 0.5;
    car_wheelbase_ = 0.26;
    allocate_num_ = 100000;
    /*------------------需要修改---------------------*/
    initial_vel_ = 0;
    max_vel_ = 1.8;
    max_acc_ = 1.8;
    non_siguav = 0.05;

    //时间异步（以画st图的轨迹为参考时间，影响上下移动）
    // bias1_=0.16;
    // bias2_=0.54;
    // bias3_=0.2;
    bias1_=0.16;
    bias2_=0.4;
    bias3_=0.2;    

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
    KinoTrajData flat_trajs;
    std::vector<Eigen::Vector2i> Find_s_t;

    for(int i=0;i<cars_num_;i++){
        car_id_=i;
        auto start = std::chrono::high_resolution_clock::now();
        graph = searchTime(data,data.timestamp[car_id_][0],flat_trajs,Find_s_t);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "经过时间: " << elapsed.count() << " 秒" << std::endl;
        
        // char ch;
        // std::cout << "Press 'q' and then Enter to continue: ";
        // std::cin >> ch;

        // while (ch != 'q') {
        //     std::cout << "You pressed '" << ch << "'. Press 'q' and then Enter to continue: ";
        //     std::cin >> ch;
        // }


        std::ostringstream filename1,filename2;
        filename1 << "s_t_graph" << i << ".txt";
        filename2 << "Find_s_t" << i << ".txt";
        std::string currentFilename1 = filename1.str();
        std::string currentFilename2 = filename2.str();
        std::cout << "当前文件名1: " << currentFilename1 << std::endl;
        std::cout << "当前文件名2: " << currentFilename2 << std::endl;
        writeGraphToFile(graph,currentFilename1);
        writevectorToFile(Find_s_t,currentFilename2);
    }

    saveToCSV(data, 0, "newtrajectory0.csv");
    saveToCSV(data, 1, "newtrajectory1.csv");
    saveToCSV(data, 2, "newtrajectory2.csv");

    // std::string filename="s_t_graph.txt";
    // writeGraphToFile(graph,filename);
    // writevectorToFile(Find_s_t,"Find_s_t.txt");
    
    return 0;
}


    // char ch;
    // std::cout << "Press 'q' and then Enter to continue: ";
    // std::cin >> ch;

    // while (ch != 'q') {
    //     std::cout << "You pressed '" << ch << "'. Press 'q' and then Enter to continue: ";
    //     std::cin >> ch;
    // }