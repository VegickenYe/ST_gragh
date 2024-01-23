#ifndef _SAVE_TRAJ_H
#define _SAVE_TRAJ_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <set>
#include <unordered_map>
#include <map>
#include <eigen3/Eigen/Dense>

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

double initial_yaw1;
double car_d_cr_,distance_resolution_,car_width_,time_resolution_,velocity_resolution_,trajectory2_x_,trajectory2_y_,trajectory3_x_,trajectory3_y_,car_wheelbase_;
int cars_num_,car_id_,time_samples_,allocate_num_;
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

struct FlatTrajData
{
    int singul;
    std::vector<Eigen::Vector3d> traj_pts;      // 3, N  x,y dur
    std::vector<double> thetas;
    Eigen::MatrixXd start_state;   // start flat state (2, 3)
    Eigen::MatrixXd final_state;   // end flat state (2, 3)

};
typedef std::vector<FlatTrajData> KinoTrajData;

class MiddleNode{
public:
    double start_s;
    double start_t;
    double start_vel;
    double end_s;
    double end_t;
    double end_vel;
    Eigen::Vector3i s_t_v_idx;
    double length;
    double distance2end;
    double acc;
    double g_score, f_score; //f_score是到指定距离的花费时间
    char node_state;

    MiddleNode* parent;

    MiddleNode()
    {
        parent = NULL;
        node_state = NOT_EXPAND;
    }
    ~MiddleNode(){};
};
typedef MiddleNode* MiddleNodePtr;

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (long int i = 0; i < matrix.size(); ++i) {
        auto elem = *(matrix.data() + i);
        seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
    }
};

class MiddleNodeHashTable {
private:
    /* data */
    std::unordered_map<Eigen::Vector3i, MiddleNodePtr, matrix_hash<Eigen::Vector3i>> middle_data_3d_;

public:
    MiddleNodeHashTable(/* args */) {
    }
    ~MiddleNodeHashTable() {
    }

    // For middle node searching
    void insert(Eigen::Vector3i idx, MiddleNodePtr node)
    {
    middle_data_3d_.insert(std::make_pair(idx, node));
    }

    MiddleNodePtr find(Eigen::Vector3i idx)
    {
        auto iter = middle_data_3d_.find(idx);
        return iter == middle_data_3d_.end() ? NULL : iter->second;
    }

    void clear() {
    middle_data_3d_.clear();
    }
};  

/*--------------speed planning-----------------*/
MiddleNodeHashTable expanded_middle_nodes_;
// double calculateMiniTime2singulPoint(double initspeed, double distance2end);
int s2idx(double s);
int t2idx(double t);
int v2idx(double v);

std::vector<MiddleNodePtr> middle_node_pool_;
std::vector<MiddleNodePtr> middle_nodes_;
int use_time_node_num_;
double total_s_;
std::multimap<double, MiddleNodePtr> openSet_;  //openSet_按照第一个元素double类型排列，升序；
// double gettimeHeu(int& idx, double& vel);
// double calculateCurNode2nearestSingul(int idx);
/*------------------------------------------*/

/*------------------需要修改---------------------*/
double initial_vel_;
double max_vel_;
double max_acc_;
double non_siguav;
double calculateMiniTime2singulPoint(double initspeed, double distance2end);
Eigen::Vector3d CalculateInitPos(double& t, int& singul,TrajectoryData data);
void getFlatState(Eigen::Vector4d state, Eigen::Vector2d control_input,Eigen::MatrixXd &flat_state, int singul);
double roundToNearestMultiple(double value, double multiple);
#endif