#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>

struct Point {
    double x, y, yaw;
};

class Vehicle {
public:
    Vehicle(double length) : length(length), x(0), y(0), yaw(0), v(0), steering_angle(0) {}

    void update(double dt) {
        // 更新位置和方向
        x += v * cos(yaw) * dt;
        y += v * sin(yaw) * dt;
        yaw += v / length * tan(steering_angle) * dt;
    }

    void randomize_motion(double max_speed, double max_steering_angle) {
        // 随机速度和转向角
        v = (double(rand()) / RAND_MAX) * max_speed;
        steering_angle = (double(rand()) / RAND_MAX - 0.5) * 2 * max_steering_angle;
    }

    Point get_position() {
        return {x, y, yaw};
    }

private:
    double length;  // 车辆长度
    double x, y;    // 位置
    double yaw;     // 方向角
    double v;       // 速度
    double steering_angle;  // 转向角
};

int main() {
    srand(time(0));  // 初始化随机数生成器

    const double dt = 0.02;  // 时间间隔
    const int samples = 500; // 采样次数
    const double max_speed = 1.0; // 最大速度
    const double max_steering_angle = M_PI / 4; // 最大转向角

    Vehicle car(0.3); // 车辆对象，长度为0.3米

    std::ofstream file("trajectory2.csv");

    for (int i = 0; i < samples; ++i) {
        car.randomize_motion(max_speed, max_steering_angle);
        car.update(dt);
        Point p = car.get_position();
        file << p.x << ", " << p.y << ", " << p.yaw << std::endl;
    }

    file.close();

    std::cout << "Trajectory data saved to trajectory.csv" << std::endl;

    return 0;
}
