#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "hybrid_astar/occupancy.h"

class GridConverter {
public:
    GridConverter() {
        // 初始化发布者和订阅者
        pub_ = nh_.advertise<hybrid_astar::occupancy>("/extended_map", 10);
        sub_ = nh_.subscribe("/map", 10, &GridConverter::mapCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        hybrid_astar::occupancy new_msg;
        new_msg.header = msg->header;
        new_msg.info = msg->info;
        new_msg.data.resize(msg->data.size());

        // 转换数据
        for (size_t i = 0; i < msg->data.size(); ++i) {
                new_msg.data[i] = static_cast<uint8_t>(msg->data[i]);
        }

        // 发布转换后的消息
        pub_.publish(new_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "grid_converter");
    GridConverter converter;
    ros::spin();
    return 0;
}
