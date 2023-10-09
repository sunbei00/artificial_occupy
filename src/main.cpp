#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <map>
#include <chrono>
#include <algorithm>
#include "blockPos.hpp"


class ArtificialOccupy : public rclcpp::Node {
public:
	ArtificialOccupy() : Node("artificial_occupy") {
		rmw_qos_profile_t qos_profile = rmw_qos_profile_default;  
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
		publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_ao", qos);

		auto callback = [this](const std::shared_ptr<rmw_request_id_t> request_header,
						const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
						const std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
						{
							isRun = (*request).data;
							if(isRun)
								RCLCPP_INFO(this->get_logger(), "ArtificialOccupy : ON");	
							else
								RCLCPP_INFO(this->get_logger(), "ArtificialOccupy : OFF");
							(*response).success = true;
							(*response).message = isRun ? "ArtificialOccupy : ON" : "ArtificialOccupy : OFF";
							(*service_).send_response(*request_header, *response);
						};

		service_ = this->create_service<std_srvs::srv::SetBool>("ON_OFF", callback);
		
		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ArtificialOccupy::odomCallback, this, std::placeholders::_1));

		state_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/state", 10, std::bind(&ArtificialOccupy::stateCallback, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(), "ArtificialOccupy : ON");
		ServiceBlockPos.emplace_back(9.0625, -2.25);
		ServiceBlockPos.emplace_back(436.125, -100);
		input(blockPos);
		input2(blockPos2);
	}


private:
	geometry_msgs::msg::Vector3 quaternionToEuler(const geometry_msgs::msg::Quaternion& q) {
		geometry_msgs::msg::Vector3 euler;

		double t0 = +2.0 * (q.w * q.x + q.y * q.z);
		double t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
		euler.x = std::atan2(t0, t1);

		double t2 = +2.0 * (q.w * q.y - q.z * q.x);
		t2 = t2 > 1.0 ? 1.0 : t2;
		t2 = t2 < -1.0 ? -1.0 : t2;
		euler.y = std::asin(t2);

		double t3 = +2.0 * (q.w * q.z + q.x * q.y);
		double t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
		euler.z = std::atan2(t3, t4);

		return euler;
	}

	void stateCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
		auto& from = msg->data[0];
		auto& to = msg->data[1];
		if((from < 10 && to < 10) || (from >= 10 || to >= 10))
			isRun = true; 
		else
			isRun = false;
		
	}

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
		static auto lastCallTime =  std::chrono::steady_clock::now();
		auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastCallTime);

        if (elapsedTime.count() < 100) 
            return;
        lastCallTime = currentTime;

		double direction = quaternionToEuler(msg->pose.pose.orientation).z;
		double& x = msg->pose.pose.position.x;
		double& y = msg->pose.pose.position.y;
		

		auto pubMsg = sensor_msgs::msg::LaserScan();
		pubMsg.header.frame_id = "velodyne";
		pubMsg.header.stamp = msg->header.stamp;
		pubMsg.angle_min = -3.1415927410125732;
		pubMsg.angle_max = 3.1415927410125732;
		pubMsg.angle_increment = 0.007000000216066837;
		pubMsg.scan_time = 0;
		pubMsg.range_min = 0;
		pubMsg.range_max = 200.0;
		
		// int num_data_points = static_cast<int>((pubMsg.angle_max - pubMsg.angle_min) / pubMsg.angle_increment) + 1;
		// printf("%d\n",num_data_points);
		// 898
		static std::vector<float> intensities(898, 0);
		static std::vector<float> ranges(898, 199);
		pubMsg.intensities = intensities;
		pubMsg.ranges = ranges;

		
		if(isRun){
			for(auto& it : ServiceBlockPos){
				float xd = it.first - x;
				float yd = it.second - y;
				float distance = xd*xd + yd*yd;
				if(distance - 200 > 0)
					continue;

				if(abs(xd) < 0.000001f)
					xd = 0.000001f;

				float angle = std::atan2(yd, xd) - direction;
				if(angle < -3.1415927410125732)
					angle += 2*3.1415927410125732;
				if(angle > 3.1415927410125732)
					angle -= 2*3.1415927410125732;
				
				int idx = static_cast<int>((angle + 3.1415927410125732) / pubMsg.angle_increment);

				for(int i=0;i<=0;i++){
					int index = idx + i;
					if(index < 0)
						index = index + 898;
					if(index >= 898)
						index = index - 898;
					pubMsg.intensities[index] = 250.f;
					pubMsg.ranges[index] = sqrt(distance);
				}
			}
		} 

		for(auto& it : blockPos){
			float xd = it.first - x;
			float yd = it.second - y;
			float distance = xd*xd + yd*yd;
			if(distance - 5000 > 0)
				continue;

			if(abs(xd) < 0.000001f)
				xd = 0.000001f;

			float angle = std::atan2(yd, xd) - direction;
			if(angle < -3.1415927410125732)
				angle += 2*3.1415927410125732;
			if(angle > 3.1415927410125732)
				angle -= 2*3.1415927410125732;
			
			int idx = static_cast<int>((angle + 3.1415927410125732) / pubMsg.angle_increment);

			for(int i=-1;i<=1;i++){
				int index = idx + i;
				if(index < 0)
					index = index + 898;
				if(index >= 898)
					index = index - 898;
				pubMsg.intensities[index] = 250.f;
				pubMsg.ranges[index] = sqrt(distance);
			}
		}
		for(auto& it : blockPos2){
			float xd = it.first - x;
			float yd = it.second - y;
			float distance = xd*xd + yd*yd;
			if(distance - 5000 > 0)
				continue;

			if(abs(xd) < 0.000001f)
				xd = 0.000001f;

			float angle = std::atan2(yd, xd) - direction;
			if(angle < -3.1415927410125732)
				angle += 2*3.1415927410125732;
			if(angle > 3.1415927410125732)
				angle -= 2*3.1415927410125732;
			
			int idx = static_cast<int>((angle + 3.1415927410125732) / pubMsg.angle_increment);

			for(int i=-10;i<=10;i++){
				int index = idx + i;
				if(index < 0)
					index = index + 898;
				if(index >= 898)
					index = index - 898;
				pubMsg.intensities[index] = 250.f;
				pubMsg.ranges[index] = sqrt(distance)- 0.05;
			}
		}
		
	
		(*publisher_).publish(pubMsg);
		
    }

private:
	std::vector<std::pair<float,float>> blockPos;
	std::vector<std::pair<float,float>> blockPos2;
	std::vector<std::pair<float,float>> ServiceBlockPos;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr  publisher_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr state_sub_;
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
	
	bool isRun = true;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc,argv);
	
	rclcpp::spin(std::make_shared<ArtificialOccupy>());
	rclcpp::shutdown();

	return 0;
}