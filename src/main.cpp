#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <map>
#include <chrono>
#include <algorithm>


class ArtificialOccupy : public rclcpp::Node {
public:
	ArtificialOccupy() : Node("artificial_occupy") {
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
		publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/ao", qos);

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
		
		subscrition_ = this->create_subscription<std_msgs::msg::Float32MultiArray>
			("/utm_coordinates", 10, std::bind(&ArtificialOccupy::callback, this, std::placeholders::_1));

		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ArtificialOccupy::odomCallback, this, std::placeholders::_1));
    

		RCLCPP_INFO(this->get_logger(), "ArtificialOccupy : ON");	
		
		blockPos.emplace_back(334215.28125, 4143040.5);

	}


private:
	void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		if(!isRun)
			return;

		static auto lastExecutionTime = std::chrono::high_resolution_clock::now();
		auto currentTime = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastExecutionTime).count();
		if(duration < 100)
			return;

		auto& inst = *msg;
		float& x = inst.data[0];
		float& y = inst.data[1];
		

		auto pubMsg = sensor_msgs::msg::LaserScan();
		pubMsg.header.frame_id = "velodyne";
		pubMsg.header.stamp = rclcpp::Clock().now();
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

		for(auto& it : blockPos){
			float xd = it.first - x;
			float yd = it.second - y;
			float distance = xd*xd + yd*yd;
			if(distance - 200 > 0)
				continue;
			pubMsg.intensities = intensities;
			pubMsg.ranges = ranges;
			if(xd < 0.000001f)
				xd = 0.000001f;

			float angle = std::atan2(yd, xd) - direction;
			if(angle < -3.1415927410125732)
				angle += 2*3.1415927410125732;
			if(angle > 3.1415927410125732)
				angle -= 2*3.1415927410125732;
			

			int idx = static_cast<int>((angle + 3.1415927410125732) / pubMsg.angle_increment);
			if(idx < 0)
				idx = 0;
			if(idx > 897)
				idx = 897;

			pubMsg.intensities[idx] = 250.f;
			pubMsg.ranges[idx] = distance;
			
			printf("%f %f %f, %f, %d, %f, %f\n",xd,yd,distance,angle, idx ,std::atan2(yd, xd), direction);
			(*publisher_).publish(pubMsg);
		}

	}

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        direction = msg->pose.pose.orientation.z * 3.1415927410125732;
    }

private:
	std::vector<std::pair<float,float>> blockPos;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr  publisher_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscrition_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
	
	bool isRun = true;
	float direction = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc,argv);
	
	rclcpp::spin(std::make_shared<ArtificialOccupy>());
	rclcpp::shutdown();

	return 0;
}