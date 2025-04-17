#ifndef LIDAR_GST_PLUGIN
#define LIDAR_GST_PLUGIN

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <iostream>
#include <string>
#include <vector>

namespace gazebo
{
    class LidarGstPlugin : public SensorPlugin
    {
        public:
            LidarGstPlugin() {}

            virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
            {
                this->sensor = sensor;

                // load sdf fields (not udp ports)
                this->topicName = sdf->Get<std::string>("topicName");
                this->udpHost = sdf->Get<std::string>("udpHost");

                // load udp ports
                std::string ports = sdf->Get<std::string>("udpPorts");
                std::stringstream ss(ports);
                std::string port;
                while (std::getline(ss, port, ','))
                    this->udpPorts.push_back(std::stoi(port));

                std::cout << "Plugin loaded" << "\n";
                std::cout << "LiDAR topic: " << this->topicName << "\n";
                std::cout << "UDP host: " << this->udpHost << "\n";
                std::cout << "UDP ports: ";
                for (int port : this->udpPorts)
                {
                    std::cout << std::to_string(port) << " ";
                }
                std::cout << std::endl;

                // initialize transport node
                this->node = transport::NodePtr(new transport::Node());
                this->node->Init();
                this->sub = this->node->Subscribe(topicName, &LidarGstPlugin::msgCallback, this);
            }

            void msgCallback(ConstLaserScanStampedPtr &msg)
            {
                static bool executed = false;
                const gazebo::msgs::LaserScan &scan = msg->scan();
            
                if (!executed) // Print info on only one message
                {
                    std::cout << "angle_min: " << scan.angle_min() << "\n";
                    std::cout << "angle_max: " << scan.angle_max() << "\n";
                    std::cout << "count: " << scan.count() << "\n";
                    std::cout << "intensities size: " << scan.intensities_size() << "\n";
                    std::cout << "ranges size: " << scan.ranges_size() << "\n";
                    executed = true;
                }
            }               

        private:
            sensors::SensorPtr sensor;
            transport::NodePtr node;
            transport::SubscriberPtr sub;
            std::string topicName;
            std::string udpHost;
            std::vector<int> udpPorts;
    };

    GZ_REGISTER_SENSOR_PLUGIN(LidarGstPlugin)
}

#endif
