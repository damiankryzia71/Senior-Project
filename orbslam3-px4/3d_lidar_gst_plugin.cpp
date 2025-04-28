#ifndef LIDAR_GST_PLUGIN
#define LIDAR_GST_PLUGIN

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

            ~LidarGstPlugin()
            {
                if (this->pipeline)
                {
                    gst_element_set_state(this->pipeline, GST_STATE_NULL);
                    gst_object_unref(this->pipeline);
                    this->pipeline = nullptr;
                    this->udpsinks.clear();
                }
            }

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

                // initialize GStreamer
                gst_init(nullptr, nullptr);

                // create pipeline
                this->pipeline = gst_pipeline_new("lidar_pipeline");
                this->appsrc = gst_element_factory_make("appsrc", "source");
                this->tee = gst_element_factory_make("tee", "tee");

                if (!this->pipeline || !this->appsrc || !this->tee)
                {
                    std::cerr << "Failed to create core GStreamer elements." << std::endl;
                    return;
                }

                // configure appsrc
                GstCaps *caps = gst_caps_new_simple("application/octet-stream", nullptr);
                g_object_set(this->appsrc, "caps", caps, "format", GST_FORMAT_TIME, "is-live", TRUE, nullptr);

                gst_bin_add_many(GST_BIN(this->pipeline), this->appsrc, this->tee, nullptr);

                // configure udpsinks
                for (int port : this->udpPorts)
                {
                    GstElement *queue = gst_element_factory_make("queue", nullptr);
                    GstElement *udpsink = gst_element_factory_make("udpsink", nullptr);
                    g_object_set(udpsink, "host", this->udpHost.c_str(), "port", port, nullptr);

                    if (!queue || !udpsink)
                    {
                        std::cerr << "Failed to create GStreamer queue or udpsink." << std::endl;
                        continue;
                    }                    
                    
                    gst_bin_add_many(GST_BIN(this->pipeline), queue, udpsink, nullptr);

                    if (!gst_element_link_many(this->tee, queue, udpsink, nullptr))
                    {
                        std::cerr << "Failed to link tee -> queue -> udpsink for port " << port << std::endl;
                    }

                    this->udpsinks.push_back(udpsink);
                }

                if (!gst_element_link(this->appsrc, this->tee))
                {
                    std::cerr << "Failed to link appsrc to tee." << std::endl;
                    return;
                }

                if (gst_element_set_state(this->pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
                {
                    std::cerr << "Failed to start GStreamer pipeline." << std::endl;
                    return;
                }

                // initialize transport node
                this->node = transport::NodePtr(new transport::Node());
                this->node->Init();
                this->sub = this->node->Subscribe(topicName, &LidarGstPlugin::msgCallback, this);
            }

            void msgCallback(ConstLaserScanStampedPtr &msg)
            {
                const gazebo::msgs::LaserScan &scan = msg->scan();
                const int count_h = scan.count();          // horizontal samples
                const int count_v = scan.vertical_count(); // vertical samples
            
                if (count_h == 0 || count_v == 0)
                {
                    std::cerr << "Empty 3D scan received, skipping." << std::endl;
                    return;
                }
            
                const int n = count_h * count_v; // total number of points
            
                // Allocate buffer for (x, y, z) per point
                GstBuffer *buffer = gst_buffer_new_allocate(nullptr, n * 3 * sizeof(float), nullptr);
                if (!buffer)
                {
                    std::cerr << "Failed to allocate GStreamer buffer." << std::endl;
                    return;
                }
            
                GstMapInfo map;
                if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE))
                {
                    std::cerr << "Failed to map GStreamer buffer." << std::endl;
                    gst_buffer_unref(buffer);
                    return;
                }
            
                const float angle_min_h = scan.angle_min();
                const float angle_step_h = scan.angle_step();
                const float angle_min_v = scan.vertical_angle_min();
                const float angle_step_v = scan.vertical_angle_step();
            
                for (int v = 0; v < count_v; ++v)
                {
                    float vertical_angle = angle_min_v + v * angle_step_v;
                    for (int h = 0; h < count_h; ++h)
                    {
                        int idx = v * count_h + h;
                        float r = scan.ranges(idx);
            
                        if (r < scan.range_min() || r > scan.range_max())
                        {
                            ((float*)map.data)[idx * 3 + 0] = std::numeric_limits<float>::quiet_NaN();
                            ((float*)map.data)[idx * 3 + 1] = std::numeric_limits<float>::quiet_NaN();
                            ((float*)map.data)[idx * 3 + 2] = std::numeric_limits<float>::quiet_NaN();
                            continue;
                        }
            
                        float horizontal_angle = angle_min_h + h * angle_step_h;
            
                        // 3D point calculation
                        float x = r * cosf(vertical_angle) * cosf(horizontal_angle);
                        float y = r * cosf(vertical_angle) * sinf(horizontal_angle);
                        float z = r * sinf(vertical_angle);
            
                        ((float*)map.data)[idx * 3 + 0] = x;
                        ((float*)map.data)[idx * 3 + 1] = y;
                        ((float*)map.data)[idx * 3 + 2] = z;
                    }
                }
            
                gst_buffer_unmap(buffer, &map);
            
                // Push buffer into GStreamer appsrc
                GstFlowReturn ret;
                g_signal_emit_by_name(this->appsrc, "push-buffer", buffer, &ret);
                gst_buffer_unref(buffer);
            
                if (ret != GST_FLOW_OK)
                    std::cerr << "Failed to push buffer to appsrc\n";
            }                                  

        private:
            sensors::SensorPtr sensor;
            transport::NodePtr node;
            transport::SubscriberPtr sub;
            std::string topicName;
            std::string udpHost;
            std::vector<int> udpPorts;
            GstElement *pipeline = nullptr;
            GstElement *appsrc = nullptr;
            GstElement *tee = nullptr;
            std::vector<GstElement*> udpsinks;
    };

    GZ_REGISTER_SENSOR_PLUGIN(LidarGstPlugin)
}

#endif
