// Copyright 2023 Ar-Ray-code.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <opencv2/opencv.hpp>
#include <depth_estimate_blur/depth_estimate_blur.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

namespace depth_estimate_blur_node
{

class DepthEstimateBlurNode : public rclcpp::Node
{
    public:
        DepthEstimateBlurNode(rclcpp::NodeOptions);
        ~DepthEstimateBlurNode() = default;

    private:
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr&);
        void depth_estimate_blur(const cv::Mat&);

        image_transport::Subscriber sub_img_;
        image_transport::Publisher pub_img_;
        depth_estimate_blur::DepthEstimateBlur depth_estimate_blur_;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(depth_estimate_blur_node::DepthEstimateBlurNode)