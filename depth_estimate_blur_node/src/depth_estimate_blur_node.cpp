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

#include "depth_estimate_blur_node/depth_estimate_blur_node.hpp"

namespace depth_estimate_blur_node
{

DepthEstimateBlurNode::DepthEstimateBlurNode(rclcpp::NodeOptions options)
    : Node("depth_estimate_blur_node")
{
    (void)options;
    sub_img_ = image_transport::create_subscription(
        this, "image_raw", std::bind(&DepthEstimateBlurNode::image_callback, this, std::placeholders::_1), "raw");
    pub_img_ = image_transport::create_publisher(this, "depth");
}

void DepthEstimateBlurNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    auto img = cv_bridge::toCvCopy(msg, "bgr8");
    this->depth_estimate_blur(img->image);
}

void DepthEstimateBlurNode::depth_estimate_blur(const cv::Mat& img)
{
    cv::Mat1b depth = depth_estimate_blur_.depth_estimation(img);
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", depth).toImageMsg();
    pub_img_.publish(msg);
}

}
