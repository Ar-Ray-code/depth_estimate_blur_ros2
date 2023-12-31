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

namespace depth_estimate_blur
{

class DepthEstimateBlur
{
    public:
        DepthEstimateBlur() = default;
        ~DepthEstimateBlur() = default;

        cv::Mat f_blur(const cv::Mat&, double);
        cv::Mat1b depth_estimation(const cv::Mat3b&);
};
}
