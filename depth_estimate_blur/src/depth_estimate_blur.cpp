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

#include "depth_estimate_blur/depth_estimate_blur.hpp"


namespace depth_estimate_blur
{

cv::Mat DepthEstimateBlur::f_blur(const cv::Mat& img, double sigma) {
    cv::Mat blurred;
    cv::GaussianBlur(img, blurred, cv::Size(), sigma);
    return blurred;
}

cv::Mat1b DepthEstimateBlur::depth_estimation(const cv::Mat3b& img) {
    int N = 8;
    double thres = 0.0;

    cv::Mat ref_val;
    cv::cvtColor(img, ref_val, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(ref_val, channels);
    cv::Mat ref_scale = channels[2];

    cv::Mat ref_gray_bk = ref_scale.clone();
    double gthresh1 = cv::threshold(ref_scale, ref_scale, 0, 255, cv::THRESH_OTSU);
    ref_scale = ref_gray_bk.clone();

    double gthresh2 = cv::threshold(ref_scale, ref_scale, 0, 255, cv::THRESH_OTSU);

    cv::Mat ref_val_otsu = cv::Mat::zeros(ref_scale.size(), CV_64F);
    ref_val_otsu.setTo(2.0, ref_gray_bk < gthresh2);
    ref_val_otsu.setTo(1.0, ref_gray_bk >= gthresh2);
    ref_val_otsu.setTo(0.0, ref_gray_bk >= gthresh1);

    cv::TickMeter tick;
    tick.start();

    cv::Mat ref_spa = f_blur(img, 4) - f_blur(img, 16);
    std::vector<cv::Mat> imgChannels;
    cv::split(ref_spa, imgChannels);
    ref_spa = imgChannels[1];
    cv::Mat edges;
    cv::Canny(img, edges, 100, 200);
    ref_spa.setTo(0, edges == 0);

    int im_width = ref_spa.cols;
    int im_height = ref_spa.rows;

    cv::Mat1b img_dense = cv::Mat::zeros(ref_spa.size(), CV_64F);

    bool fill_enable = false;
    double edge_factor = 0.0;

    for (int i = 0; i < im_width - N; i += N) {
        for (int j = 0; j < im_height - N; j += N) {
            cv::Mat pick_defocus = ref_spa(cv::Rect(i, j, N, N));
            cv::Mat pick_matrices = ref_val_otsu(cv::Rect(i, j, N, N));

            double ker_max;
            cv::minMaxLoc(pick_defocus, nullptr, &ker_max);

            double mat_sum = cv::sum(pick_matrices)[0];

            if (ker_max > 0.0) {
                if (fill_enable) {
                    if (edge_factor < ker_max) {
                        edge_factor = ker_max;
                    }
                }
                else {
                    edge_factor = ker_max;
                    fill_enable = true;
                }
            }

            if (fill_enable) {
                img_dense(cv::Rect(i, j, N, N)) = edge_factor * pick_matrices;
            }

            if (mat_sum <= thres) {
                fill_enable = false;
                edge_factor = 0.0;
            }
        }
    }

    tick.stop();
    return img_dense;
}

}
