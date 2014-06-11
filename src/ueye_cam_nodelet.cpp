/*******************************************************************************
* DO NOT MODIFY - AUTO-GENERATED
*
*
* DISCLAMER:
*
* This project was created within an academic research setting, and thus should
* be considered as EXPERIMENTAL code. There may be bugs and deficiencies in the
* code, so please adjust expectations accordingly. With that said, we are
* intrinsically motivated to ensure its correctness (and often its performance).
* Please use the corresponding web repository tool (e.g. github/bitbucket/etc.)
* to file bugs, suggestions, pull requests; we will do our best to address them
* in a timely manner.
*
*
* SOFTWARE LICENSE AGREEMENT (BSD LICENSE):
*
* Copyright (c) 2013, Anqi Xu
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of the School of Computer Science, McGill University,
*    nor the names of its contributors may be used to endorse or promote
*    products derived from this software without specific prior written
*    permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include "ueye_cam_nodelet.hpp"
#include <cstdlib> // needed for getenv()
#include <ros/package.h>
#include <driver_base/SensorLevels.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>

//#define DEBUG_PRINTOUT_FRAME_GRAB_RATES


using namespace std;
using namespace sensor_msgs::image_encodings;


namespace ueye_cam {


const string UEyeCamNodelet::DEFAULT_CAMERA_NAME = "camera";
const string UEyeCamNodelet::DEFAULT_CAMERA_TOPIC = "image_raw";
const int UEyeCamNodelet::DEFAULT_IMAGE_WIDTH = 752;
const int UEyeCamNodelet::DEFAULT_IMAGE_HEIGHT = 480;
const string UEyeCamNodelet::DEFAULT_COLOR_MODE = " ";
const double UEyeCamNodelet::DEFAULT_EXPOSURE = 3.0;
const double UEyeCamNodelet::DEFAULT_FRAME_RATE = 30.0;
const int UEyeCamNodelet::DEFAULT_PIXEL_CLOCK = 25;
const int UEyeCamNodelet::DEFAULT_FLASH_DURATION = 1000;


UEyeCamNodelet::UEyeCamNodelet() :
    nodelet::Nodelet(),
    UEyeCamDriver(ANY_CAMERA, DEFAULT_CAMERA_NAME),
    frame_grab_alive_(false),
    ros_cfg_(NULL),
    cfg_sync_requested_(false),
    ros_frame_count_(0),
    cam_topic_(DEFAULT_CAMERA_TOPIC),
    cam_intr_filename_(""),
    cam_params_filename_("") {
  cam_params_.image_width = DEFAULT_IMAGE_WIDTH;
  cam_params_.image_height = DEFAULT_IMAGE_HEIGHT;
  cam_params_.image_left = -1;
  cam_params_.image_top = -1;
  cam_params_.color_mode = DEFAULT_COLOR_MODE;
  cam_params_.subsampling = cam_subsampling_rate_;
  cam_params_.binning = cam_binning_rate_;
  cam_params_.sensor_scaling = cam_sensor_scaling_rate_;
  cam_params_.auto_gain = true;
  cam_params_.master_gain = 0;
  cam_params_.red_gain = 0;
  cam_params_.green_gain = 0;
  cam_params_.blue_gain = 0;
  cam_params_.gain_boost = 0;
  cam_params_.auto_exposure = true;
  cam_params_.exposure = DEFAULT_EXPOSURE;
  cam_params_.auto_white_balance = false;
  cam_params_.white_balance_red_offset = 0;
  cam_params_.white_balance_blue_offset = 0;
  cam_params_.auto_frame_rate = false;
  cam_params_.frame_rate = DEFAULT_FRAME_RATE;
  cam_params_.pixel_clock = DEFAULT_PIXEL_CLOCK;
  cam_params_.ext_trigger_mode = false;
  cam_params_.flash_delay = 0;
  cam_params_.flash_duration = DEFAULT_FLASH_DURATION;
};


UEyeCamNodelet::~UEyeCamNodelet() {
  disconnectCam();

  // NOTE: sometimes deleting dynamic reconfigure object will lock up
  //       (suspect the scoped lock is not releasing the recursive mutex)
  //
  //if (ros_cfg_ != NULL) {
  //  delete ros_cfg_;
  //  ros_cfg_ = NULL;
  //}
};

void UEyeCamNodelet::initRectification(){
    // intitialize rectification stuff
    // my additions for rectivication
    remapX = remapY = 0;
    if(!ros::param::get("~calibFile", cfg_file))
    {
        cfg_file = "";
        printf("No calib file -> not rectifying!\n");
        doRect = false;
    }
    else
    {
        std::string rect;
        doRect = true;
    }


    // prep rect
    if(cfg_file != "")
    {
        // read parameters
        std::ifstream infile(cfg_file.c_str());
        std::string l1,l2,l3,l4;

        std::getline(infile,l1);
        std::getline(infile,l2);
        std::getline(infile,l3);
        std::getline(infile,l4);


        // l1 & l2
        if(std::sscanf(l1.c_str(), "%lf %lf %lf %lf %lf", &inputCalibration[0], &inputCalibration[1], &inputCalibration[2], &inputCalibration[3], &inputCalibration[4]) == 5 &&
                std::sscanf(l2.c_str(), "%d %d", &in_width, &in_height) == 2)
        {
            printf("Input resolution: %d %d\n",in_width, in_height);
            printf("In: %f %f %f %f %f\n",
                    inputCalibration[0], inputCalibration[1], inputCalibration[2], inputCalibration[3], inputCalibration[4]);
        }
        else
        {
            printf("Failed to read camera calibration (invalid format?)\n");
            doRect = false;
        }

        // l3
        if(l3 == "crop")
        {
            outputCalibration[0] = -1;
            printf("Out: Crop\n");
        }
        else if(l3 == "full")
        {
            outputCalibration[0] = -2;
            printf("Out: Full\n");
        }
        else if(l3 == "none")
        {
            printf("NO RECTIFICATION\n");
            doRect = false;
        }
        else if(std::sscanf(l3.c_str(), "%lf %lf %lf %lf %lf", &outputCalibration[0], &outputCalibration[1], &outputCalibration[2], &outputCalibration[3], &outputCalibration[4]) == 5)
        {
            printf("Out: %f %f %f %f %f\n",
                    outputCalibration[0], outputCalibration[1], outputCalibration[2], outputCalibration[3], outputCalibration[4]);
        }
        else
        {
            printf("Out: Failed to Read Output pars... not rectifying.\n");
            doRect = false;
        }


        // l4
        if(std::sscanf(l4.c_str(), "%d %d", &out_width, &out_height) == 2)
        {
            printf("Output resolution: %d %d\n",out_width, out_height);
        }
        else
        {
            printf("Out: Failed to Read Output resolution... not rectifying.\n");
            doRect = false;
        }




        // prep warp matrices
        if(doRect)
        {
            ros_image_.data.resize(out_width*out_height);
            ros_image_.step = out_width;
            remapX = new float[out_width * out_height];
            remapY = new float[out_width * out_height];

            float dist = inputCalibration[4];
            float d2t = 2.0f * tan(dist / 2.0f);

            // current camera parameters
            float fx = inputCalibration[0] * in_width;
            float fy = inputCalibration[1] * in_height;
            float cx = inputCalibration[2] * in_width - 0.5;
            float cy = inputCalibration[3] * in_height - 0.5;

            // output camera parameters
            float ofx, ofy, ocx, ocy;

            // find new camera matrix for "crop" and "full"
            if(outputCalibration[0] == -1)	// "crop"
            {
                // find left-most and right-most radius
                float left_radius = (cx)/fx;
                float right_radius = (in_width-1 - cx)/fx; //DAVID: WHy is there a minus one here? left_radius+right_radius != width
                float top_radius = (cy)/fy;
                float bottom_radius = (in_height-1 - cy)/fy; //DAVID: Same question

                float trans_left_radius = tan(left_radius * dist)/d2t;
                float trans_right_radius = tan(right_radius * dist)/d2t;
                float trans_top_radius = tan(top_radius * dist)/d2t;
                float trans_bottom_radius = tan(bottom_radius * dist)/d2t;

                printf("left_radius: %f -> %f\n", left_radius, trans_left_radius);
                printf("right_radius: %f -> %f\n", right_radius, trans_right_radius);
                printf("top_radius: %f -> %f\n", top_radius, trans_top_radius);
                printf("bottom_radius: %f -> %f\n", bottom_radius, trans_bottom_radius);


                ofy = fy * ((top_radius + bottom_radius) / (trans_top_radius + trans_bottom_radius)) * ((float)out_height / (float)in_height);
                ocy = (trans_top_radius/top_radius) * ofy*cy/fy;

                ofx = fx * ((left_radius + right_radius) / (trans_left_radius + trans_right_radius)) * ((float)out_width / (float)in_width);
                ocx = (trans_left_radius/left_radius) * ofx*cx/fx;

                printf("new K: %f %f %f %f\n",ofx,ofy,ocx,ocy);
                printf("old K: %f %f %f %f\n",fx,fy,cx,cy);
            }
            else if(outputCalibration[0] == -2)	 // "full"
            {
                float left_radius = cx/fx;
                float right_radius = (in_width-1 - cx)/fx;
                float top_radius = cy/fy;
                float bottom_radius = (in_height-1 - cy)/fy;

                // find left-most and right-most radius
                float tl_radius = sqrt(left_radius*left_radius + top_radius*top_radius);
                float tr_radius = sqrt(right_radius*right_radius + top_radius*top_radius);
                float bl_radius = sqrt(left_radius*left_radius + bottom_radius*bottom_radius);
                float br_radius = sqrt(right_radius*right_radius + bottom_radius*bottom_radius);

                float trans_tl_radius = tan(tl_radius * dist)/d2t;
                float trans_tr_radius = tan(tr_radius * dist)/d2t;
                float trans_bl_radius = tan(bl_radius * dist)/d2t;
                float trans_br_radius = tan(br_radius * dist)/d2t;

                //printf("trans_tl_radius: %f -> %f\n", tl_radius, trans_tl_radius);
                //printf("trans_tr_radius: %f -> %f\n", tr_radius, trans_tr_radius);
                //printf("trans_bl_radius: %f -> %f\n", bl_radius, trans_bl_radius);
                //printf("trans_br_radius: %f -> %f\n", br_radius, trans_br_radius);


                float hor = max(br_radius,tr_radius) + max(bl_radius,tl_radius);
                float vert = max(tr_radius,tl_radius) + max(bl_radius,br_radius);

                float trans_hor = max(trans_br_radius,trans_tr_radius) + max(trans_bl_radius,trans_tl_radius);
                float trans_vert = max(trans_tr_radius,trans_tl_radius) + max(trans_bl_radius,trans_br_radius);

                ofy = fy * ((vert) / (trans_vert)) * ((float)out_height / (float)in_height);
                ocy = max(trans_tl_radius/tl_radius,trans_tr_radius/tr_radius) * ofy*cy/fy;

                ofx = fx * ((hor) / (trans_hor)) * ((float)out_width / (float)in_width);
                ocx = max(trans_bl_radius/bl_radius,trans_tl_radius/tl_radius) * ofx*cx/fx;

                printf("new K: %f %f %f %f\n",ofx,ofy,ocx,ocy);
                printf("old K: %f %f %f %f\n",fx,fy,cx,cy);
            }
            else
            {
                ofx = outputCalibration[0] * out_width;
                ofy = outputCalibration[1] * out_height;
                ocx = outputCalibration[2] * out_width-0.5;	// TODO: -0.5 here or not?
                ocy = outputCalibration[3] * out_height-0.5;
            }

            outputCalibration[0] = ofx / out_width;
            outputCalibration[1] = ofy / out_height;
            outputCalibration[2] = (ocx+0.5) / out_width;
            outputCalibration[3] = (ocy+0.5) / out_height;
            outputCalibration[4] = 0;

            for(int x=0;x<out_width;x++)
                for(int y=0;y<out_height;y++)
                {
                    float ix = (x - ocx) / ofx;
                    float iy = (y - ocy) / ofy;

                    float r = sqrt(ix*ix + iy*iy);
                    float fac = atan(r * d2t)/(dist*r);

                    if(r==0) fac = 1;

                    ix = fx*fac*ix+cx;
                    iy = fy*fac*iy+cy;

                    // make rounding resistant.
                    if(ix == 0) ix = 0.01;
                    if(iy == 0) iy = 0.01;
                    if(ix == in_width-1) ix = in_width-1.01;
                    if(iy == in_height-1) ix = in_height-1.01;

                    if(ix > 0 && iy > 0 && ix < in_width-1 &&  iy < in_height-1)
                    {
                        remapX[x+y*out_width] = ix;
                        remapY[x+y*out_width] = iy;
                    }
                    else
                    {
                        remapX[x+y*out_width] = -1;
                        remapY[x+y*out_width] = -1;
                    }
                }
            printf("Prepped Warp matrices\n");
        }
        else
        {
            printf("Not Rectifying\n");
            outputCalibration[0] = inputCalibration[0];
            outputCalibration[1] = inputCalibration[1];
            outputCalibration[2] = inputCalibration[2];
            outputCalibration[3] = inputCalibration[3];
            outputCalibration[4] = inputCalibration[4];
            out_width = in_width;
            out_height = in_height;
        }


        for(int i=0;i<9;i++)
            ros_cam_info_.K.at(i) = 0;

        for(int i=0;i<12;i++)
            ros_cam_info_.P.at(i) = 0;

        ros_cam_info_.K.at(0) = outputCalibration[0] * out_width;
        ros_cam_info_.K.at(4) = outputCalibration[1] * out_height;
        ros_cam_info_.K.at(8) = 1;
        ros_cam_info_.K.at(2) = outputCalibration[2] * out_width - 0.5;
        ros_cam_info_.K.at(5) = outputCalibration[3] * out_height - 0.5;

        ros_cam_info_.P.at(0) = outputCalibration[0] * out_width;
        ros_cam_info_.P.at(5) = outputCalibration[1] * out_height;
        ros_cam_info_.P.at(10) = 1;
        ros_cam_info_.P.at(2) = outputCalibration[2] * out_width - 0.5;
        ros_cam_info_.P.at(6) = outputCalibration[3] * out_height - 0.5;

        ros_cam_info_.D.clear();
        ros_cam_info_.D.push_back(outputCalibration[4]);
        ros_cam_info_.D.push_back(0);
        ros_cam_info_.D.push_back(0);
        ros_cam_info_.D.push_back(0);
        ros_cam_info_.D.push_back(0);


        /*  Supported models are listed in sensor_msgs/distortion_models.h.
            For most cameras, "plumb_bob" - a simple model of radial and
            tangential distortion - is sufficent. */
        ros_cam_info_.distortion_model = "plumb_bob";

    }
}

void UEyeCamNodelet::onInit() {
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& local_nh = getPrivateNodeHandle();
  image_transport::ImageTransport it(nh);

  // Load camera-agnostic ROS parameters
  local_nh.param<string>("camera_name", cam_name_, DEFAULT_CAMERA_NAME);
  local_nh.param<string>("camera_topic", cam_topic_, DEFAULT_CAMERA_TOPIC);
  local_nh.param<string>("camera_intrinsics_file", cam_intr_filename_, "");
  local_nh.param<int>("camera_id", cam_id_, ANY_CAMERA);
  local_nh.param<string>("camera_parameters_file", cam_params_filename_, "");
  if (cam_id_ < 0) {
    NODELET_WARN_STREAM("Invalid camera ID specified: " << cam_id_ <<
      "; setting to ANY_CAMERA");
    cam_id_ = ANY_CAMERA;
  }

  loadIntrinsicsFile();
  initRectification();
  // Setup dynamic reconfigure server
  ros_cfg_ = new ReconfigureServer(ros_cfg_mutex_, local_nh);
  ReconfigureServer::CallbackType f;
  f = bind(&UEyeCamNodelet::configCallback, this, _1, _2);

  // Setup publishers, subscribers, and services
  ros_cam_pub_ = it.advertiseCamera("/" + cam_name_ + "/" + cam_topic_, 1);
  set_cam_info_srv_ = nh.advertiseService("/" + cam_name_ + "/set_camera_info",
      &UEyeCamNodelet::setCamInfo, this);

  // Initiate camera and start capture
  if (connectCam() != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to initialize UEye camera '" << cam_name_ << "'");
    return;
  }
  ros_cfg_->setCallback(f); // this will call configCallback, which will configure the camera's parameters
  startFrameGrabber();
  NODELET_INFO_STREAM(
      "UEye camera '" << cam_name_ << "' initialized on topic " << ros_cam_pub_.getTopic() << endl <<
      "Width:\t\t\t" << cam_params_.image_width << endl <<
      "Height:\t\t\t" << cam_params_.image_height << endl <<
      "Left Pos.:\t\t" << cam_params_.image_left << endl <<
      "Top Pos.:\t\t" << cam_params_.image_top << endl <<
      "Color Mode:\t\t" << cam_params_.color_mode << endl <<
      "Subsampling:\t\t" << cam_params_.subsampling << endl <<
      "Binning:\t\t" << cam_params_.binning << endl <<
      "Sensor Scaling:\t\t" << cam_params_.sensor_scaling << endl <<
      "Auto Gain:\t\t" << cam_params_.auto_gain << endl <<
      "Master Gain:\t\t" << cam_params_.master_gain << endl <<
      "Red Gain:\t\t" << cam_params_.red_gain << endl <<
      "Green Gain:\t\t" << cam_params_.green_gain << endl <<
      "Blue Gain:\t\t" << cam_params_.blue_gain << endl <<
      "Gain Boost:\t\t" << cam_params_.gain_boost << endl <<
      "Auto Exposure:\t\t" << cam_params_.auto_exposure << endl <<
      "Exposure (ms):\t\t" << cam_params_.exposure << endl <<
      "Auto White Balance:\t" << cam_params_.auto_white_balance << endl <<
      "WB Red Offset:\t\t" << cam_params_.white_balance_red_offset << endl <<
      "WB Blue Offset:\t\t" << cam_params_.white_balance_blue_offset << endl <<
      "Flash Delay (us):\t" << cam_params_.flash_delay << endl <<
      "Flash Duration (us):\t" << cam_params_.flash_duration << endl <<
      "Ext Trigger Mode:\t" << cam_params_.ext_trigger_mode << endl <<
      "Auto Frame Rate:\t" << cam_params_.auto_frame_rate << endl <<
      "Frame Rate (Hz):\t" << cam_params_.frame_rate << endl <<
      "Pixel Clock (MHz):\t" << cam_params_.pixel_clock << endl
      );
};


INT UEyeCamNodelet::parseROSParams(ros::NodeHandle& local_nh) {
  bool hasNewParams = false;
  ueye_cam::UEyeCamConfig prevCamParams = cam_params_;
  INT is_err = IS_SUCCESS;

  if (local_nh.hasParam("image_width")) {
    local_nh.getParam("image_width", cam_params_.image_width);
    if (cam_params_.image_width != prevCamParams.image_width) {
      if (cam_params_.image_width <= 0) {
        NODELET_WARN_STREAM("Invalid requested image width: " << cam_params_.image_width <<
          "; using current width: " << prevCamParams.image_width);
        cam_params_.image_width = prevCamParams.image_width;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("image_height")) {
    local_nh.getParam("image_height", cam_params_.image_height);
    if (cam_params_.image_height != prevCamParams.image_height) {
      if (cam_params_.image_height <= 0) {
        NODELET_WARN_STREAM("Invalid requested image height: " << cam_params_.image_height <<
          "; using current height: " << prevCamParams.image_height);
        cam_params_.image_height = prevCamParams.image_height;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("image_top")) {
    local_nh.getParam("image_top", cam_params_.image_top);
    if (cam_params_.image_top != prevCamParams.image_top) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("image_left")) {
    local_nh.getParam("image_left", cam_params_.image_left);
    if (cam_params_.image_left != prevCamParams.image_left) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("color_mode")) {
    local_nh.getParam("color_mode", cam_params_.color_mode);
    if (cam_params_.color_mode != prevCamParams.color_mode) {
      if (cam_params_.color_mode.length() > 0) {
        transform(cam_params_.color_mode.begin(),
            cam_params_.color_mode.end(),
            cam_params_.color_mode.begin(),
            ::tolower);
        if (cam_params_.color_mode != RGB8 &&
            cam_params_.color_mode != MONO8 &&
            cam_params_.color_mode != BAYER_RGGB8) {
          NODELET_WARN_STREAM("Invalid requested color mode: " << cam_params_.color_mode <<
            "; using current mode: " << prevCamParams.color_mode);
          cam_params_.color_mode = prevCamParams.color_mode;
        } else {
          hasNewParams = true;
        }
      } else { // Empty requested color mode string
        cam_params_.color_mode = prevCamParams.color_mode;
      }
    }
  }
  if (local_nh.hasParam("subsampling")) {
    local_nh.getParam("subsampling", cam_params_.subsampling);
    if (cam_params_.subsampling != prevCamParams.subsampling) {
      if (!(cam_params_.subsampling == 1 ||
          cam_params_.subsampling == 2 ||
          cam_params_.subsampling == 4 ||
          cam_params_.subsampling == 8 ||
          cam_params_.subsampling == 16)) {
        NODELET_WARN_STREAM("Invalid or unsupported requested subsampling rate: " << cam_params_.subsampling <<
            "; using current rate: " << prevCamParams.subsampling);
        cam_params_.subsampling = prevCamParams.subsampling;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("auto_gain")) {
    local_nh.getParam("auto_gain", cam_params_.auto_gain);
    if (cam_params_.auto_gain != prevCamParams.auto_gain) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("master_gain")) {
    local_nh.getParam("master_gain", cam_params_.master_gain);
    if (cam_params_.master_gain != prevCamParams.master_gain) {
      if (cam_params_.master_gain < 0 || cam_params_.master_gain > 100) {
        NODELET_WARN_STREAM("Invalid master gain: " << cam_params_.master_gain <<
            "; using current master gain: " << prevCamParams.master_gain);
        cam_params_.master_gain = prevCamParams.master_gain;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("red_gain")) {
    local_nh.getParam("red_gain", cam_params_.red_gain);
    if (cam_params_.red_gain != prevCamParams.red_gain) {
      if (cam_params_.red_gain < 0 || cam_params_.red_gain > 100) {
        NODELET_WARN_STREAM("Invalid red gain: " << cam_params_.red_gain <<
            "; using current red gain: " << prevCamParams.red_gain);
        cam_params_.red_gain = prevCamParams.red_gain;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("green_gain")) {
    local_nh.getParam("green_gain", cam_params_.green_gain);
    if (cam_params_.green_gain != prevCamParams.green_gain) {
      if (cam_params_.green_gain < 0 || cam_params_.green_gain > 100) {
        NODELET_WARN_STREAM("Invalid green gain: " << cam_params_.green_gain <<
            "; using current green gain: " << prevCamParams.green_gain);
        cam_params_.green_gain = prevCamParams.green_gain;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("blue_gain")) {
    local_nh.getParam("blue_gain", cam_params_.blue_gain);
    if (cam_params_.blue_gain != prevCamParams.blue_gain) {
      if (cam_params_.blue_gain < 0 || cam_params_.blue_gain > 100) {
        NODELET_WARN_STREAM("Invalid blue gain: " << cam_params_.blue_gain <<
            "; using current blue gain: " << prevCamParams.blue_gain);
        cam_params_.blue_gain = prevCamParams.blue_gain;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("gain_boost")) {
    local_nh.getParam("gain_boost", cam_params_.gain_boost);
    if (cam_params_.gain_boost != prevCamParams.gain_boost) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("auto_exposure")) {
    local_nh.getParam("auto_exposure", cam_params_.auto_exposure);
    if (cam_params_.auto_exposure != prevCamParams.auto_exposure) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("exposure")) {
    local_nh.getParam("exposure", cam_params_.exposure);
    if (cam_params_.exposure != prevCamParams.exposure) {
      if (cam_params_.exposure < 0.0) {
        NODELET_WARN_STREAM("Invalid requested exposure: " << cam_params_.exposure <<
          "; using current exposure: " << prevCamParams.exposure);
        cam_params_.exposure = prevCamParams.exposure;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("auto_white_balance")) {
    local_nh.getParam("auto_white_balance", cam_params_.auto_white_balance);
    if (cam_params_.auto_white_balance != prevCamParams.auto_white_balance) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("white_balance_red_offset")) {
    local_nh.getParam("white_balance_red_offset", cam_params_.white_balance_red_offset);
    if (cam_params_.white_balance_red_offset != prevCamParams.white_balance_red_offset) {
      if (cam_params_.white_balance_red_offset < -50 || cam_params_.white_balance_red_offset > 50) {
        NODELET_WARN_STREAM("Invalid white balance red offset: " << cam_params_.white_balance_red_offset <<
            "; using current white balance red offset: " << prevCamParams.white_balance_red_offset);
        cam_params_.white_balance_red_offset = prevCamParams.white_balance_red_offset;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("white_balance_blue_offset")) {
    local_nh.getParam("white_balance_blue_offset", cam_params_.white_balance_blue_offset);
    if (cam_params_.white_balance_blue_offset != prevCamParams.white_balance_blue_offset) {
      if (cam_params_.white_balance_blue_offset < -50 || cam_params_.white_balance_blue_offset > 50) {
        NODELET_WARN_STREAM("Invalid white balance blue offset: " << cam_params_.white_balance_blue_offset <<
            "; using current white balance blue offset: " << prevCamParams.white_balance_blue_offset);
        cam_params_.white_balance_blue_offset = prevCamParams.white_balance_blue_offset;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("ext_trigger_mode")) {
    local_nh.getParam("ext_trigger_mode", cam_params_.ext_trigger_mode);
    // NOTE: no need to set any parameters, since external trigger / live-run
    //       modes come into effect during frame grab loop, which is assumed
    //       to not having been initialized yet
  }
  if (local_nh.hasParam("flash_delay")) {
    local_nh.getParam("flash_delay", cam_params_.flash_delay);
    // NOTE: no need to set any parameters, since flash delay comes into
    //       effect during frame grab loop, which is assumed to not having been
    //       initialized yet
  }
  if (local_nh.hasParam("flash_duration")) {
    local_nh.getParam("flash_duration", cam_params_.flash_duration);
    if (cam_params_.flash_duration < 0) {
      NODELET_WARN_STREAM("Invalid flash duration: " << cam_params_.flash_duration <<
          "; using current flash duration: " << prevCamParams.flash_duration);
      cam_params_.flash_duration = prevCamParams.flash_duration;
    }
    // NOTE: no need to set any parameters, since flash duration comes into
    //       effect during frame grab loop, which is assumed to not having been
    //       initialized yet
  }
  if (local_nh.hasParam("auto_frame_rate")) {
    local_nh.getParam("auto_frame_rate", cam_params_.auto_frame_rate);
    if (cam_params_.auto_frame_rate != prevCamParams.auto_frame_rate) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("frame_rate")) {
    local_nh.getParam("frame_rate", cam_params_.frame_rate);
    if (cam_params_.frame_rate != prevCamParams.frame_rate) {
      if (cam_params_.frame_rate <= 0.0) {
        NODELET_WARN_STREAM("Invalid requested frame rate: " << cam_params_.frame_rate <<
          "; using current frame rate: " << prevCamParams.frame_rate);
        cam_params_.frame_rate = prevCamParams.frame_rate;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("pixel_clock")) {
    local_nh.getParam("pixel_clock", cam_params_.pixel_clock);
    if (cam_params_.pixel_clock != prevCamParams.pixel_clock) {
      if (cam_params_.pixel_clock < 0) {
        NODELET_WARN_STREAM("Invalid requested pixel clock: " << cam_params_.pixel_clock <<
          "; using current pixel clock: " << prevCamParams.pixel_clock);
        cam_params_.pixel_clock = prevCamParams.pixel_clock;
      } else {
        hasNewParams = true;
      }
    }
  }

  if (hasNewParams) {
    // Configure color mode, resolution, and subsampling rate
    if ((is_err = setColorMode(cam_params_.color_mode, true)) != IS_SUCCESS) return is_err;
    if ((is_err = setResolution(cam_params_.image_width, cam_params_.image_height,
        cam_params_.image_left, cam_params_.image_top, false)) != IS_SUCCESS) return is_err;
    if ((is_err = setSubsampling(cam_params_.subsampling, false)) != IS_SUCCESS) return is_err;
    if ((is_err = setBinning(cam_params_.binning, false)) != IS_SUCCESS) return is_err;
    if ((is_err = setSensorScaling(cam_params_.sensor_scaling, true)) != IS_SUCCESS) return is_err;

    // (Re-)populate ROS image message
    // NOTE: the non-ROS UEye parameters and buffers have been updated by setColorMode, setResolution(), and setSubsampling()
    ros_image_.header.frame_id = "/" + cam_name_;
    ros_image_.height = cam_params_.image_height / (cam_params_.sensor_scaling * cam_params_.subsampling * cam_params_.binning);
    ros_image_.width = cam_params_.image_width / (cam_params_.sensor_scaling * cam_params_.subsampling * cam_params_.binning);
    ros_image_.encoding = cam_params_.color_mode;
    ros_image_.step = cam_buffer_pitch_;
    ros_image_.is_bigendian = 0;
    ros_image_.data.resize(cam_buffer_size_);

    // Check for mutual exclusivity among requested sensor parameters
    if (!cam_params_.auto_exposure) { // Auto frame rate requires auto shutter
      cam_params_.auto_frame_rate = false;
    }
    if (cam_params_.auto_frame_rate) { // Auto frame rate has precedence over auto gain
      cam_params_.auto_gain = false;
    }

    // Configure camera sensor parameters
    if ((is_err = setGain(cam_params_.auto_gain, cam_params_.master_gain,
        cam_params_.red_gain, cam_params_.green_gain,
        cam_params_.blue_gain, cam_params_.gain_boost)) != IS_SUCCESS) return is_err;
    if ((is_err = setPixelClockRate(cam_params_.pixel_clock)) != IS_SUCCESS) return is_err;
    if ((is_err = setFrameRate(cam_params_.auto_frame_rate, cam_params_.frame_rate)) != IS_SUCCESS) return is_err;
    if ((is_err = setExposure(cam_params_.auto_exposure, cam_params_.exposure)) != IS_SUCCESS) return is_err;
    if ((is_err = setWhiteBalance(cam_params_.auto_white_balance, cam_params_.white_balance_red_offset,
      cam_params_.white_balance_blue_offset)) != IS_SUCCESS) return is_err;
  }

  return is_err;
};


void UEyeCamNodelet::configCallback(ueye_cam::UEyeCamConfig& config, uint32_t level) {
  if (!isConnected()) return;

  // See if frame grabber needs to be restarted
  bool restartFrameGrabber = false;
  bool needToReallocateBuffer = false;
  if (level == driver_base::SensorLevels::RECONFIGURE_STOP && frame_grab_alive_) {
    restartFrameGrabber = true;
    stopFrameGrabber();
  }

  // Configure color mode, resolution, and subsampling rate
  if (config.color_mode != cam_params_.color_mode) {
    needToReallocateBuffer = true;
    if (setColorMode(config.color_mode, false) != IS_SUCCESS) return;
  }

  if (config.image_width != cam_params_.image_width ||
      config.image_height != cam_params_.image_height ||
      config.image_left != cam_params_.image_left ||
      config.image_top != cam_params_.image_top) {
    needToReallocateBuffer = true;
    if (setResolution(config.image_width, config.image_height,
        config.image_left, config.image_top, false) != IS_SUCCESS) {
      // Attempt to restore previous (working) resolution
      config.image_width = cam_params_.image_width;
      config.image_height = cam_params_.image_height;
      config.image_left = cam_params_.image_left;
      config.image_top = cam_params_.image_top;
      if (setResolution(config.image_width, config.image_height,
          config.image_left, config.image_top) != IS_SUCCESS) return;
    }
  }

  if (config.subsampling != cam_params_.subsampling) {
    needToReallocateBuffer = true;
    if (setSubsampling(config.subsampling, false) != IS_SUCCESS) return;
  }

  if (config.binning != cam_params_.binning) {
    needToReallocateBuffer = true;
    if (setBinning(config.binning, false) != IS_SUCCESS) return;
  }

  if (config.sensor_scaling != cam_params_.sensor_scaling) {
    needToReallocateBuffer = true;
    if (setSensorScaling(config.sensor_scaling, false) != IS_SUCCESS) return;
  }

  if (needToReallocateBuffer) {
    if (reallocateCamBuffer() != IS_SUCCESS) return;
    needToReallocateBuffer = false;
  }

  // (Re-)populate ROS image message
  // NOTE: the non-ROS UEye parameters and buffers have been updated by setColorMode(),
  // setResolution(), setSubsampling(), setBinning(), and setSensorScaling()
  ros_image_.header.frame_id = "/" + cam_name_;
  ros_image_.height = config.image_height / (config.sensor_scaling * config.subsampling * config.binning);
  ros_image_.width = config.image_width / (config.sensor_scaling * config.subsampling * config.binning);
  ros_image_.encoding = config.color_mode;
  ros_image_.step = cam_buffer_pitch_;
  ros_image_.is_bigendian = 0;
  ros_image_.data.resize(cam_buffer_size_);

  // Check for mutual exclusivity among requested sensor parameters
  if (!config.auto_exposure) { // Auto frame rate requires auto shutter
    config.auto_frame_rate = false;
  }
  if (config.auto_frame_rate) { // Auto frame rate has precedence over auto gain
    config.auto_gain = false;
  }

  // Configure camera sensor parameters
  if (config.auto_gain != cam_params_.auto_gain ||
      config.master_gain != cam_params_.master_gain ||
      config.red_gain != cam_params_.red_gain ||
      config.green_gain != cam_params_.green_gain ||
      config.blue_gain != cam_params_.blue_gain ||
      config.gain_boost != cam_params_.gain_boost) {
    if (setGain(config.auto_gain, config.master_gain,
        config.red_gain, config.green_gain,
        config.blue_gain, config.gain_boost) != IS_SUCCESS) return;
  }

  if (config.pixel_clock != cam_params_.pixel_clock) {
    if (setPixelClockRate(config.pixel_clock) != IS_SUCCESS) return;
  }

  if (config.auto_frame_rate != cam_params_.auto_frame_rate ||
      config.frame_rate != cam_params_.frame_rate) {
    if (setFrameRate(config.auto_frame_rate, config.frame_rate) != IS_SUCCESS) return;
  }

  if (config.auto_exposure != cam_params_.auto_exposure ||
      config.exposure != cam_params_.exposure) {
    if (setExposure(config.auto_exposure, config.exposure) != IS_SUCCESS) return;
  }

  if (config.auto_white_balance != cam_params_.auto_white_balance ||
      config.white_balance_red_offset != cam_params_.white_balance_red_offset ||
      config.white_balance_blue_offset != cam_params_.white_balance_blue_offset) {
    if (setWhiteBalance(config.auto_white_balance, config.white_balance_red_offset,
      config.white_balance_blue_offset) != IS_SUCCESS) return;
  }

  // NOTE: nothing needs to be done for config.ext_trigger_mode, since frame grabber loop will re-initialize to the right setting

  if (config.flash_delay != cam_params_.flash_delay ||
      config.flash_duration != cam_params_.flash_duration) {
    // NOTE: need to copy flash parameters to local copies since
    //       cam_params_.flash_duration is type int, and also sizeof(int)
    //       may not equal to sizeof(INT) / sizeof(UINT)
    INT flash_delay = config.flash_delay;
    UINT flash_duration = config.flash_duration;
    if (setFlashParams(flash_delay, flash_duration) != IS_SUCCESS) return;
    // Copy back actual flash parameter values that were set
    config.flash_delay = flash_delay;
    config.flash_duration = flash_duration;
  }

  // Update local copy of parameter set to newly updated set
  cam_params_ = config;

  // Restart frame grabber if needed
  cfg_sync_requested_ = true;
  if (restartFrameGrabber) {
    startFrameGrabber();
  }
};


INT UEyeCamNodelet::queryCamParams() {
  INT is_err = IS_SUCCESS;
  INT query;
  double pval1, pval2;

  query = is_SetColorMode(cam_handle_, IS_GET_COLOR_MODE);
  if (query == IS_CM_MONO8) cam_params_.color_mode = MONO8;
  else if (query == IS_CM_SENSOR_RAW8) cam_params_.color_mode = BAYER_RGGB8;
  else if (query == IS_CM_RGB8_PACKED) cam_params_.color_mode = RGB8;
  else {
    NODELET_WARN_STREAM("Camera configuration loaded into an unsupported color mode; switching to MONO8.");
    cam_params_.color_mode = MONO8;
    setColorMode(cam_params_.color_mode);
  }

  if ((is_err = is_AOI(cam_handle_, IS_AOI_IMAGE_GET_AOI,
      (void*) &cam_aoi_, sizeof(cam_aoi_))) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Could not retrieve Area Of Interest from UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    disconnectCam();
    return is_err;
  }
  cam_params_.image_width = cam_aoi_.s32Width;
  cam_params_.image_height = cam_aoi_.s32Height;
  cam_params_.image_left = cam_aoi_.s32X;
  cam_params_.image_top = cam_aoi_.s32Y;

  query = is_SetSubSampling(cam_handle_, IS_GET_SUBSAMPLING);
  switch (query) {
    case IS_SUBSAMPLING_DISABLE:
      cam_params_.subsampling = 1;
      break;
    case IS_SUBSAMPLING_2X:
      cam_params_.subsampling = 2;
      break;
    case IS_SUBSAMPLING_4X:
      cam_params_.subsampling = 4;
      break;
    case IS_SUBSAMPLING_8X:
      cam_params_.subsampling = 8;
      break;
    case IS_SUBSAMPLING_16X:
      cam_params_.subsampling = 16;
      break;
    default:
      NODELET_WARN_STREAM("Query returned unsupported subsampling rate; resetting to 1X.");
      cam_params_.subsampling = 1;
      if ((is_err = setSubsampling(cam_params_.subsampling)) != IS_SUCCESS) return is_err;
      break;
  }

  query = is_SetBinning(cam_handle_, IS_GET_BINNING);
  switch (query) {
    case IS_BINNING_DISABLE:
      cam_params_.binning = 1;
      break;
    case IS_BINNING_2X:
      cam_params_.binning = 2;
      break;
    case IS_BINNING_4X:
      cam_params_.binning = 4;
      break;
    case IS_BINNING_8X:
      cam_params_.binning = 8;
      break;
    case IS_BINNING_16X:
      cam_params_.binning = 16;
      break;
    default:
      NODELET_WARN_STREAM("Query returned unsupported binning rate; resetting to 1X.");
      cam_params_.binning = 1;
      if ((is_err = setBinning(cam_params_.binning)) != IS_SUCCESS) return is_err;
      break;
  }

  SENSORSCALERINFO sensorScalerInfo;
  is_err = is_GetSensorScalerInfo(cam_handle_, &sensorScalerInfo, sizeof(sensorScalerInfo));
  if (is_err == IS_NOT_SUPPORTED) {
    cam_params_.sensor_scaling = 1.0;
  } else if (is_err != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query sensor scaler info (" << err2str(is_err) << ")");
    return is_err;
  } else {
    cam_params_.sensor_scaling = sensorScalerInfo.dblCurrFactor;
    if (!(cam_params_.sensor_scaling == 1.0 ||
        cam_params_.sensor_scaling == 2.0 ||
        cam_params_.sensor_scaling == 4.0 ||
        cam_params_.sensor_scaling == 8.0 ||
        cam_params_.sensor_scaling == 16.0)) {
      NODELET_WARN_STREAM("Unsupported sensor scaling rate: " << cam_params_.sensor_scaling <<
          "; resetting to 1X.");
      cam_params_.sensor_scaling = 1.0;
      if ((is_err = setSensorScaling(cam_params_.sensor_scaling)) != IS_SUCCESS) return is_err;
    }
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_GAIN, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_GAIN, &pval1, &pval2)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query auto gain mode for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_gain = (pval1 != 0);

  cam_params_.master_gain = is_SetHardwareGain(cam_handle_, IS_GET_MASTER_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  cam_params_.red_gain = is_SetHardwareGain(cam_handle_, IS_GET_RED_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  cam_params_.green_gain = is_SetHardwareGain(cam_handle_, IS_GET_GREEN_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  cam_params_.blue_gain = is_SetHardwareGain(cam_handle_, IS_GET_BLUE_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);

  query = is_SetGainBoost(cam_handle_, IS_GET_GAINBOOST);
  if (query == IS_SET_GAINBOOST_ON) {
    cam_params_.gain_boost = true;
  } else if (query == IS_SET_GAINBOOST_OFF) {
    cam_params_.gain_boost = false;
  } else {
    NODELET_ERROR_STREAM("Failed to query gain boost for UEye camera '" <<
        cam_name_ << "' (" << err2str(query) << ")");
    return query;
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_SHUTTER, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_SHUTTER, &pval1, &pval2)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query auto shutter mode for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_exposure = (pval1 != 0);

  if ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE,
      &cam_params_.exposure, sizeof(cam_params_.exposure))) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query exposure timing for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_WHITEBALANCE, &pval1, &pval2)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query auto white balance mode for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_white_balance = (pval1 != 0);

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_AUTO_WB_OFFSET, &pval1, &pval2)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query auto white balance red/blue channel offsets for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.white_balance_red_offset = pval1;
  cam_params_.white_balance_blue_offset = pval2;

  IO_FLASH_PARAMS currFlashParams;
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_GET_PARAMS,
      (void*) &currFlashParams, sizeof(IO_FLASH_PARAMS))) != IS_SUCCESS) {
    ERROR_STREAM("Could not retrieve current flash parameter info for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.flash_delay = currFlashParams.s32Delay;
  cam_params_.flash_duration = currFlashParams.u32Duration;

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query auto frame rate mode for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_frame_rate = (pval1 != 0);

  if ((is_err = is_SetFrameRate(cam_handle_, IS_GET_FRAMERATE, &cam_params_.frame_rate)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query frame rate for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }

  UINT currPixelClock;
  if ((is_err = is_PixelClock(cam_handle_, IS_PIXELCLOCK_CMD_GET,
      (void*) &currPixelClock, sizeof(currPixelClock))) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query pixel clock rate for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.pixel_clock = currPixelClock;

  // Populate ROS image message
  // NOTE: the non-ROS UEye parameters and buffers have been updated by setColorMode, setResolution(), and setSubsampling()
  ros_image_.header.frame_id = "/" + cam_name_;
  ros_image_.height = cam_params_.image_height /
      (cam_params_.sensor_scaling * cam_params_.subsampling * cam_params_.binning);
  ros_image_.width = cam_params_.image_width /
      (cam_params_.sensor_scaling * cam_params_.subsampling * cam_params_.binning);
  ros_image_.encoding = cam_params_.color_mode;
  ros_image_.step = cam_buffer_pitch_;
  ros_image_.is_bigendian = 0;
  ros_image_.data.resize(cam_buffer_size_);
  NODELET_INFO_STREAM("Camera Params Width"<<cam_params_.image_width);
  NODELET_INFO_STREAM("Camera Params Height"<<cam_params_.image_height);
  NODELET_INFO_STREAM("Camera Buffer Pitch"<<cam_buffer_pitch_);



  return is_err;
};


INT UEyeCamNodelet::connectCam() {
  INT is_err = IS_SUCCESS;

  if ((is_err = UEyeCamDriver::connectCam()) != IS_SUCCESS) return is_err;

  // (Attempt to) load UEye camera parameter configuration file
  if (cam_params_filename_.length() <= 0) { // Use default filename
    cam_params_filename_ = string(getenv("HOME")) + "/.ros/camera_conf/" + cam_name_ + ".ini";
  }
  loadCamConfig(cam_params_filename_);

  // Query existing configuration parameters from camera
  if ((is_err = queryCamParams()) != IS_SUCCESS) return is_err;

  // Parse and load ROS camera settings
  if ((is_err = parseROSParams(getPrivateNodeHandle())) != IS_SUCCESS) return is_err;

  return IS_SUCCESS;
};


INT UEyeCamNodelet::disconnectCam() {
  INT is_err = IS_SUCCESS;

	if (isConnected()) {
    stopFrameGrabber();
    is_err = UEyeCamDriver::disconnectCam();
	}

  return is_err;
};


bool UEyeCamNodelet::setCamInfo(sensor_msgs::SetCameraInfo::Request& req,
    sensor_msgs::SetCameraInfo::Response& rsp) {
  ros_cam_info_ = req.camera_info;
  ros_cam_info_.header.frame_id = "/" + cam_name_;
  rsp.success = saveIntrinsicsFile();
  rsp.status_message = (rsp.success) ? "successfully wrote to file" : "failed to write to file";
  return true;
};


void UEyeCamNodelet::frameGrabLoop() {
#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
  ros::Time prevStartGrab = ros::Time::now();
  ros::Time prevGrabbedFrame = ros::Time::now();
  ros::Time currStartGrab;
  ros::Time currGrabbedFrame;
  double startGrabSum = 0;
  double grabbedFrameSum = 0;
  double startGrabSumSqrd = 0;
  double grabbedFrameSumSqrd = 0;
  unsigned int startGrabCount = 0;
  unsigned int grabbedFrameCount = 0;
#endif

  ros::Rate idleDelay(200);

  int prevNumSubscribers = 0;
  int currNumSubscribers = 0;
  while (frame_grab_alive_ && ros::ok()) {

    // Initialize live video mode if camera was previously asleep, and ROS image topic has subscribers;
    // and stop live video mode if ROS image topic no longer has any subscribers
    currNumSubscribers = ros_cam_pub_.getNumSubscribers();
    if (currNumSubscribers > 0 && prevNumSubscribers <= 0) {
      INFO_STREAM("New Subscriber detected");
      if (cam_params_.ext_trigger_mode) {
        if (setExtTriggerMode() != IS_SUCCESS) {
          NODELET_ERROR_STREAM("Shutting down UEye camera interface...");
          ros::shutdown();
          return;
        }
        NODELET_INFO_STREAM("Camera " << cam_name_ << " set to external trigger mode");
      } else {
        // NOTE: need to copy flash parameters to local copies since
        //       cam_params_.flash_duration is type int, and also sizeof(int)
        //       may not equal to sizeof(INT) / sizeof(UINT)
        INT flash_delay = cam_params_.flash_delay;
        UINT flash_duration = cam_params_.flash_duration;
        if ((setFreeRunMode() != IS_SUCCESS) ||
            (setFlashParams(flash_delay, flash_duration) != IS_SUCCESS)) {
          NODELET_ERROR_STREAM("Shutting down UEye camera interface...");
          ros::shutdown();
          return;
        }
        // Copy back actual flash parameter values that were set
        cam_params_.flash_delay = flash_delay;
        cam_params_.flash_duration = flash_duration;
        NODELET_INFO_STREAM("Camera " << cam_name_ << " set to free-run mode");
      }
    } else if (currNumSubscribers <= 0 && prevNumSubscribers > 0) {
      if (setStandbyMode() != IS_SUCCESS) {
        NODELET_ERROR_STREAM("Shutting down UEye camera interface...");
        ros::shutdown();
        return;
      }
      NODELET_INFO_STREAM("Camera " << cam_name_ << " set to standby mode");
    }
    prevNumSubscribers = currNumSubscribers;

    // Send updated dyncfg parameters if previously changed
    if (cfg_sync_requested_) {
      if (ros_cfg_mutex_.try_lock()) { // Make sure that dynamic reconfigure server or config callback is not active
        ros_cfg_mutex_.unlock();
        ros_cfg_->updateConfig(cam_params_);
        cfg_sync_requested_ = false;
      }
    }


#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
    startGrabCount++;
    currStartGrab = ros::Time::now();
    if (startGrabCount > 1) {
      startGrabSum += (currStartGrab - prevStartGrab).toSec() * 1000.0;
      startGrabSumSqrd += ((currStartGrab - prevStartGrab).toSec() * 1000.0)*((currStartGrab - prevStartGrab).toSec() * 1000.0);
    }
    prevStartGrab = currStartGrab;
#endif

    if (isCapturing()) {
      INT eventTimeout = (cam_params_.auto_frame_rate || cam_params_.ext_trigger_mode) ?
          (INT) 2000 : (INT) (1000.0 / cam_params_.frame_rate * 2);
      if (processNextFrame(eventTimeout) != NULL) {
        // Process new frame
#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
        grabbedFrameCount++;
        currGrabbedFrame = ros::Time::now();
        if (grabbedFrameCount > 1) {
          grabbedFrameSum += (currGrabbedFrame - prevGrabbedFrame).toSec() * 1000.0;
          grabbedFrameSumSqrd += ((currGrabbedFrame - prevGrabbedFrame).toSec() * 1000.0)*((currGrabbedFrame - prevGrabbedFrame).toSec() * 1000.0);
        }
        prevGrabbedFrame = currGrabbedFrame;

        if (grabbedFrameCount > 1) {
          ROS_WARN_STREAM("\nPre-Grab: " << startGrabSum/startGrabCount << " +/- " <<
              sqrt(startGrabSumSqrd/startGrabCount - (startGrabSum/startGrabCount)*(startGrabSum/startGrabCount)) << " ms (" <<
              1000.0*startGrabCount/startGrabSum << "Hz)\n" <<
              "Post-Grab: " << grabbedFrameSum/grabbedFrameCount << " +/- " <<
              sqrt(grabbedFrameSumSqrd/grabbedFrameCount - (grabbedFrameSum/grabbedFrameCount)*(grabbedFrameSum/grabbedFrameCount)) << " ms (" <<
              1000.0*grabbedFrameCount/grabbedFrameSum << "Hz)\n" <<
              "Target: " << cam_params_.frame_rate << "Hz");
        }
#endif

        if (!frame_grab_alive_ || !ros::ok()) break;

        ////////////////
        if(doRect)
        {
            // set height and width anew.
            ros_cam_info_.height = out_height;
            ros_cam_info_.width = out_width;
            ros_image_.step = out_width;
            ros_image_.width = out_width;
            ros_image_.height = out_height;
            int w = cam_buffer_pitch_;

            for(int idx = out_width*out_height-1;idx>=0;idx--)
            {
                // get interp. values
                float xx = remapX[idx];
                float yy = remapY[idx];

//                printf("xx: %f ",xx);
//                printf("yy: %f \n",yy);

                if(xx<0)
                    ros_image_.data.at(idx) = 0;
                else
                {
                    // get integer and rational parts
                    int xxi = xx;
                    int yyi = yy;
                    xx -= xxi;
                    yy -= yyi;
                    float xxyy = xx*yy;


                    // get array base pointer
                    uchar* src = (uchar*)cam_buffer_ + xxi + yyi * w;

                    // interpolate (bilinear)!
                    ros_image_.data.at(idx) =  xxyy * src[1+w]
                                                + (yy-xxyy) * src[w]
                                                + (xx-xxyy) * src[1]
                                                + (1-xx-yy+xxyy) * src[0];
                }

            }
        }
        else
        {
            ros_cam_info_.height = cam_params_.image_height;
            ros_cam_info_.width = cam_params_.image_width;
            copy((char*) cam_buffer_,
              ((char*) cam_buffer_) + cam_buffer_size_,
              ros_image_.data.begin());
        }
        ////////////////

        ros_image_.header.stamp = ros_cam_info_.header.stamp = ros::Time::now();
        ros_image_.header.seq = ros_cam_info_.header.seq = ros_frame_count_++;

        if (!frame_grab_alive_ || !ros::ok()) break;
        ros_cam_pub_.publish(ros_image_, ros_cam_info_);
      }
    }

    if (!frame_grab_alive_ || !ros::ok()) break;
    idleDelay.sleep();
  }

  setStandbyMode();
  frame_grab_alive_ = false;
  INFO_STREAM("Frame Grabber Thread has stopped");

};


void UEyeCamNodelet::startFrameGrabber() {
  INFO_STREAM("Starting Frame Grabber Thread");
  frame_grab_alive_ = true;
  frame_grab_thread_ = thread(bind(&UEyeCamNodelet::frameGrabLoop, this));
};


void UEyeCamNodelet::stopFrameGrabber() {
  INFO_STREAM("Stopping Frame Grabber Thread");
  frame_grab_alive_ = false;
  if (frame_grab_thread_.joinable()) {
    frame_grab_thread_.join();
  }
  frame_grab_thread_ = thread();
};


void UEyeCamNodelet::loadIntrinsicsFile() {
  if (cam_intr_filename_.length() <= 0) { // Use default filename
    cam_intr_filename_ = string(getenv("HOME")) + "/.ros/camera_info/" + cam_name_ + ".yaml";
  }

  std::string dummyCamName;
  if (camera_calibration_parsers::readCalibrationIni(cam_intr_filename_, dummyCamName, ros_cam_info_)) {
    NODELET_DEBUG_STREAM("Loaded intrinsics parameters for UEye camera " << cam_name_);
  }
  ros_cam_info_.header.frame_id = "/" + cam_name_;
};


bool UEyeCamNodelet::saveIntrinsicsFile() {
  if (camera_calibration_parsers::writeCalibrationIni(cam_intr_filename_, cam_name_, ros_cam_info_)) {
    NODELET_DEBUG_STREAM("Saved intrinsics parameters for UEye camera " << cam_name_ <<
        " to " << cam_intr_filename_);
    return true;
  }
  return false;
};
// TODO: 0 bug where nodelet locks and requires SIGTERM when there are still subscribers (need to find where does code hang)

}; // namespace ueye_cam


// TODO: 9 bug: when binning (and suspect when subsampling / sensor scaling), white balance / color gains seem to have different effects


#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(ueye_cam, ueye_cam_nodelet, ueye_cam::UEyeCamNodelet, nodelet::Nodelet)
