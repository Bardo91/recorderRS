////////////////////////////////////////////////////////////////
//															  //
//		RGB-D Slam and Active Perception Project			  //
//															  //
//				Author: Pablo R.S. (aka. Bardo91)			  //
//															  //
////////////////////////////////////////////////////////////////

#include "StereoCameraRealSense.h"

#ifdef ENABLE_LIBREALSENSE
	#include <librealsense/rs.hpp>
#endif

#include <pcl/features/integral_image_normal.h>

#include <cstdio>

namespace rgbd {
	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::init(){
		#ifdef ENABLE_LIBREALSENSE
			mRsContext = new rs::context();
			if (mRsContext->get_device_count() == 0) {
				std::cout << "[STEREOCAMERA][REALSENSE] There's no any compatible device connected." << std::endl;
				return false;
			}

			//if (mConfig.contains("cloudDownsampleStep")) {
			//	mDownsampleStep = mConfig["cloudDownsampleStep"];
			//}

			// Get device
			mRsDevice = mRsContext->get_device(0);
			std::cout << "[STEREOCAMERA][REALSENSE] Using device 0, an "<< mRsDevice->get_name() << std::endl;
			std::cout << "[STEREOCAMERA][REALSENSE]     Serial number: " << mRsDevice->get_serial() << std::endl;
			std::cout << "[STEREOCAMERA][REALSENSE]     Firmware version: " << mRsDevice->get_firmware_version() << std::endl;

			// Initialize streams of data.
			mRsDevice->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
			mRsDevice->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
			mRsDevice->start();

			mRsDepthIntrinsic = new rs::intrinsics();
			auto tempDepthIntrinsic = mRsDevice->get_stream_intrinsics(rs::stream::depth);
			memcpy(mRsDepthIntrinsic, &tempDepthIntrinsic, sizeof(rs::intrinsics));
		
			mRsDepthToColor = new rs::extrinsics(); 
			auto tempDepth2Color = mRsDevice->get_extrinsics(rs::stream::depth, rs::stream::color);
			memcpy(mRsDepthToColor, &tempDepth2Color, sizeof(rs::extrinsics));
		

            mRsColorToDepth = new rs::extrinsics();
            auto tempColor2Depth = mRsDevice->get_extrinsics(rs::stream::color, rs::stream::depth);
            memcpy(mRsColorToDepth, &tempColor2Depth, sizeof(rs::extrinsics));

			mRsColorIntrinsic = new rs::intrinsics(); 
			auto tempColorIntrinsic = mRsDevice->get_stream_intrinsics(rs::stream::color);
			memcpy(mRsColorIntrinsic, &tempColorIntrinsic, sizeof(rs::intrinsics));
		
			mRsDepthScale		= mRsDevice->get_depth_scale();


            // Projection matrix Depth
            mCvDepthIntrinsic = cv::Mat::eye(3,3,CV_32F);
            mCvDepthIntrinsic.at<float>(0,0) = mRsDepthIntrinsic->fx;
            mCvDepthIntrinsic.at<float>(1,1) = mRsDepthIntrinsic->fy;
            mCvDepthIntrinsic.at<float>(0,2) = mRsDepthIntrinsic->ppx;
            mCvDepthIntrinsic.at<float>(1,2) = mRsDepthIntrinsic->ppy;

            // Projection matrix Color
            mCvColorIntrinsic= cv::Mat::eye(3,3,CV_32F);
            mCvColorIntrinsic.at<float>(0,0) = mRsColorIntrinsic->fx;
            mCvColorIntrinsic.at<float>(1,1) = mRsColorIntrinsic->fy;
            mCvColorIntrinsic.at<float>(0,2) = mRsColorIntrinsic->ppx;
            mCvColorIntrinsic.at<float>(1,2) = mRsColorIntrinsic->ppy;

            mExtrinsicColorToDepth = cv::Mat::eye(4,4,CV_32F);
            cv::Mat(3,3,CV_32F, &mRsColorToDepth->rotation[0]).copyTo(mExtrinsicColorToDepth(cv::Rect(0,0,3,3)));
            mExtrinsicColorToDepth(cv::Rect(0,0,3,3)) = mExtrinsicColorToDepth(cv::Rect(0,0,3,3)).t(); // RS use color major instead of row mayor.
            cv::Mat(3,1,CV_32F, &mRsColorToDepth->translation[0]).copyTo(mExtrinsicColorToDepth(cv::Rect(3,0,1,3)));

			//mUseUncolorizedPoints = (bool) mConfig["useUncolorizedPoints"];

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::rgb(cv::Mat & _left, cv::Mat & _right){
		#ifdef ENABLE_LIBREALSENSE
            mLastRGB.copyTo(_left);
			return mHasRGB;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::depth(cv::Mat & _depth){
		#ifdef ENABLE_LIBREALSENSE
            mLastDepthInColor.copyTo(_depth);
			return mComputedDepth;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::grab(){
		#ifdef ENABLE_LIBREALSENSE
			mRsDevice->wait_for_frames();

            cv::cvtColor(cv::Mat(mRsColorIntrinsic->height, mRsColorIntrinsic->width, CV_8UC3, (uchar*)mRsDevice->get_frame_data(rs::stream::color)), mLastRGB, CV_RGB2BGR);
			mHasRGB = true;

            mLastDepthInColor = cv::Mat(mRsDepthIntrinsic->height, mRsDepthIntrinsic->width, CV_16U, (uchar*) mRsDevice->get_frame_data(rs::stream::depth_aligned_to_color));
            mComputedDepth = true;

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointXYZ>& _cloud) {
		#ifdef ENABLE_LIBREALSENSE
		for (int dy = 0; dy < mLastRGB.rows; dy = dy + mDownsampleStep) {
			for (int dx = 0; dx < mLastRGB.cols; dx = dx + mDownsampleStep) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = mLastDepthInColor.at<uint16_t>(dy, dx);
					float depth_in_meters = depth_value * mRsDepthScale;

                    // Skip over pixels with a depth value of zero, which is used to indicate no data
                    if (depth_value == 0) {
                        if (mUseUncolorizedPoints) {
                            _cloud.push_back(pcl::PointXYZ(NAN, NAN, NAN));
                        }
                        //else
                            continue;
                    }
                    else {
                        // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                        rs::float2 depth_pixel = { (float)dx, (float)dy };
                        rs::float3 depth_point = mRsColorIntrinsic->deproject(depth_pixel, depth_in_meters);

						_cloud.push_back(pcl::PointXYZ(depth_point.x, depth_point.y, depth_point.z));
					}
				}
			}
			if (mUseUncolorizedPoints)
				setOrganizedAndDense(_cloud);

			return true;
		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointXYZRGB>& _cloud) {
		#ifdef ENABLE_LIBREALSENSE
            for (int dy = 0; dy < mLastRGB.rows; dy = dy + mDownsampleStep) {
                for (int dx = 0; dx < mLastRGB.cols; dx = dx + mDownsampleStep) {
					// Retrieve the 16-bit depth value and map it into a depth in meters
                    uint16_t depth_value = mLastDepthInColor.at<uint16_t>(dy, dx);
					float depth_in_meters = depth_value * mRsDepthScale;
                    // Set invalid pixels with a depth value of zero, which is used to indicate no data
                    pcl::PointXYZRGB point;
                    if (depth_value == 0) {
                        if (mUseUncolorizedPoints) {
                            point.x = NAN;
                            point.y = NAN;
                            point.z = NAN;
                        }
                        else
                            continue;
                    }
                    else {
                        // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                        rs::float2 depth_pixel = { (float)dx, (float)dy };
                        rs::float3 depth_point = mRsColorIntrinsic->deproject(depth_pixel, depth_in_meters);
                        point.x = depth_point.x;
                        point.y = depth_point.y;
                        point.z = depth_point.z;
                        auto rgb = mLastRGB.at<cv::Vec3b>(dy, dx);
                        point.r = rgb[2];
                        point.g = rgb[1];
                        point.b = rgb[0];

                    }

					_cloud.push_back(point);
                }
            }
            if(mUseUncolorizedPoints)
				setOrganizedAndDense(_cloud);
            return true;

		#else
			return false;
		#endif
	}

	//-----------------------------------------------------------------------------------------------------------------
	bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointXYZRGBNormal>& _cloud) {
		if (!mUseUncolorizedPoints) {
			std::cout << "[STEREOCAMERA][REALSENSE] Cannot compute the normals if points out of the colorized region are ignored. Please set the \"UseUncolorizedPoints\" to true in the configuration of the camera" << std::endl;
			return false;
		}
		else {
			pcl::PointCloud<pcl::PointXYZRGB> cloudWoNormals;
			if (!cloud(cloudWoNormals)) {
				return false;
			}

            if(cloudWoNormals.size() == 0){
                std::cout << "[STEREOCAMERA][REALSENSE] Empty cloud, can't compute normals" << std::endl;
                _cloud.resize(0);
                return true;
            }

			//pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
			pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
			ne.setInputCloud(cloudWoNormals.makeShared());
			ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
			ne.setMaxDepthChangeFactor(0.02f);
			ne.setNormalSmoothingSize(10.0f);
			ne.compute(_cloud);

			// Fill XYZ and RGB of cloud
			for (unsigned i = 0; i < _cloud.size(); i++) {
				_cloud[i].x = cloudWoNormals[i].x;
				_cloud[i].y = cloudWoNormals[i].y;
				_cloud[i].z = cloudWoNormals[i].z;
				_cloud[i].r = cloudWoNormals[i].r;
				_cloud[i].g = cloudWoNormals[i].g;
				_cloud[i].b = cloudWoNormals[i].b;
			}

			return true;
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::cloud(pcl::PointCloud<pcl::PointNormal>& _cloud) {
        return false;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::leftCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        mCvColorIntrinsic.copyTo(_intrinsic);
        _coefficients = cv::Mat(1,5, CV_32F);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::rightCalibration(cv::Mat &_intrinsic, cv::Mat &_coefficients) {
        mCvDepthIntrinsic.copyTo(_intrinsic);
        _coefficients = cv::Mat(1,5, CV_32F);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::extrinsic(cv::Mat &_rotation, cv::Mat &_translation) {
        cv::Mat(3,3,CV_32F, &mRsDepthToColor->rotation[0]).copyTo(_rotation);
        cv::Mat(3,1,CV_32F, &mRsDepthToColor->translation[0]).copyTo(_translation);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::extrinsic(Eigen::Matrix3f &_rotation, Eigen::Vector3f &_translation) {
        _rotation = Eigen::Matrix3f(&mRsDepthToColor->rotation[0]);
        _translation = Eigen::Vector3f(&mRsDepthToColor->translation[0]);
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::disparityToDepthParam(double &_dispToDepth){
        _dispToDepth = mRsDepthScale;
        return true;
    }

    //---------------------------------------------------------------------------------------------------------------------
    bool StereoCameraRealSense::colorPixelToPoint(const cv::Point2f &_pixel, cv::Point3f &_point){
        // Retrieve the 16-bit depth value and map it into a depth in meters
        uint16_t depth_value = mLastDepthInColor.at<uint16_t>(_pixel.y, _pixel.x);
        float depth_in_meters = depth_value * mRsDepthScale;
        // Set invalid pixels with a depth value of zero, which is used to indicate no data
        pcl::PointXYZRGB point;
        if (depth_value == 0) {
            return false;
        }
        else {
            // Map from pixel coordinates in the depth image to pixel coordinates in the color image
            rs::float2 depth_pixel = { _pixel.x, _pixel.y };
            rs::float3 depth_point = mRsColorIntrinsic->deproject(depth_pixel, depth_in_meters);

            _point.x = depth_point.x;
            _point.y = depth_point.y;
            _point.z = depth_point.z;
            return true;
        }
    }

    //---------------------------------------------------------------------------------------------------------------------
    cv::Point StereoCameraRealSense::distortPixel(const cv::Point &_point, const rs::intrinsics * const _intrinsics) const {
        float x = (_point.x - _intrinsics->ppx) / _intrinsics->fx;
        float y = (_point.y - _intrinsics->ppy) / _intrinsics->fy;

        float r2  = x*x + y*y;
        float f = 1 + _intrinsics->coeffs[0]*r2 + _intrinsics->coeffs[1]*r2*r2 + _intrinsics->coeffs[4]*r2*r2*r2;
        x *= f;
        y *= f;
        float dx = x + 2*_intrinsics->coeffs[2]*x*y + _intrinsics->coeffs[3]*(r2 + 2*x*x);
        float dy = y + 2*_intrinsics->coeffs[3]*x*y + _intrinsics->coeffs[2]*(r2 + 2*y*y);
        x = dx;
        y = dy;

        cv::Point distortedPixel;
        distortedPixel.x = x * _intrinsics->fx + _intrinsics->ppx;
        distortedPixel.y = y * _intrinsics->fy + _intrinsics->ppy;

        return distortedPixel;
    }

    //---------------------------------------------------------------------------------------------------------------------
    cv::Point StereoCameraRealSense::undistortPixel(const cv::Point &_point,  const rs::intrinsics * const _intrinsics) const {
        float x = (_point.x - _intrinsics->ppx) / _intrinsics->fx;
        float y = (_point.y - _intrinsics->ppy) / _intrinsics->fy;

        float r2  = x*x + y*y;
        float f = 1 + _intrinsics->coeffs[0]*r2 + _intrinsics->coeffs[1]*r2*r2 + _intrinsics->coeffs[4]*r2*r2*r2;
        float ux = x*f + 2*_intrinsics->coeffs[2]*x*y + _intrinsics->coeffs[3]*(r2 + 2*x*x);
        float uy = y*f + 2*_intrinsics->coeffs[3]*x*y + _intrinsics->coeffs[2]*(r2 + 2*y*y);

        cv::Point undistortedPixel;
        undistortedPixel.x = ux* _intrinsics->fx + _intrinsics->ppx;;
        undistortedPixel.y = uy* _intrinsics->fy + _intrinsics->ppy;;

        return undistortedPixel;
    }

}	//	namespace rgbd
