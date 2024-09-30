#ifndef MESSAGESTRUCT_H
#define MESSAGESTRUCT_H

#include <memory>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>

namespace sensor_msgs {
	struct Stamp {
		double stamp = 0.0;
		uint64_t sec = 0;
		uint64_t nsec = 0;

		Stamp() {}
		Stamp(double _stamp) : stamp(_stamp) {}
		double toSec() { 
			if (stamp > 0.0) return stamp;
			double nsecd = 0.0;
			int n = nsec;
			for (int i = 0; i < 9; ++i) {
				int t = n % 10;
				nsecd = (nsecd + t) * 0.1;
				n /= 10;
			}
			stamp = sec + nsecd;
			return stamp;
		};
	};

	struct Header {
		//int seq;
		Stamp stamp;
		//std::string frame_id;
	};

	// imu
	namespace Imu {
		struct StImu {
			Header header;
			Eigen::Vector3d linear_acceleration;
			Eigen::Vector3d angular_velocity;
			
			StImu() {}
			StImu(const StImu* p) {
				linear_acceleration = p->linear_acceleration;
				angular_velocity = p->angular_velocity;
				header = p->header;
			}
		};

		typedef std::shared_ptr<StImu> ConstPtr;
		typedef std::shared_ptr<StImu> Ptr;
	}

	typedef Imu::ConstPtr ImuConstPtr;

	// points
	namespace PointCloud2 {
		struct Field {
			std::string name;
			int offset;
			int datatype;
			int count;
		};

		struct Point {
			float x, y, z;
		};

		struct StPointCloud {
			Header header;
			//std::vector<Field> fields;
			//pcl::PointCloud< pcl::PointXYZINormal > pcl_pc;
			std::vector<Point> pcl_pc;
			
			StPointCloud() {}
			StPointCloud(const StPointCloud* p) {
				header = p->header;
				//fields = p->fields;
				pcl_pc = p->pcl_pc;
			}
		};

		typedef std::shared_ptr<StPointCloud> ConstPtr;
		typedef std::shared_ptr<StPointCloud> Ptr;
	}

	typedef PointCloud2::ConstPtr PointCloudConstPtr;

	// image
	namespace Image {
		struct StImage {
			Header header;
			cv::Mat imageView;

			StImage() {}

			StImage(const StImage* p) {
				header = p->header;
				p->imageView.copyTo(imageView);
			}
		};

		typedef std::shared_ptr<StImage> ConstPtr;
		typedef std::shared_ptr<StImage> Ptr;
	}

	typedef Image::ConstPtr ImageConstPtr;
	typedef Image::ConstPtr CompressedImageConstPtr;
}

#endif