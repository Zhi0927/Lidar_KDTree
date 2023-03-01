#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <memory>

#include <string>
#include <fstream>

#include "Cluster2D.h"
#include "Vector3.hpp"

#ifdef _DEBUG
#pragma comment(lib, "opencv_world455d.lib")
#else
#pragma comment(lib, "opencv_world455.lib")
#endif


#define M_DELTA_ANGLE		(M_PI * 2 / 1440) 
#define SCAN_STEP			1081
#define DISTANCE_MAXVALUE	3500

#define VIEW_WIDTH			720
#define VIEW_HEIGHT			720


inline void getClusterMinMax(const std::vector<vector3>& points, vector3& min_point, vector3& max_point) {
	min_point.setConstant(std::numeric_limits<float>::max());
	max_point.setConstant(std::numeric_limits<float>::lowest());

	if (!points.empty()) {
		for (const auto& point : points) {
			if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
				continue;
			}
			min_point = min_point.cwiseMin(point);
			max_point = max_point.cwiseMax(point);
		}
	}
}

inline std::vector<std::string> split(const std::string& srcstr, const std::string& delimeter) {
	std::vector<std::string> ret(0);
	if (srcstr.empty())
	{
		return ret;
	}
	std::string::size_type pos_begin = srcstr.find_first_not_of(delimeter);

	std::string::size_type dlm_pos;
	std::string temp;
	while (pos_begin != std::string::npos) {
		dlm_pos = srcstr.find(delimeter, pos_begin);
		if (dlm_pos != std::string::npos) {
			temp = srcstr.substr(pos_begin, dlm_pos - pos_begin);
			pos_begin = dlm_pos + delimeter.length();
		}
		else {
			temp = srcstr.substr(pos_begin);
			pos_begin = dlm_pos;
		}
		if (!temp.empty())
			ret.push_back(temp);
	}
	return ret;
}

inline void map(float& value, const float& fromsource, const float& tosource, const float& fromtarget, const float& totarget) {
	value = (value - fromsource) / (tosource - fromsource) * (totarget - fromtarget) + fromtarget;
}

std::vector<vector3> CacheDirections() {
	std::vector<vector3> directions(SCAN_STEP);
	float offset = M_DELTA_ANGLE * 540;
	for (size_t i = 0; i < directions.size(); i++) {
		float a = M_DELTA_ANGLE * i + offset;
		directions[i] = vector3(-cos(a), -sin(a), 0);
	}
	std::cout << "Direction has been calculated!" << std::endl;

	return directions;
}

std::vector<std::vector<vector3>> LoadDate(std::string file) {
	std::ifstream ifs(file, std::ios::in);

	std::vector<vector3> lidarDirects = CacheDirections();
	std::vector<std::vector<vector3>> lidarPoints;

	if (!ifs.is_open()) {
		std::cerr << "ERROR: can't read file with class names." << std::endl;
		return lidarPoints;
	}
	
	std::string onelineData;
	while (std::getline(ifs, onelineData)) {
		std::vector<std::string> datalist = split(onelineData, ";");
		std::vector<vector3> ondData(datalist.size());
		for (size_t i = 0; i < datalist.size(); ++i) {
			float dataTrs = std::stof(datalist[i]);
			float size = std::min(VIEW_WIDTH, VIEW_HEIGHT) / 2;
			map(dataTrs, 0, DISTANCE_MAXVALUE, 0, size);
			ondData[i] = lidarDirects[i] * dataTrs + vector3(size, size);
		}
		lidarPoints.emplace_back(ondData);
	}

	ifs.close();
	return lidarPoints;
}


int main() {
	cv::Mat img = cv::Mat::zeros(cv::Size(VIEW_WIDTH, VIEW_HEIGHT), CV_8UC3);

	cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(VIEW_WIDTH, VIEW_HEIGHT));
	std::vector<std::vector<vector3>> NFramesPoints = LoadDate("output.txt");

	int distanceTolerance = 1;
	int min_size = 5;
	int max_size = 150;
	cv::namedWindow("Slider", cv::WINDOW_AUTOSIZE);
	cv::createTrackbar("DistanceTolerance", "Slider", &distanceTolerance, 100);
	cv::createTrackbar("Min_size", "Slider", &min_size,	100);
	cv::createTrackbar("Max_size", "Slider", &max_size,	50);


	for (const auto& points : NFramesPoints) {
		cv::Mat ImgClone = img.clone();
		float size = std::min(VIEW_WIDTH, VIEW_HEIGHT) / 2;
		cv::circle(ImgClone, cv::Point2f(size, size), 3, cv::Scalar(0, 255, 0), -1);
		for (auto& pt : points) {
			cv::circle(ImgClone, cv::Point2f(pt.x, pt.y), 1, cv::Scalar(255, 255, 255), -1);
		}

		std::unique_ptr<Cluster2D> pointProcessor(new Cluster2D(points.size(), distanceTolerance, min_size, max_size));
		std::vector<std::vector<vector3>> clustersPoints = pointProcessor->EuclidCluster(points);

		//int clusterId = 0;
		//std::vector<cv::Scalar> colors = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255) };
		
		for (auto clusterPoint : clustersPoints) {
			//for (auto point : clusterPoint) {
			//	cv::circle(ImgClone, cv::Point2f(point[0], point[1]), 1, colors[clusterId % 3], -1);
			//}

			vector3 minPoint, maxPoint;
			getClusterMinMax(clusterPoint, minPoint, maxPoint);

			cv::rectangle(ImgClone, cv::Point2f(minPoint[0], minPoint[1]), cv::Point2f(maxPoint[0], maxPoint[1]), cv::Scalar(0, 255, 255), 2);
			
			vector3 center = (minPoint + maxPoint) / 2;
			cv::circle(ImgClone, cv::Point2f(center[0], center[1]), 2, cv::Scalar(0, 0, 255), -1);
			//++clusterId;
		}


		cv::imshow("Result", ImgClone);
		video.write(ImgClone);
		if (cv::waitKey(1) == 27) {
			break;
		}
	}

	cv::destroyAllWindows();
	video.release();

	return 0;
}