#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <memory>

#include <string>
#include <fstream>

#include "Cluster2D.h"
#include "Ransac2D.h"
#include "Vector3.hpp"

#ifdef _DEBUG
#pragma comment(lib, "opencv_world455d.lib")
#else
#pragma comment(lib, "opencv_world455.lib")
#endif


#define M_DELTA_ANGLE		(M_PI * 2 / 1440) 
#define SCAN_STEP			1081
#define DISTANCE_MAXVALUE	3000

#define VIEW_WIDTH			720
#define VIEW_HEIGHT			720


inline void GetClusterMinMax(const std::vector<vector3>& points, vector3& min_point, vector3& max_point) {
	if (points.empty()) {
		return;
	}

	min_point.setConstant(std::numeric_limits<float>::max());
	max_point.setConstant(std::numeric_limits<float>::lowest());

	for (const auto& point : points) {
		if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
			continue;
		}
		min_point = min_point.cwiseMin(point);
		max_point = max_point.cwiseMax(point);
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

std::vector<std::vector<vector3>> LoadDate(std::string file) {
	auto CacheDirections = []() {
		std::vector<vector3> directions(SCAN_STEP);
		float offset = M_DELTA_ANGLE * 540;
		for (size_t i = 0; i < directions.size(); i++) {
			float a = M_DELTA_ANGLE * i + offset;
			directions[i] = vector3(-cos(a), -sin(a), 0);
		}
		std::cout << "Direction has been calculated!" << std::endl;

		return directions;
	};

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
		std::vector<vector3> oneData(datalist.size());
		float size = std::min(VIEW_WIDTH, VIEW_HEIGHT) / 2;
		for (size_t i = 0; i < datalist.size(); ++i) {
			float dataTrs = std::stof(datalist[i]);
			map(dataTrs, 0, DISTANCE_MAXVALUE, 0, size);
			oneData[i] = lidarDirects[i] * dataTrs + vector3(size, size);
		}
		lidarPoints.emplace_back(oneData);
	}

	ifs.close();
	return lidarPoints;
}

class AdjustableRect
{
public:
	AdjustableRect(const cv::Rect& rect) : m_Rect(rect) {}

	void moveTopCenter(const cv::Point& newTopCenter) {
		int newX = newTopCenter.x - m_Rect.width / 2;
		int newY = newTopCenter.y;

		if (newX < 0) newX = 0;
		if (newY < 0) newY = 0;
		if (newX + m_Rect.width  > VIEW_WIDTH)  newX = VIEW_WIDTH  - m_Rect.width;
		if (newY + m_Rect.height > VIEW_HEIGHT) newY = VIEW_HEIGHT - m_Rect.height;

		m_Rect.x = newX;
		m_Rect.y = newY;
	}

	void setWidth(int newWidth) {
		if (newWidth < 1) newWidth = 1;
		int centerX = m_Rect.x + m_Rect.width / 2;

		m_Rect.width = newWidth;
		m_Rect.x = centerX - m_Rect.width / 2;

		if (m_Rect.x < 0) m_Rect.x = 0;
		if (m_Rect.x + m_Rect.width > VIEW_WIDTH) m_Rect.width = VIEW_WIDTH - m_Rect.x;
	}

	void setHeight(int newHeight) {
		if (newHeight < 1) newHeight = 1;
		m_Rect.height = newHeight;

		if (m_Rect.y < 0) m_Rect.y = 0;
		if (m_Rect.y + m_Rect.height > VIEW_HEIGHT) m_Rect.height = VIEW_HEIGHT - m_Rect.y;
	}

	cv::Point getTopCenter() const {
		return cv::Point(m_Rect.x + m_Rect.width / 2, m_Rect.y);
	}

	cv::Rect getRect() const {
		return m_Rect;
	}

	bool contains(vector3 point) const {
		return (point.x >= m_Rect.x && point.x < m_Rect.x + m_Rect.width &&
				point.y >= m_Rect.y && point.y < m_Rect.y + m_Rect.height);
	}

private:
	cv::Rect m_Rect;
};

struct TrackbarData {
	AdjustableRect* rect;
	int* topCenterX;
	int* topCenterY;
	int* width;
	int* height;
};

void onTrackbar(int, void* userdata) {
	auto* data = reinterpret_cast<TrackbarData*>(userdata);

	data->rect->moveTopCenter(cv::Point2f(*(data->topCenterX), *(data->topCenterY)));
	data->rect->setWidth(*(data->width));
	data->rect->setHeight(*(data->height));
}


int main() {
	cv::Mat img = cv::Mat::zeros(cv::Size(VIEW_WIDTH, VIEW_HEIGHT), CV_8UC3);

	cv::VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(VIEW_WIDTH, VIEW_HEIGHT));

	std::vector<std::vector<vector3>> NFramesPoints = LoadDate("output.txt");
	float size = std::min(VIEW_WIDTH, VIEW_HEIGHT) / 2;
	vector3 centerPoint = vector3(size, size);

	int ROITopCenterX	= centerPoint.x;
	int ROITopCenterY	= centerPoint.y;
	int ROIWidth		= 430;
	int ROIHeight		= 235;
	AdjustableRect ROI(cv::Rect(ROITopCenterX, ROITopCenterY, ROIWidth, ROIHeight));
	TrackbarData data = { &ROI, &ROITopCenterX, &ROITopCenterY, &ROIWidth, &ROIHeight };

	int		ClusterDistTol  = 3;
	int		ClusterMinSize	= 30;
	int		ClusterMaxSize	= 80;

	int		SegMinLength	= 80;
	int		SegDistTol		= 2;
	int		SegMaxIter		= 100;

	//cv::namedWindow("Slider", cv::WINDOW_NORMAL);
	cv::namedWindow("Slider", cv::WINDOW_AUTOSIZE);

	cv::createTrackbar("ROI_TopCenterX",  "Slider",   &ROITopCenterX,   VIEW_WIDTH,  onTrackbar, &data);
	cv::createTrackbar("ROI_TopCenterY",  "Slider",   &ROITopCenterY,   VIEW_HEIGHT, onTrackbar, &data);
	cv::createTrackbar("ROI_Width",		  "Slider",   &ROIWidth,		VIEW_WIDTH,  onTrackbar, &data);
	cv::createTrackbar("ROI_Height",	  "Slider",   &ROIHeight,		VIEW_HEIGHT, onTrackbar, &data);

	cv::createTrackbar("Segment_MaxIter", "Slider",	  &SegMaxIter,   200);
	cv::createTrackbar("Segment_DistTol", "Slider",	  &SegDistTol,   20);
	cv::createTrackbar("Segment_MinLength", "Slider", &SegMinLength, 100);

	cv::createTrackbar("Cluster_DisTol",  "Slider",   &ClusterDistTol, 100);
	cv::createTrackbar("Cluster_MinSize", "Slider",   &ClusterMinSize, 100);
	cv::createTrackbar("Cluster_MaxSize", "Slider",   &ClusterMaxSize, 200);


	bool loop = true;
	while (loop)
	{
		for (const auto& RawPoints : NFramesPoints) {
			cv::Mat ImgClone = img.clone();
			cv::circle(ImgClone, cv::Point2f(centerPoint.x, centerPoint.y), 5, cv::Scalar(0, 255, 0), -1);
			cv::rectangle(ImgClone, ROI.getRect(), cv::Scalar(0, 0, 255), 1);

			for (auto& pt : RawPoints) {
				cv::circle(ImgClone, cv::Point2f(pt.x, pt.y), 1, cv::Scalar(255, 255, 255), -1);
				cv::line(ImgClone, cv::Point2f(pt.x, pt.y), cv::Point2f(centerPoint.x, centerPoint.y), cv::Scalar(100, 100, 100), 1);
			}

			std::vector<vector3> filteredPoints;
			for (const auto& pt : RawPoints) {
				if (ROI.contains(pt)) {
					filteredPoints.emplace_back(pt);
				}
			}

			if (filteredPoints.size() > 3) {
				//for (auto& pt : filteredPoints) {
				//	cv::circle(ImgClone, cv::Point2f(pt.x, pt.y), 1, cv::Scalar(255, 255, 255), -1);
				//	cv::line(ImgClone, cv::Point2f(pt.x, pt.y), cv::Point2f(centerPoint.x, centerPoint.y), cv::Scalar(100, 100, 100), 1);
				//}


				std::unique_ptr<Ransac2D> RansacSegmentLine(new Ransac2D(SegMaxIter, SegDistTol, SegMinLength));
				//auto linePointsIndex = RansacSegmentLine->Ransac(filteredPoints);
				//auto [A, B] = RansacSegmentLine->FindFurthestInliers(filteredPoints, linePointsIndex);
				//for (const int idx : linePointsIndex) {
				//	cv::circle(ImgClone, cv::Point2f(filteredPoints[idx].x, filteredPoints[idx].y), 2, cv::Scalar(255, 0, 0), -1);
				//}
				//cv::line(ImgClone, cv::Point2f(A.x, A.y), cv::Point2f(B.x, B.y), cv::Scalar(255, 255, 0), 1);

				std::vector<vector3> segmentPoints = RansacSegmentLine->RansacSegment(filteredPoints);
				for (const auto& pt : segmentPoints) {
					cv::circle(ImgClone, cv::Point2f(pt.x, pt.y), 1, cv::Scalar(255, 255, 255), -1);
					cv::line(ImgClone, cv::Point2f(pt.x, pt.y), cv::Point2f(centerPoint.x, centerPoint.y), cv::Scalar(100, 100, 100), 1);
				}


				std::unique_ptr<Cluster2D> EuclideanClustering(new Cluster2D(segmentPoints.size(), ClusterDistTol, ClusterMinSize, ClusterMaxSize));
				std::vector<std::vector<vector3>> clustersPoints = EuclideanClustering->EuclidCluster(segmentPoints);

				int clusterId = 0;
				std::vector<cv::Scalar> colors = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255) };
				for (auto clusterPoint : clustersPoints) {
					for (auto& pt : clusterPoint) {
						cv::circle(ImgClone, cv::Point2f(pt[0], pt[1]), 1, colors[clusterId % 3], -1);
					}

					vector3 minPoint, maxPoint;
					GetClusterMinMax(clusterPoint, minPoint, maxPoint);
					vector3 center = (minPoint + maxPoint) / 2;

					cv::rectangle(ImgClone, cv::Point2f(minPoint[0], minPoint[1]), cv::Point2f(maxPoint[0], maxPoint[1]), cv::Scalar(0, 255, 255), 2);
					cv::circle(ImgClone, cv::Point2f(center[0], center[1]), 2, cv::Scalar(0, 0, 255), -1);
					++clusterId;
				}
			}

			cv::imshow("Result", ImgClone);
			video.write(ImgClone);

			if (cv::waitKey(1) == 27) {
				loop = false;
				break;
			}
		}
	}

	cv::destroyAllWindows();
	video.release();

	return 0;
}