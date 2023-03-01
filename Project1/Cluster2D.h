#ifndef ZHI_CLUSTER2D_H_
#define ZHI_CLUSTER2D_H_

#include <chrono>
#include <string>
#include <vector>
#include "KdTree2D.hpp"
#include "Vector3.hpp"


class Cluster2D {
public:
    Cluster2D(int pointNum, float distanceTolerance, int minSize, int maxSize);
    ~Cluster2D();

    std::vector<std::vector<vector3>> EuclidCluster(const std::vector<vector3>& points);

private:
    void clusterHelper(int idx, const std::vector<vector3>& points, std::vector<int>& cluster, KdTree2D* KdTree);

private:
    int                                 m_pointsNum;
    float                               m_distanceTolerance;
    int                                 m_minClusterSize;
    int                                 m_maxClusterSize;
    std::vector<bool>                   m_processed;
    std::vector<std::vector<int>>       m_clustersIds;
    std::vector<std::vector<vector3>>   m_clusters;
};
#endif
