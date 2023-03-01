#include "Cluster2D.h"


Cluster2D::Cluster2D(int pointNum, float distanceTolerance, int minSize, int maxSize):
    m_pointsNum(pointNum),
    m_distanceTolerance(distanceTolerance),
    m_minClusterSize(minSize),
    m_maxClusterSize(maxSize)
{
    m_processed.assign(m_pointsNum, false);
}

Cluster2D::~Cluster2D() {}


void Cluster2D::clusterHelper(int idx, const std::vector<vector3>& points, std::vector<int>& cluster, KdTree2D* KdTree) {
    m_processed[idx] = true;
    cluster.emplace_back(idx);

    std::vector<int> nearestPoint = KdTree->search(points[idx], m_distanceTolerance);
    for (int nearestId : nearestPoint) {
        if (!m_processed[nearestId]) {
            clusterHelper(nearestId, points, cluster, KdTree);
        }
    }
}


std::vector<std::vector<vector3>> Cluster2D::EuclidCluster(const std::vector<vector3>& points) {
    KdTree2D* KdTree = new KdTree2D;

    for (int idx = 0; idx < m_pointsNum; idx++) {
        KdTree->insert(points[idx], idx);
    }

    for (int idx = 0; idx < m_pointsNum; idx++) {
        if (m_processed[idx]) {
            idx++;
            continue;
        }

        std::vector<int> cluster_idx;
        std::vector<vector3> cluster;
        clusterHelper(idx, points, cluster_idx, KdTree);

        int cluster_size = cluster_idx.size();
        if (cluster_size >= m_minClusterSize && cluster_size <= m_maxClusterSize) {
            for (int i = 0; i < cluster_size; i++) {
                cluster.emplace_back(points[cluster_idx[i]]);
            }
            m_clustersIds.emplace_back(cluster_idx);
            m_clusters.emplace_back(cluster);
        }
    }
    delete KdTree;

    return m_clusters;
}