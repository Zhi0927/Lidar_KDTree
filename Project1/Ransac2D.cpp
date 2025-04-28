#include "Ransac2D.h"


Ransac2D::Ransac2D(int maxIter, float distanceTol, float minLineLength) :
    m_maxIter(maxIter),
    m_minLineLength(minLineLength),
    m_distanceTol(distanceTol)
{}

Ransac2D::~Ransac2D() {}

std::unordered_set<int> Ransac2D::Ransac(std::vector<vector3>& points) {
    std::unordered_set<int> inliersResult;

    while (m_maxIter--) {
        std::unordered_set<int> inliers;

        while (inliers.size() < 2) {
            inliers.insert(rand() % points.size());
        }

        auto itr = inliers.begin();
        float x1 = points[*itr].x;
        float y1 = points[*itr].y;
        itr++;
        float x2 = points[*itr].x;
        float y2 = points[*itr].y;

        float a = y1 - y2;
        float b = x2 - x1;
        float c = (x1 * y2) - (x2 * y1);

        float sqrt_ab = std::sqrt(a * a + b * b);

        for (size_t index = 0; index < points.size(); index++) {
            if (inliers.count(index) > 0) {
                continue;
            }

            vector3 point = points[index];
            float x = point.x;
            float y = point.y;

            float dist = std::fabs(a * x + b * y + c) / sqrt_ab;

            if (dist < m_distanceTol) {
                inliers.insert(index);
            }
        }

        if (inliers.size() > 1) {
            float maxDist = 0.0f;
            for (auto it1 = inliers.begin(); it1 != inliers.end(); ++it1) {
                for (auto it2 = std::next(it1); it2 != inliers.end(); ++it2) {
                    float dx = points[*it1].x - points[*it2].x;
                    float dy = points[*it1].y - points[*it2].y;
                    float dist = std::sqrt(dx * dx + dy * dy);
                    if (dist > maxDist) {
                        maxDist = dist;
                    }
                }
            }

            if (maxDist < m_minLineLength) {
                continue;
            }
        }

        if (inliers.size() > inliersResult.size()) {
            inliersResult = inliers;
        }
    }

    return inliersResult;
}

std::vector<vector3> Ransac2D::RansacSegment(std::vector<vector3>& points) {
    std::vector<vector3> SegmentPoints;
    SegmentPoints.reserve(points.size()); 

    auto inliersResult = Ransac(points);

    for (size_t i = 0; i < points.size(); ++i) {
        if (inliersResult.count(i) == 0) {
            SegmentPoints.emplace_back(points[i]);
        }
    }

    return SegmentPoints;
}

std::pair<vector3, vector3> Ransac2D::FindFurthestInliers(std::vector<vector3>& points, std::unordered_set<int>& inliers) {
    float maxDist = 0.0f;
    vector3 point1;
    vector3 point2;

    for (auto itr1 = inliers.begin(); itr1 != inliers.end(); ++itr1) {
        auto itr2 = itr1;
        ++itr2;
        for (; itr2 != inliers.end(); ++itr2) {
            const vector3& p1 = points[*itr1];
            const vector3& p2 = points[*itr2];

            float dist = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
            if (dist > maxDist) {
                maxDist = dist;
                point1 = p1;
                point2 = p2;
            }
        }
    }

    return std::make_pair(point1, point2);
}