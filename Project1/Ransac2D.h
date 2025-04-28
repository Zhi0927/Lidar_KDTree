#ifndef ZHI_RANSAC2D_H_
#define ZHI_RANSAC2D_H_

#include <unordered_set>
#include "Vector3.hpp"

class Ransac2D {
public:
    Ransac2D(int maxIter, float distanceTol, float minLineLength);
    ~Ransac2D();

    std::unordered_set<int> Ransac(std::vector<vector3>& points);
    std::vector<vector3> RansacSegment(std::vector<vector3>& points);
    std::pair<vector3, vector3> FindFurthestInliers(std::vector<vector3>& points, std::unordered_set<int>& inliers);

private:
    int     m_maxIter;
    float   m_minLineLength;
    float   m_distanceTol;
};


#endif
