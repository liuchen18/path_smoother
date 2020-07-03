#ifndef PATH_SMOOTHER_PATH_SMOOTHER_H
#define PATH_SMOOTHER_PATH_SMOOTHER_H

#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>

#include "basic_define/pose2d.h"
#include "basic_define/vec2d.h"
#include "dynamic_voronoi.h"
#include "constants.h"


class PathSmoother {
public:
    /**
     * @brief default constructor
     */
    PathSmoother() {};

    /**
     * @brief default constructor
     * @param map_img the image that contains the map
     * @param cv_path unsmothed path
     */
    PathSmoother(cv::Mat map_img, const std::vector<Vec2d>& cv_path);

    /**
     * @brief smooth the path
     */
    void smoothPath();

    /**
     * @brief get unsmoothed path
     * @return vector of pose2d that contains the unsmoothed path
     */
    std::vector<Pose2d> getOriginalPath() {return original_path_;}

    /**
     * @brief get smoothed path
     * @return vector of pose2d that contains the smoothed path
     */
    std::vector<Pose2d> getSmoothedPath() {return smoothed_path_;}

private:

    /**
     * @brief compute the obstacle term of the given path point. the purpose is to avoid collision
     * @param xi the given path point
     * @return the obstacle term of the point
     */
    Vec2d obstacleTerm(Vec2d xi);

    /**
     * @brief compute the curvature term of the given point. the purpose is to get a drivable path
     * @param xi0 last point
     * @param xi1 current point
     * @param xi2 next point
     * @return curvature term
     */
    Vec2d curvatureTerm(Vec2d xi0, Vec2d xi1, Vec2d xi2);

    /**
     * @brief compute the smoothness term of the given point. the purpose is to get a smooth path
     * @param xim last point
     * @param xi current point
     * @param xip next point
     * @return smoothness term
     */
    Vec2d smoothnessTerm(Vec2d xim, Vec2d xi, Vec2d xip);

    /**
     * @brief compute the voronoi termof the given point. the purpose is to move away from the obstacle
     * @param xi the given point
     * @return voronoi term
     */
    Vec2d voronoiTerm(Vec2d xi);

    /**
     * @brief check whether the point is on the map
     * @param vec the given point
     * @return true if on the map
     */
    bool isOnGrid(Vec2d vec);

    /// maximum possible curvature of the non-holonomic vehicle
    float kappaMax_ = 1.f / (Constants::min_turn_radius * 1.1);
    /// maximum distance to obstacles that is penalized
    float obsDMax_ = 5*Constants::minRoadWidth;
    /// maximum distance for obstacles to influence the voronoi field
    float vorObsDMax_ = 5*Constants::minRoadWidth;

    /// falloff rate for the voronoi field
    float alpha_ = 0.1;
    ///obstacle term weight
    float wObstacle_ = 0.2;
    ///voronoi term weight
    float wVoronoi_ = 0.2;
    ///curvature term weight
    float wCurvature_ = 0.2;
    ///smoothness term weight
    float wSmoothness_ = 0.2;

    DynamicVoronoi voronoi_; ///voronoi diagram of the map
    cv::Mat map_img_; /// the input map
    int map_width_;
    int map_height_;
    std::vector<Pose2d> original_path_;
    std::vector<Pose2d> smoothed_path_;
};

#endif // PATH_SMOOTHER_PATH_SMOOTHER_H
