/**
 * 原始代码：http://www2.informatik.uni-freiburg.de/~lau/dynamicvoronoi/
 * ROS版本：https://github.com/frontw/dynamicvoronoi
 * 参考文献：B. Lau, C. Sprunk and W. Burgard, Improved Updating of Euclidean Distance Maps and Voronoi Diagrams,
 *         IEEE Intl. Conf. on Intelligent Robots and Systems (IROS), Taipei, Taiwan, 2010.
 */

#ifndef PATH_SMOOTHER_DYNAMIC_VORONOI_H
#define PATH_SMOOTHER_DYNAMIC_VORONOI_H


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>
#include <unordered_map>
#include <opencv2/opencv.hpp>

#include "basic_define/vec2i.h"
#include "basic_define/bucket_queue.h"
#include "basic_define/vec2d.h"


class DynamicVoronoi {
public:
    /**
     * @brief default constructor
     */
    DynamicVoronoi();

    /**
     * @brief default destructor
     */
    ~DynamicVoronoi();

    /**
     * @brief build voronoi graph from the given image
     * @param map_img given image
     */
    void buildVoronoiFromImage(const cv::Mat& map_img);

    /**
     * @brief get the distance from the given position to the closed obstacle
     * @param x position x
     * @param y position y
     * @return distance
     */
    float getDistance(int x, int y);

    /**
     * @brief check whether the given position is part of the pruned voronoi graph
     * @param x position x
     * @param y position y
     * @return true if is
     */
    inline bool isVoronoi(int x, int y);

    /**
     * @brief check whether the given position is obstacle
     * @param x position x
     * @param y position y
     * @return true if is obstacle
     */
    inline bool isOccupied(int x, int y);

    /**
     * @brief write the map to a ppm file
     * @param filename
     */
    void visualize(const char* filename="result.ppm");

    /**
     * @brief get the edge points of the voronoi graph
     * @return all edge points
     */
    std::vector<Vec2i> GetVoronoiEdgePoints() const {
        return edge_points_;
    };

    /**
     * @brief get closest voronoi edge point of the given point
     * @param xi given point
     * @param closest_dis
     * @return closed edge point
     */
    Vec2i GetClosestVoronoiEdgePoint(Vec2d xi, double& closest_dis);

    /**
     * @brief get closed obstacle coor of the given point
     * @param p given point
     * @return
     */
    Vec2i GetClosetObstacleCoor(const Vec2d& p) const;

    /**
     * @brief get width of the map
     * @return
     */
    unsigned int getSizeX() const {return sizeX;}

    /**
     * get height of the map
     * @return
     */
    unsigned int getSizeY() const {return sizeY;}

private:
    struct dataCell {
        float dist; ///distance to the closed obstacle
        char voronoi;/// the value of the enum type state
        char queueing;///the value of the enum type queueingstate
        int obstX; /// position x of the closed obstacle
        int obstY; ///position y of the closed obstacle
        bool needsRaise;
        int sqdist;
    };


    typedef enum {voronoiKeep = -4, freeQueued = -3, voronoiRetry = -2, voronoiPrune = -1, free = 0, occupied = 1} State;
    typedef enum {fwNotQueued = 1, fwQueued = 2, fwProcessed = 3, bwQueued = 4, bwProcessed = 1} QueueingState;
    typedef enum {invalidObstData = SHRT_MAX / 2} ObstDataState;
    typedef enum {pruned, keep, retry} markerMatchResult;

    /**
     * @brief ask for space to save data. if there is data, clear it.
     * @param _sizeX width of the map
     * @param _sizeY height of the map
     * @param initGridMap
     */
    void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap = true);

    /**
     * @brief @brief initialize according to the map
     * @param _sizeX width of the map
     * @param _sizeY height of the map
     * @param _gridMap the binary map
     */
    void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

    /**
     * @brief occupy the geiven position as obstacle,update the gridmap and data at the same time
     * @param x position x
     * @param y position y
     */
    inline void occupyCell(int x, int y);

    /**
     * @brief remove the given obstacle and set the position to free,update the gridmap and data at the same time
     * @param x
     * @param y
     */
    inline void clearCell(int x, int y);

    /**
     * @brief change the old obstacle list to the new obstacle list
     * @param newObstacles
     */
    void exchangeObstacles(std::vector<Vec2i> newObstacles);

    /**
     * @brief update the distance map and voronoi diagram
     * @param updateRealDist
     */
    void update(bool updateRealDist = true);

    /**
     * @brief prune the Voronoi diagram
     */
    void prune();

    /**
     *
     */
    void CollectVoronoiEdgePoints();

    /**
     * @brief set the given position to obstacle. update the data only
     * @param x
     * @param y
     */
    inline void setObstacle(int x, int y);

    /**
     * @brief remove the obstacle of the given position,update the data only
     * @param x
     * @param y
     */
    inline void removeObstacle(int x, int y);

    /**
     * @brief check whetner the position x,y is on  the edge of (obstacle nx,ny)
     * @param x position x
     * @param y position y
     * @param nx position nx
     * @param ny position ny
     * @param c cell of the position x,y
     * @param nc cell of the position nx,ny
     */
    inline void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);

    /**
     *@brief deal with the addlist and removelist
     * @param updateRealDist
     */
    void commitAndColorize(bool updateRealDist = true);

    /**
     *
     * @param x
     * @param y
     */
    void reviveVoroNeighbors(int& x, int& y);

    /**
     *@brief check whether the closed obstacle of datacell c is the given position
     * @param x position x
     * @param y position y
     * @param c data cell c
     * @return true if is
     */
    inline bool isOccupied(int& x, int& y, dataCell& c);

    /**
     *
     * @param x
     * @param y
     * @return
     */
    markerMatchResult markerMatch(int x, int y);

    std::string ComputeIndex(const Vec2i& pi) const;
    std::string ComputeIndex(const Vec2d& pd) const;

    ///the queue that contains the grids need to be visited
    BucketPrioQueue open;
    ///the queue that contains the grids need to be pruned
    std::queue<Vec2i> pruneQueue;
    /// the grids that are not going to be occupied by obstacles
    std::vector<Vec2i> removeList;
    ///the grids that are going to be occupied by obstacles. usually and the boundary of the obstacles
    std::vector<Vec2i> addList;
    /// the grid that are added last time
    std::vector<Vec2i> lastObstacles;

    int sizeY;
    int sizeX;
    dataCell** data;
    bool** gridMap;   //true is occupied, false is not
    std::vector<Vec2i> edge_points_;
    std::unordered_map<std::string, std::pair<Vec2i, float>> closest_edge_points_;
};


#endif  // PATH_SMOOTHER_DYNAMIC_VORONOI_H

