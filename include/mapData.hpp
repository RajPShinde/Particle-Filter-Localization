#ifndef INCLUDE_MAPDATA_HPP_
#define INCLUDE_MAPDATA_HPP_

class MapData
{
    public:
        double xWidth;
        double yWidth;
        double xMin, xMax;
        double yMin, yMax;
        double resolution;
        std::vector<std::vector<int>> data;
};

#endif  //  INCLUDE_MAPDATA_HPP_