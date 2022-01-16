#pragma once

#include "point.h"

#include <iostream>
#include <tuple>
#include <vector>

namespace dbscan {

class Cluster {
public:
    struct BoundingBox {
        double x_center, y_center, z_center;
        double x_dist, y_dist, z_dist;
        BoundingBox(double x, double y, double z, double x_dist, double y_dist, double z_dist)
            : x_center(x), y_center(y), z_center(z), x_dist(x_dist), y_dist(y_dist), z_dist(z_dist) {} 
    };

    Cluster(std::vector<Point> const& new_points);

    BoundingBox ConstructBoundingBox();

    std::tuple<double, double, double> Centroid();

    double IntraClusterDistance();

    int id() const;

    double magnitude() const;

    double azimuth() const;

    double elevation() const;

    std::vector<Point> points;

    static int current_id;

    friend std::ostream& operator<<(std::ostream& out, const Cluster& c);

private:
    int id_;
    double magnitude_;
    double azimuth_;
    double elevation_;
};

}
