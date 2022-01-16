#pragma once

#include <cmath>

namespace dbscan {

class Point {
public:
    enum class PointState {
        unclassified,
        noise,
        classified
    };

    struct Velocity {
        double magnitude;
        double azimuth;
        double elevation;
    };

    Point(double x, double y, double z);

    double EuclideanDistance(Point const& p) const;

    bool IsClassified() const;

    double x() const;

    double y() const;

    double z() const;

    double magnitude() const;

    double azimuth() const;

    double elevation() const;

    PointState state;

private:
    double x_, y_, z_;
    double magnitude_;
    double azimuth_;
    double elevation_;
};

}

