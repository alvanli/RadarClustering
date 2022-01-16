#include "cluster.h"

#include <algorithm>

namespace dbscan {

Cluster::Cluster(std::vector<Point> const& new_points) {
    id_ = current_id;
    points = new_points;

    double total_magnitude = 0;
    double total_azimuth = 0;
    double total_elevation = 0;
    for (auto point:new_points) {
        total_magnitude += point.magnitude();
        total_azimuth += point.azimuth();
        total_elevation += point.elevation();
    }
    int n = new_points.size();
    magnitude_ = total_magnitude/n;
    azimuth_ = total_azimuth/n;
    elevation_ = total_elevation/n;

    current_id++;
}

Cluster::BoundingBox Cluster::ConstructBoundingBox() {
    double x_min, y_min, z_min = std::numeric_limits<double>::max();
    double x_max, y_max, z_max = std::numeric_limits<double>::min();

    for (auto point:points) {
        x_min = std::min(x_min, point.x());
        y_min = std::min(y_min, point.y());
        z_min = std::min(z_min, point.z());
        x_max = std::max(x_max, point.x());
        y_max = std::max(y_max, point.y());
        z_max = std::max(z_max, point.z());
    }

    double x_center, y_center, z_center;
    std::tie(x_center, y_center, z_center) = Centroid();

    double x_dist = x_max - x_min;
    double y_dist = y_max - y_min;
    double z_dist = z_max - z_min;

    Cluster::BoundingBox new_box(x_center, y_center, z_center, x_dist, y_dist, z_dist);
    return new_box;
}

std::tuple<double, double, double> Cluster::Centroid() {
    double x, y, z = 0;
    for (auto point:points) {
        x += point.x();
        y += point.y();
        z += point.z();
    }
    int n = points.size();
    return {x/n, y/n, z/n};
}

double Cluster::IntraClusterDistance() {
    double total_distances = 0;
    int total_comparisons = 0;
    for (int i = 0; i < points.size(); i++) {
        for (int j = i + 1; j < points.size(); j++) {
            total_distances += points[i].EuclideanDistance(points[j]);
            total_comparisons++;
        }
    }
    double avg = total_distances/total_comparisons;
    return avg;
}

int Cluster::id() const {
    return id_;
}

double Cluster::magnitude() const {
    return magnitude_;
}

double Cluster::azimuth() const {
    return azimuth_;
}

double Cluster::elevation() const {
    return elevation_;
}

int Cluster::current_id = 0;

std::ostream& operator<<(std::ostream& out, const Cluster& c) {
    for (auto point:c.points) {
        out << point.x() << "," << point.y() << "," << point.z() << "," << c.id() << "\n";
    }
    return out;
}

}
