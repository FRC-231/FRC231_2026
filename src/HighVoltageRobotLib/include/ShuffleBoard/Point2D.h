#pragma once

#include <memory>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

namespace HighVoltage {
namespace ShuffleBoard {

class Point2D {
public:
    Point2D(double x, double y) : x(x), y(y) {}

    double getX() const { return x; }
    double getY() const { return y; }

    static std::shared_ptr<nt::Value> ToNetworkTableValue(const Point2D& point) {
        auto value = std::make_shared<nt::Value>(nt::Value::MakeDoubleArray({point.getX(), point.getY()}));
        return value;
    }

    static Point2D FromNetworkTableValue(const std::shared_ptr<nt::Value>& value) {
        auto array = value->GetDoubleArray();
        return Point2D(array[0], array[1]);
    }

private:
    double x;
    double y;
};

class Point2DType {
public:
    static const std::shared_ptr<Point2DType>& GetInstance() {
        static auto instance = std::make_shared<Point2DType>();
        return instance;
    }

    Point2DType() = default;

    std::shared_ptr<nt::Value> ToNetworkTableValue(const Point2D& point) const {
        return Point2D::ToNetworkTableValue(point);
    }

    Point2D FromNetworkTableValue(const std::shared_ptr<nt::Value>& value) const {
        return Point2D::FromNetworkTableValue(value);
    }
};

}  // namespace ShuffleBoard
}  // namespace HighVoltage