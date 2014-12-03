#ifndef WHEATLEY_COMMON_ANGLES_H
#define WHEATLEY_COMMON_ANGLES_H

#include <ros/ros.h>
#include <angles/angles.h>
#define _USE_MATH_DEFINES
#include <cmath>

namespace angles
{
    double angleOf(double x, double y)
    {
        return std::atan2(y, x);
    }

    double angleOf(geometry_msgs::Point from, geometry_msgs::Point to)
    {
        return angleOf(to.x-from.x, to.y-from.y);
    }

    class StraightAngle
    {
    public:
        static const StraightAngle RIGHT;
        static const StraightAngle UP;
        static const StraightAngle LEFT;
        static const StraightAngle DOWN;
        static const StraightAngle ANY;
        static const StraightAngle rotations[4];

    private:
        double angle_;
        std::string text_;

    private:
        StraightAngle(double angle, std::string text)
            : angle_(angle)
            , text_(text)
        {}
    public:
        double angle() const
        {
            return angle_;
        }
        std::string text() const
        {
            return text_;
        }

    public:
        /* returns the closest of the angles [0, PI/2, PI, PI/2*3] */
        static StraightAngle getClosest(double angle)
        {
            angle = normalize_angle_positive(angle);
            if (angle > M_PI*7/4)
                return RIGHT;
            else if (angle > M_PI*5/4)
                return DOWN;
            else if (angle > M_PI*3/4)
                return LEFT;
            else if (angle > M_PI*1/4)
                return UP;
            else
                return RIGHT;
        }

        static StraightAngle getClosest(double x, double y)
        {
            if (x == 0 && y == 0)
                return ANY;
            else
                return getClosest(angleOf(x, y));
        }

        static StraightAngle getClosest(geometry_msgs::Point from, geometry_msgs::Point to)
        {
            return getClosest(angleOf(from, to));
        }

        bool operator==(const StraightAngle& o) const
        {
            return angle_ == o.angle_;
        }

        bool operator!=(const StraightAngle& o) const
        {
            return !operator==(o);
        }

        friend std::ostream& operator<<(std::ostream& stream, const StraightAngle& o)
        {
            return stream << o.text_;
        }
    };

    const StraightAngle StraightAngle::RIGHT( 0     , "right");
    const StraightAngle StraightAngle::UP   ( M_PI/2, "up");
    const StraightAngle StraightAngle::LEFT ( M_PI  , "left");
    const StraightAngle StraightAngle::DOWN (-M_PI/2, "down");
    const StraightAngle StraightAngle::ANY  (-10    , "any");
    const StraightAngle StraightAngle::rotations[] = {StraightAngle::RIGHT, StraightAngle::UP, StraightAngle::LEFT, StraightAngle::DOWN};
}

#endif
