// Line-following robot - control-principle simulation (core model)
// ----------------------------------------------------------------
// A 2D model of the analog line-following car. Two reflective sensors sit at
// the front of the chassis. Each sensor's reading is compared against a
// brightness threshold (the job the LM393 comparator does on the real board)
// to decide "on the line" or "off the line". When the car drifts and one
// sensor slips off the line, the wheel on the side still over the line is eased
// so the car pivots back. That is a closed loop with no microcontroller and no
// firmware on the car itself.
//
// This header holds the pure model: track geometry, the sensor geometry, the
// bang-bang controller and the differential-drive kinematics. The simulator
// (line_follower.cpp) and the headless test (tests/test.cpp) both include it,
// so they exercise the exact same code. It is a software model of the control
// principle, not firmware for a real car.
//
// On the physical board the same decision is made in analog hardware by an
// LM393 comparator; the recovery memory used here when both sensors are off the
// line is a simulation aid the analog board does not have.

#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace lf {

// ----------------------------- Configuration -----------------------------
// Kept identical to the values the original simulation used.

struct Config {
    double width      = 760.0;  // field width  (px)
    double height     = 480.0;  // field height (px)
    double trackWidth = 22.0;   // drawn line thickness (px)

    double carLength    = 34.0; // car body length (px)
    double carWidth     = 22.0; // car body width; also the wheel base for kinematics
    double sensorAhead  = 13.0; // how far in front of the car center the sensors sit
    double sensorSpread = 9.0;  // lateral distance of each sensor from the center line

    double baseSpeed = 1.3;     // forward speed at speed = 1.0 (px / frame)
    double turnInner = 0.0;     // inner-wheel speed factor while steering (0 = that wheel stops)
};

inline const Config& config() {
    static const Config c;
    return c;
}

// Half the line width: a sensor whose center is within this distance of the
// line center is reading "on the line".
inline double half() { return config().trackWidth / 2.0; }

// ------------------------------- Types -----------------------------------

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Car {
    double x     = 0.0;
    double y     = 0.0;
    double theta = 0.0; // heading (rad)
};

struct Sensors {
    Point left;
    Point right;
};

struct Wheels {
    double vL = 0.0;
    double vR = 0.0;
};

// Carries the last steering direction between frames so the car can recover if
// it briefly loses the line on a sharp bend. -1 left, +1 right, 0 straight.
struct Memory {
    int lastTurn = 0;
};

using Track = std::vector<Point>;

// --------------------------- Track definitions ---------------------------
// Each track is a closed polyline. The same polyline is drawn as the line and
// sensed analytically (distance from a sensor to the polyline).

inline Track makeOval(double cx, double cy, double rx, double ry, int n = 600) {
    Track pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i) {
        double t = (static_cast<double>(i) / n) * M_PI * 2.0;
        pts.push_back({cx + rx * std::cos(t), cy + ry * std::sin(t)});
    }
    return pts;
}

inline Track makeWavy(double cx, double cy, double r, double amp, int lobes, int n = 700) {
    Track pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i) {
        double t  = (static_cast<double>(i) / n) * M_PI * 2.0;
        double rr = r + amp * std::sin(lobes * t);
        pts.push_back({cx + rr * std::cos(t), cy + rr * std::sin(t)});
    }
    return pts;
}

inline Track makeFigureEight(double cx, double cy, double a, double b, int n = 800) {
    // Lemniscate-of-Gerono style closed curve that crosses itself in the middle.
    Track pts;
    pts.reserve(n);
    for (int i = 0; i < n; ++i) {
        double t = (static_cast<double>(i) / n) * M_PI * 2.0;
        pts.push_back({cx + a * std::sin(t), cy + b * std::sin(t) * std::cos(t)});
    }
    return pts;
}

inline Track makeTrack(const std::string& name) {
    if (name == "oval")    return makeOval(380, 240, 300, 170);
    if (name == "wavy")    return makeWavy(380, 240, 175, 55, 5);
    if (name == "figure8") return makeFigureEight(380, 240, 310, 200);
    return makeOval(380, 240, 300, 170);
}

// ------------------------------ Geometry ---------------------------------

// World positions of the two sensors given the car pose.
inline Sensors sensorPositions(const Car& car) {
    const Config& cfg = config();
    double c = std::cos(car.theta), s = std::sin(car.theta);
    double ax = car.x + c * cfg.sensorAhead;
    double ay = car.y + s * cfg.sensorAhead;
    double px = -s, py = c; // unit vector along the car's left/right axis
    double d  = cfg.sensorSpread;
    Sensors out;
    out.left  = {ax + px * -d, ay + py * -d};
    out.right = {ax + px * d, ay + py * d};
    return out;
}

// Shortest distance from a point to a closed polyline. This is how a sensor
// reading is computed: the line is dark, so a sensor close to the polyline is
// "on the line".
inline double pointToPolyline(const Point& pt, const Track& poly) {
    double best = std::numeric_limits<double>::infinity();
    size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        const Point& a = poly[i];
        const Point& b = poly[(i + 1) % n];
        double dx = b.x - a.x, dy = b.y - a.y;
        double len2 = dx * dx + dy * dy;
        if (len2 == 0.0) len2 = 1e-9;
        double t = ((pt.x - a.x) * dx + (pt.y - a.y) * dy) / len2;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        double ex = a.x + t * dx - pt.x;
        double ey = a.y + t * dy - pt.y;
        double d2 = ex * ex + ey * ey;
        if (d2 < best) best = d2;
    }
    return std::sqrt(best);
}

// --------------------------- Control + motion ----------------------------

// Bang-bang controller - the heart of the line follower.
//   onLeft / onRight : true when that sensor is over the line.
//   mem              : carries the last steering direction between frames.
// Returns the two wheel speeds.
//
// This mirrors the pair of LM393 comparators on the real board: each sensor's
// brightness is thresholded, and the digital result eases one motor so the car
// pivots back. The recovery action when both sensors are off the line is the
// only part that needs memory and is a simulation aid; the analog board has
// none.
//
// Steering rule: ease the wheel on the side whose sensor is still on the line,
// so the car pivots toward the line. A car always pivots toward its slower
// wheel, so this is the corrective response. On the real board this same action
// comes from cross-wiring - each sensor's comparator drives the opposite motor,
// so a sensor leaving the line cuts the far motor (the one on the still-on
// side).
inline Wheels controller(bool onLeft, bool onRight, Memory& mem, double speed) {
    const Config& cfg = config();
    double base  = cfg.baseSpeed * speed;
    double inner = base * cfg.turnInner;

    if (onLeft && onRight) {
        // Both sensors on the line: centered -> drive straight.
        mem.lastTurn = 0;
        return {base, base};
    }
    if (onLeft && !onRight) {
        // The right sensor slipped off the edge -> the car drifted right and the
        // line is now to its left. Ease the LEFT wheel so the car pivots left,
        // back onto the line.
        mem.lastTurn = -1;
        return {inner, base};
    }
    if (onRight && !onLeft) {
        // Mirror case: left sensor slipped off -> drifted left -> pivot right by
        // easing the RIGHT wheel.
        mem.lastTurn = 1;
        return {base, inner};
    }
    // Both sensors off the line (an overshoot on a tight bend): keep pivoting the
    // way we last pivoted until the line is reacquired.
    if (mem.lastTurn < 0) return {inner, base};
    if (mem.lastTurn > 0) return {base, inner};
    return {base, base};
}

// Differential-drive kinematics: convert wheel speeds into a pose update.
inline void stepKinematics(Car& car, double vL, double vR) {
    const Config& cfg = config();
    double b = cfg.carWidth; // wheel base
    double v = (vL + vR) / 2.0; // forward speed
    // Turn rate: a car pivots toward its slower wheel. The field y-axis points
    // down, so with x += v*cos(theta), y += v*sin(theta) the textbook y-up yaw
    // (vR - vL) is negated to (vL - vR): a faster left wheel (vL > vR) gives
    // omega > 0, rotating the heading from +x toward +y, i.e. toward the slower
    // right wheel.
    double omega = (vL - vR) / b;
    car.theta += omega;
    car.x += v * std::cos(car.theta);
    car.y += v * std::sin(car.theta);
}

// Place the car centered on the first point of a track, heading along it.
inline Car startPose(const Track& poly) {
    const Point& p0 = poly[0];
    const Point& p1 = poly[static_cast<size_t>(poly.size() / 40) + 1];
    Car car;
    car.x = p0.x;
    car.y = p0.y;
    car.theta = std::atan2(p1.y - p0.y, p1.x - p0.x);
    return car;
}

} // namespace lf

#endif // LINE_FOLLOWER_H
