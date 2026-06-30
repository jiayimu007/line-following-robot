// Headless test for the line-follower control model.
// ---------------------------------------------------
// Runs the exact controller + kinematics from src/line_follower.h against each
// track, sensing the line analytically (distance to the track polyline), and
// checks that the car keeps following the line instead of drifting off. It then
// locks the turn-direction handedness with world-coordinate assertions.
//
// Build + run:
//   c++ -O2 -std=c++17 tests/test.cpp -o build/test && ./build/test

#include "../src/line_follower.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace {

int g_failed = 0;

void check(bool cond, const std::string& msg) {
    std::printf("%s  %s\n", cond ? "PASS" : "FAIL", msg.c_str());
    if (!cond) ++g_failed;
}

struct TrackResult {
    double maxDev = 0.0;  // worst center-of-car deviation from the line (after settling)
    double pathLen = 0.0; // total distance travelled
    bool left = false;    // ever leave the field?
};

TrackResult runTrack(const std::string& name, int steps = 6000, int settle = 400) {
    lf::Track poly = lf::makeTrack(name);
    lf::Car car = lf::startPose(poly);
    lf::Memory mem;

    TrackResult r;
    double px = car.x, py = car.y;
    const lf::Config& cfg = lf::config();

    for (int i = 0; i < steps; ++i) {
        lf::Sensors s = lf::sensorPositions(car);
        bool senseL = lf::pointToPolyline(s.left, poly) < lf::half();
        bool senseR = lf::pointToPolyline(s.right, poly) < lf::half();
        lf::Wheels w = lf::controller(senseL, senseR, mem, 1.0);
        lf::stepKinematics(car, w.vL, w.vR);

        r.pathLen += std::hypot(car.x - px, car.y - py);
        px = car.x;
        py = car.y;

        if (car.x < -20 || car.y < -20 || car.x > cfg.width + 20 || car.y > cfg.height + 20) {
            r.left = true;
        }
        if (i > settle) {
            double dev = lf::pointToPolyline({car.x, car.y}, poly);
            if (dev > r.maxDev) r.maxDev = dev;
        }
    }
    return r;
}

// Drive a fixed wheel differential for n steps and return the world pose, so a
// flipped yaw sign cannot hide behind a frame-agnostic following test.
lf::Car drive(double vL, double vR, int n) {
    lf::Car car; // x=0,y=0,theta=0: heading +x; on this y-down field +y is the car's right
    for (int i = 0; i < n; ++i) lf::stepKinematics(car, vL, vR);
    return car;
}

} // namespace

int main() {
    const double DEV_LIMIT = 10.0; // px - center must stay within this band of the line
    const double MIN_PATH = 1500.0; // px - must actually drive a meaningful distance

    const char* tracks[] = {"oval", "wavy", "figure8"};
    for (const char* name : tracks) {
        TrackResult r = runTrack(name);
        bool ok = !r.left && r.maxDev <= DEV_LIMIT && r.pathLen >= MIN_PATH;
        if (!ok) ++g_failed;
        char line[160];
        std::snprintf(line, sizeof(line),
                      "%s  %-8s  maxDev=%.1fpx  path=%.0fpx  leftField=%s",
                      ok ? "PASS" : "FAIL", name, r.maxDev, r.pathLen,
                      r.left ? "true" : "false");
        std::printf("%s\n", line);
    }

    // --- turn-direction semantics (a car must pivot toward its slower wheel) ---
    // The track-following tests above are frame-agnostic: a mirror-inverted model
    // would follow all three tracks with the same deviations. The checks below
    // are the only guard against silently re-introducing the mirror (a
    // compensating flip of the yaw sign and the sensor axis). They assert
    // absolute, integrated motion in world coordinates, so even a double-flip is
    // caught.
    lf::Car rE = drive(1, 0, 12); // left wheel faster -> must curve toward the slower right (+y)
    check(rE.y > 0 && rE.theta > 0,
          "faster left wheel -> car curves toward its right (+y, world)");
    lf::Car lE = drive(0, 1, 12); // right wheel faster -> must curve toward the slower left (-y)
    check(lE.y < 0 && lE.theta < 0,
          "faster right wheel -> car curves toward its left (-y, world)");

    // The controller must ease the wheel on the side whose sensor is still on the line.
    lf::Memory m0;
    lf::Wheels cL = lf::controller(true, false, m0, 1.0); // left sensor on, right off
    check(cL.vL < cL.vR,
          "left sensor still on -> left wheel eased (car pivots toward the line)");
    lf::Memory m1;
    lf::Wheels cR = lf::controller(false, true, m1, 1.0); // right sensor on, left off
    check(cR.vR < cR.vL,
          "right sensor still on -> right wheel eased (car pivots toward the line)");

    if (g_failed) {
        std::fprintf(stderr, "\n%d check(s) failed.\n", g_failed);
        return 1;
    }
    std::printf("\nAll tracks followed the line within tolerance; turn semantics correct.\n");
    return 0;
}
