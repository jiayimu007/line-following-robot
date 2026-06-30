// Line-following robot - simulation runner
// ----------------------------------------
// Runs the control-principle model in src/line_follower.h on each track and
// writes a trajectory log (one row per frame) so the Python renderer can draw
// the GIF. It is a software model of the analog car's control loop, not
// firmware for a real car.
//
// Each frame: read the two sensors against the line, run the bang-bang
// controller, step the differential-drive kinematics.
//
// Usage:
//   line_follower <track> <steps> [out.csv|out.json]
//     track : oval | wavy | figure8     (default oval)
//     steps : number of frames to simulate (default 1400)
//     out   : output path. A path ending in .json writes the JSON the browser
//             player consumes (track polyline + per-frame trajectory with the
//             two sensor world positions). Any other path, or stdout, writes the
//             CSV the Python renderer reads. If omitted the CSV goes to stdout.
//
// CSV columns: track,frame,x,y,theta,sensorL_on,sensorR_on
//
// The JSON and the CSV are produced by the exact same simulation step, so the
// browser player and the GIF renderer animate the identical trajectory. The
// player does no computation of its own: every number it draws (car pose, both
// sensor dots, the on/off read-out) is written here by the C++ engine.

#include "line_follower.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

namespace {

// One simulated frame, already resolved to the world-space quantities the
// renderers draw: the car pose, both sensor positions, and the on/off states.
struct Frame {
    double x, y, theta;
    double sLx, sLy, sRx, sRy;
    bool onL, onR;
};

// Run one track and return its per-frame trajectory. This is the single source
// of truth for both output formats below; nothing downstream re-derives motion.
std::vector<Frame> run(const std::string& track, int steps) {
    lf::Track poly = lf::makeTrack(track);
    lf::Car car = lf::startPose(poly);
    lf::Memory mem;

    std::vector<Frame> frames;
    frames.reserve(static_cast<size_t>(steps));

    for (int i = 0; i < steps; ++i) {
        lf::Sensors s = lf::sensorPositions(car);
        bool onL = lf::pointToPolyline(s.left, poly) < lf::half();
        bool onR = lf::pointToPolyline(s.right, poly) < lf::half();

        lf::Wheels w = lf::controller(onL, onR, mem, 1.0);
        lf::stepKinematics(car, w.vL, w.vR);

        // Safety net: if the car ever leaves the field, drop it back on the
        // line. This matches the original simulation's reset behavior.
        const lf::Config& cfg = lf::config();
        if (car.x < -20 || car.y < -20 || car.x > cfg.width + 20 || car.y > cfg.height + 20) {
            car = lf::startPose(poly);
            mem = lf::Memory{};
        }

        // Recompute the sensor world positions for the pose we are about to
        // record, so the JSON carries the exact dots the player should draw.
        lf::Sensors sp = lf::sensorPositions(car);
        frames.push_back({car.x, car.y, car.theta,
                          sp.left.x, sp.left.y, sp.right.x, sp.right.y,
                          onL, onR});
    }
    return frames;
}

// Write the CSV the Python renderer reads (one row per frame).
void writeCsv(const std::string& track, const std::vector<Frame>& frames,
              std::ostream& out) {
    out << "track,frame,x,y,theta,sensorL_on,sensorR_on\n";
    int i = 0;
    for (const Frame& f : frames) {
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s,%d,%.4f,%.4f,%.6f,%d,%d\n",
                      track.c_str(), i, f.x, f.y, f.theta,
                      f.onL ? 1 : 0, f.onR ? 1 : 0);
        out << buf;
        ++i;
    }
}

// Write the JSON the browser player consumes: field geometry, the track
// polyline (so the player can draw the line), and the per-frame trajectory.
// Everything here is precomputed by the engine; the player only renders it.
void writeJson(const std::string& track, const std::vector<Frame>& frames,
               std::ostream& out) {
    const lf::Config& cfg = lf::config();
    lf::Track poly = lf::makeTrack(track);

    out << "{\n";
    char hdr[256];
    std::snprintf(hdr, sizeof(hdr),
                  "  \"track\": \"%s\",\n"
                  "  \"width\": %.0f,\n"
                  "  \"height\": %.0f,\n"
                  "  \"trackWidth\": %.0f,\n"
                  "  \"carLength\": %.0f,\n"
                  "  \"carWidth\": %.0f,\n",
                  track.c_str(), cfg.width, cfg.height, cfg.trackWidth,
                  cfg.carLength, cfg.carWidth);
    out << hdr;

    // Track polyline: the line the engine sensed, for the player to draw.
    out << "  \"polyline\": [";
    for (size_t i = 0; i < poly.size(); ++i) {
        char p[64];
        std::snprintf(p, sizeof(p), "%s[%.2f,%.2f]",
                      i ? "," : "", poly[i].x, poly[i].y);
        out << p;
    }
    out << "],\n";

    // Per-frame trajectory. Each entry is fully resolved: the car pose, the two
    // sensor dot positions, and their on/off states. The player draws these
    // directly; it never recomputes a sensor reading or a kinematic step.
    out << "  \"frames\": [\n";
    for (size_t i = 0; i < frames.size(); ++i) {
        const Frame& f = frames[i];
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "    {\"x\":%.3f,\"y\":%.3f,\"theta\":%.5f,"
                      "\"sLx\":%.3f,\"sLy\":%.3f,\"sRx\":%.3f,\"sRy\":%.3f,"
                      "\"onL\":%d,\"onR\":%d}%s\n",
                      f.x, f.y, f.theta, f.sLx, f.sLy, f.sRx, f.sRy,
                      f.onL ? 1 : 0, f.onR ? 1 : 0,
                      (i + 1 < frames.size()) ? "," : "");
        out << buf;
    }
    out << "  ]\n";
    out << "}\n";
}

// True when `path` ends in ".json" (case-sensitive is fine for our own paths).
bool endsWithJson(const std::string& path) {
    const std::string ext = ".json";
    return path.size() >= ext.size() &&
           path.compare(path.size() - ext.size(), ext.size(), ext) == 0;
}

} // namespace

int main(int argc, char** argv) {
    std::string track = (argc > 1) ? argv[1] : "oval";
    int steps = (argc > 2) ? std::atoi(argv[2]) : 1400;
    std::string outPath = (argc > 3) ? argv[3] : "";

    if (track != "oval" && track != "wavy" && track != "figure8") {
        std::cerr << "unknown track: " << track << " (use oval|wavy|figure8)\n";
        return 1;
    }
    if (steps <= 0) {
        std::cerr << "steps must be positive\n";
        return 1;
    }

    std::vector<Frame> frames = run(track, steps);

    if (outPath.empty()) {
        writeCsv(track, frames, std::cout);
    } else {
        std::ofstream f(outPath);
        if (!f) {
            std::cerr << "cannot open output file: " << outPath << "\n";
            return 1;
        }
        if (endsWithJson(outPath)) {
            writeJson(track, frames, f);
        } else {
            writeCsv(track, frames, f);
        }
        std::cerr << "wrote " << steps << " frames for track '" << track
                  << "' to " << outPath << "\n";
    }
    return 0;
}
