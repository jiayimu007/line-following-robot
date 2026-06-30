# Build the C++ simulation core and its headless test.
#
#   make            build the simulator and the test into build/
#   make sim        build just the simulator (build/line_follower)
#   make test       build and run the headless test
#   make data       run the simulator to produce the trajectory CSVs in build/
#   make web        run the simulator to produce web/data/*.json for the player
#   make gif        produce media/demo.gif and media/demo-figure8.gif
#   make clean      remove build/ artifacts

CXX      ?= c++
CXXFLAGS := -O2 -std=c++17 -Wall -Wextra
PYTHON   ?= python3

BUILD   := build
WEBDATA := web/data
FRAMES  := 1400

.PHONY: all sim test data web gif clean

all: sim $(BUILD)/test

sim: $(BUILD)/line_follower

$(BUILD)/line_follower: src/line_follower.cpp src/line_follower.h | $(BUILD)
	$(CXX) $(CXXFLAGS) src/line_follower.cpp -o $@

$(BUILD)/test: tests/test.cpp src/line_follower.h | $(BUILD)
	$(CXX) $(CXXFLAGS) tests/test.cpp -o $@

test: $(BUILD)/test
	./$(BUILD)/test

# Trajectory logs for the renderer. The oval and figure-8 feed the two GIFs.
data: sim | $(BUILD)
	./$(BUILD)/line_follower oval    $(FRAMES) $(BUILD)/traj-oval.csv
	./$(BUILD)/line_follower figure8 $(FRAMES) $(BUILD)/traj-figure8.csv

# Trajectory JSON for the browser player. One file per track: the same C++
# engine writes the track polyline and the per-frame trajectory the player
# animates. The player computes nothing; it only draws these numbers.
web: sim
	mkdir -p $(WEBDATA)
	./$(BUILD)/line_follower oval    $(FRAMES) $(WEBDATA)/oval.json
	./$(BUILD)/line_follower wavy    $(FRAMES) $(WEBDATA)/wavy.json
	./$(BUILD)/line_follower figure8 $(FRAMES) $(WEBDATA)/figure8.json

gif: data
	$(PYTHON) render.py $(BUILD)/traj-oval.csv    media/demo.gif
	$(PYTHON) render.py $(BUILD)/traj-figure8.csv media/demo-figure8.gif

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
