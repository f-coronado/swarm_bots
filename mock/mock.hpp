#ifndef MOCK_HPP
#define MOCK_HPP

#include <vector>
#include <utility>

struct Robot {
    double position[3];
    double orientation[3];
};

std::vector<Robot> createRobots(int num);
std::vector<Robot> startPositions(int num);
std::vector<std::pair<double, double>> pathPoints(double rad, int num);
std::vector<std::vector<std::pair<double, double>>> trajectoryDict(const std::vector<std::pair<double, double>>& lst);
void update(SDL_Renderer* renderer, const std::vector<Robot>& robots, const std::vector<std::vector<std::pair<double, double>>>& rotatedPaths);

#endif  // MOCK_HPP
