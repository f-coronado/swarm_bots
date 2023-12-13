#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <SDL2/SDL.h>

struct Robot {
    double position[3];
    double orientation[3];
};

std::vector<Robot> createRobots(int num) {
    std::vector<Robot> robots;
    for (int i = 0; i < num; ++i) {
        Robot robot;
        robot.position[0] = 0.0;
        robot.position[1] = 0.0;
        robot.position[2] = 0.0;
        robot.orientation[0] = 0.0;
        robot.orientation[1] = 0.0;
        robot.orientation[2] = 0.0;
        robots.push_back(robot);
    }
    return robots;
}

std::vector<Robot> startPositions(int num) {
    int matrixSize = std::ceil(std::sqrt(num));
    std::vector<Robot> robotPositions;
    for (int i = 0; i < num; ++i) {
        Robot robot;
        robot.position[0] = static_cast<double>(i / matrixSize);
        robot.position[1] = static_cast<double>(i % matrixSize);
        robot.position[2] = 0.0;
        robot.orientation[0] = 0.0;
        robot.orientation[1] = 0.0;
        robot.orientation[2] = 0.0;
        robotPositions.push_back(robot);
    }
    return robotPositions;
}

std::vector<std::pair<double, double>> pathPoints(double rad, int num) {
    std::vector<std::pair<double, double>> pointsList;
    double thetaStep = 2 * M_PI / num;
    for (int i = 0; i <= num; ++i) {
        double angle = i * thetaStep;
        pointsList.emplace_back(rad * std::cos(angle), rad * std::sin(angle));
    }
    return pointsList;
}

std::vector<std::vector<std::pair<double, double>>> trajectoryDict(const std::vector<std::pair<double, double>>& lst) {
    int rotations = static_cast<int>(lst.size());
    std::vector<std::vector<std::pair<double, double>>> rotatedLists(rotations);

    for (int i = 0; i < rotations; ++i) {
        rotatedLists[i] = lst;
        std::rotate(rotatedLists[i].begin(), rotatedLists[i].begin() + 1, rotatedLists[i].end());
    }
    return rotatedLists;
}

void update(SDL_Renderer* renderer, const std::vector<Robot>& robots, const std::vector<std::vector<std::pair<double, double>>>& rotatedPaths) {
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);

    for (size_t i = 0; i < robots.size(); ++i) {
        int x = static_cast<int>(rotatedPaths[i][0].first * 50 + 200);
        int y = static_cast<int>(rotatedPaths[i][0].second * 50 + 200);

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_RenderDrawPoint(renderer, x, y);
    }

    SDL_RenderPresent(renderer);
}

int main() {
    int numEnvs, circleRad;
    std::cout << "Enter the number of nodes: ";
    std::cin >> numEnvs;
    std::cout << "Enter the radius of the circle: ";
    std::cin >> circleRad;

    std::vector<Robot> robots = createRobots(numEnvs);
    std::vector<Robot> initialPositions = startPositions(numEnvs);

    for (size_t i = 0; i < robots.size(); ++i) {
        robots[i] = initialPositions[i];
    }

    std::vector<std::pair<double, double>> path = pathPoints(circleRad, numEnvs);
    std::vector<std::vector<std::pair<double, double>>> rotatedPaths = trajectoryDict(path);

    // SDL Initialization
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("Robot Animation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 400, 400, SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    bool quit = false;
    SDL_Event e;

    while (!quit) {
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }

        update(renderer, robots, rotatedPaths);

        SDL_Delay(100);  // Adjust the delay to control the animation speed
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}