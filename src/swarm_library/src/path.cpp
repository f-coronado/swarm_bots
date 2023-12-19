/******************************************************************************
 * MIT License
 *
 * Copyright (c) 2022 Fabrizzio Coronado
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

/**
* @file walk_algorithm.cpp
* @author f-coronado
* @brief Walker script
* @date 11/26/2023
*
* @copyright Copyright (c) 2023
*
*/

// openCV stuff
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
// #include <SDL2/SDL.h>

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

// void update(SDL_Renderer* renderer, const std::vector<Robot>& robots, const std::vector<std::vector<std::pair<double, double>>>& rotatedPaths) {
//     SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
//     SDL_RenderClear(renderer);

//     for (size_t i = 0; i < robots.size(); ++i) {
//         int x = static_cast<int>(rotatedPaths[i][0].first * 50 + 200);
//         int y = static_cast<int>(rotatedPaths[i][0].second * 50 + 200);

//         SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
//         SDL_RenderDrawPoint(renderer, x, y);
//     }

//     SDL_RenderPresent(renderer);
// }

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

    std::cout << "rotatedPaths: " << std::endl;
    std::cout << "position[3], orientation[3] " << std::endl;

    for (size_t i = 0; i < rotatedPaths.size(); ++i) {
        std::cout << "Robot " << i << ": ";
        for (size_t j = 0; j < rotatedPaths[i].size(); ++j) {
            std::cout << "(" << rotatedPaths[i][j].first << ", " << rotatedPaths[i][j].second << ") ";
        }
        std::cout << std::endl;
    }



    // SDL Initialization
    // if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    //     std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
    //     return 1;
    // }

    // SDL_Window* window = SDL_CreateWindow("Robot Animation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 400, 400, SDL_WINDOW_SHOWN);
    // if (!window) {
    //     std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
    //     SDL_Quit();
    //     return 1;
    // }

    // SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    // if (!renderer) {
    //     std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
    //     SDL_DestroyWindow(window);
    //     SDL_Quit();
    //     return 1;
    // }

    // bool quit = false;
    // SDL_Event e;

    // while (!quit) {
    //     while (SDL_PollEvent(&e) != 0) {
    //         if (e.type == SDL_QUIT) {
    //             quit = true;
    //         }
    //     }

    //     update(renderer, robots, rotatedPaths);

    //     SDL_Delay(100);  // Adjust the delay to control the animation speed
    // }

    // SDL_DestroyRenderer(renderer);
    // SDL_DestroyWindow(window);
    // SDL_Quit();

    return 0;
}

