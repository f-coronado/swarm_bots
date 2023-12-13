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
* @file path.hpp
* @author f-coronado
* @brief Containes members of path.hpp
* @date 12/09/2023
*
* @copyright Copyright (c) 2023
*
*/
#pragma once

#include <vector>
#include <string>

class Path {
 private:


    /**
    *@brief private members needed for generate_path function
    */
    struct PathEntry {
        float x_vel;
        float y_vel;
        float z_vel;
        float roll;
        float pitch;
        float yaw;
    };

    /**
    *@brief private members needed for generate_path function
    */
    std::vector<PathEntry> path_;
    std::string shape;

 public:

   /**
   * @brief Generates a path
   *
   * @param shape shape of path we want to generate
   * @return vector with velocities and angles to publish
   */
    std::vector<PathEntry> generate_path(std::string shape);
};
