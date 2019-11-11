/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 Suyash Yeotikar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  @file tf_test.cpp
 *  @date Nov 11, 2019
 *  @author Suyash Yeotikar
 *  @brief main file
 *  @mainpage project page
 *  Please refer the talker.cpp file in file section
 *  and function members sections for detailed documentation
 */

#include <math.h>
#include <iostream>
#include <sstream>
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

/**
 * @brief test to check if the translation from /world to /talker is correct
 */

TEST(TfTest, tfTestTranslation) {
    tf::TransformListener listener;
    tf::StampedTransform trans;
    std::cout << "Doing translation test" << std::endl;
    listener.waitForTransform("/world", "/talker",
                ros::Time::now(), ros::Duration(2.0));
    listener.lookupTransform("/world", "/talker", ros::Time(0), trans);
    tf::Vector3 origin = trans.getOrigin();
    ASSERT_EQ(origin.x(), 10);
    ASSERT_EQ(origin.y(), 10);
}


/**
 * @brief test to check if the rotation from world to talker is correct.
 */
TEST(TfTest, tfTestRotation) {
    tf::TransformListener listener;
    tf::StampedTransform trans;
    listener.waitForTransform("/world", "/talker",
            ros::Time::now(), ros::Duration(2.0));
    listener.lookupTransform("/world", "/talker", ros::Time(0), trans);
    tf::Quaternion q = trans.getRotation();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ASSERT_DOUBLE_EQ(yaw, M_PI/3);
}


