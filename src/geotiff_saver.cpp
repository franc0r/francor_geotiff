//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <cstdio>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/srv/get_map.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include <Eigen/Geometry>

#include <QtWidgets/QApplication>

#include <francor_geotiff/HectorMapTools.h>

#include <francor_geotiff/geotiff_writer.h>
#include <rclcpp/subscription.hpp>

using namespace std;

namespace francor_geotiff{

/**
 * @brief Map generation node.
 */
class MapGenerator : public rclcpp::Node
{
  public:
    MapGenerator(const std::string& mapname) : Node("map_saver"), geotiff_writer(rclcpp::get_logger("map_saver")), mapname_(mapname)
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for the map");
      map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&MapGenerator::mapCallback, this, std::placeholders::_1));
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
      rclcpp::Time start_time (this->now());

      geotiff_writer.setMapFileName(mapname_);
      geotiff_writer.setupTransforms(*map);
      geotiff_writer.drawBackgroundCheckerboard();
      geotiff_writer.drawMap(*map);
      geotiff_writer.drawCoords();

      geotiff_writer.writeGeotiffImage();

      rclcpp::Duration elapsed_time (this->now() - start_time);
      RCLCPP_INFO(this->get_logger(), "GeoTiff created in %f seconds", elapsed_time.seconds());
    }

    GeotiffWriter geotiff_writer;

    std::string mapname_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

}

#define USAGE "Usage: \n" \
              "  geotiff_saver -h\n"\
              "  geotiff_saver [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::string mapname = "map";

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  //GeotiffWriter geotiff_writer;
  //geotiff_writer.setMapName("test");
  rclcpp::spin(std::make_shared<francor_geotiff::MapGenerator>(mapname));
  rclcpp::shutdown();
  return 0;
}

