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
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ohm_rrl_perception_msgs/msg/qr.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <pluginlib/class_loader.hpp>

#include <boost/algorithm/string.hpp>


#include "nav_msgs/srv/get_map.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_srvs/srv/empty.hpp"

#include <Eigen/Geometry>

#include <QtWidgets/QApplication>

#include <francor_geotiff/HectorMapTools.h>

#include <francor_geotiff/geotiff_writer.h>
#include <francor_geotiff/map_writer_plugin_interface.h>

#include <rclcpp/time.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <unordered_map>
#include <fstream>
#include <ctime>
#include <random>

using namespace std;

namespace francor_geotiff{

class ObjectLog
{
public:
  explicit ObjectLog(rclcpp::Logger logger, const std::string& file_name)
    : logger_(logger)
    , file_name_(file_name)
  {
    RCLCPP_WARN_STREAM(logger_, "open new file " << (file_name_ + std::to_string(run_counter_) + ".csv"));
    file_.open(file_name + std::to_string(run_counter_) + ".csv");
    writeHeader();
  }
  ~ObjectLog() {
    file_.close();
  }

  ObjectLog& operator<<(const ohm_rrl_perception_msgs::msg::Qr& qr_object) {
    file_ << qr_object.id << "; " << qr_object.data << "; " << qr_object.pose.position.x << "; " << qr_object.pose.position.y << "; " << qr_object.pose.position.z << "\n";
    return *this;
  }

  void clear() {
    file_.close();
    ++run_counter_;
    RCLCPP_WARN_STREAM(logger_, "open new file " << (file_name_ + std::to_string(run_counter_) + ".csv"));
    file_.open(file_name_ + std::to_string(run_counter_) + ".csv");
    writeHeader();    
  }

private:
  void writeHeader() {
    file_ << "id; text; pos x; pos y; pos z\n";
  }

  rclcpp::Logger logger_;
  std::ofstream file_;
  std::string file_name_;
  std::size_t run_counter_ = 0u;
};

/**
 * @brief Map generation node.
 */
class MapGenerator : public rclcpp::Node
{
public:
  MapGenerator()
    : Node("geotiff_node")
    , geotiff_writer_(rclcpp::get_logger("geotiff_node"), false)
//    , plugin_loader_(0)
    , qr_codes_logger_(rclcpp::get_logger("geotiff_node"), "/home/user/qr_codes_exp")
    , running_saved_map_num_(0)
  {

    this->declare_parameter<string>("map_file_path", ".");
    this->declare_parameter<string>("map_file_base_name", std::string());
    this->declare_parameter<bool>("draw_background_checkerboard", true);
    this->declare_parameter<bool>("draw_free_space_grid", true);
    this->declare_parameter<double>("geotiff_save_period", 0.0);


    p_map_file_path_ = this->get_parameter("map_file_path").as_string();
    geotiff_writer_.setMapFilePath(p_map_file_path_);
    geotiff_writer_.setUseUtcTimeSuffix(true);

    p_map_file_base_name_ = this->get_parameter("map_file_base_name").as_string();

    p_draw_background_checkerboard_ = this->get_parameter("draw_background_checkerboard").as_bool();
    p_draw_free_space_grid_ = this->get_parameter("draw_free_space_grid").as_bool();

    sys_cmd_sub_ = this->create_subscription<std_msgs::msg::String>("syscommand", 10, std::bind(&MapGenerator::sysCmdCallback, this, std::placeholders::_1));
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&MapGenerator::subMapCallback, this, std::placeholders::_1));
    sub_victims_ = this->create_subscription<geometry_msgs::msg::PoseArray>("hector/_victims", 10, std::bind(&MapGenerator::subVictimsCallback, this, std::placeholders::_1));
    sub_qr_detection_ = this->create_subscription<ohm_rrl_perception_msgs::msg::Qr>("qr/localized", 10, std::bind(&MapGenerator::subQrDetectionCallback, this, std::placeholders::_1));

    map_service_client_ = this->create_client<nav_msgs::srv::GetMap>("static_map");
    //object_service_client_ = n_.serviceClient<worldmodel_msgs::srv::GetObjectModel>("worldmodel/get_object_model");

    srv_save_geotiff_ = this->create_service<std_srvs::srv::Empty>("save_geotiff", std::bind(&MapGenerator::serviceSaveGeotiffCall, this, std::placeholders::_1, std::placeholders::_2));

    double p_geotiff_save_period = this->get_parameter("geotiff_save_period").as_double();

    if(p_geotiff_save_period > 0.0){
      //ros::Timer timer = pn_.createTimer(ros::Duration(p_geotiff_save_period), &MapGenerator::timerSaveGeotiffCallback, false);
      //publish_trajectory_timer_ = private_nh.createTimer(ros::Duration(1.0 / p_trajectory_publish_rate_), &PathContainer::publishTrajectoryTimerCallback, this, false);
      map_save_timer_ = this->create_wall_timer(std::chrono::duration<double>(p_geotiff_save_period), std::bind(&MapGenerator::timerSaveGeotiffCallback, this));
    }

    // pn_.param("plugins", p_plugin_list_, std::string(""));

    // std::vector<std::string> plugin_list;
    // boost::algorithm::split(plugin_list, p_plugin_list_, boost::is_any_of("\t "));

    // //We always have at least one element containing "" in the string list
    // if ((plugin_list.size() > 0) && (plugin_list[0].length() > 0)){
    //   plugin_loader_ = new pluginlib::ClassLoader<francor_geotiff::MapWriterPluginInterface>("francor_geotiff", "francor_geotiff::MapWriterPluginInterface");

    //   for (size_t i = 0; i < plugin_list.size(); ++i){
    //     try
    //     {
    //       boost::shared_ptr<francor_geotiff::MapWriterPluginInterface> tmp (plugin_loader_->createInstance(plugin_list[i]));
    //       tmp->initialize(plugin_loader_->getName(plugin_list[i]));
    //       plugin_vector_.push_back(tmp);
    //     }
    //     catch(pluginlib::PluginlibException& ex)
    //     {
    //       ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    //     }
    //   }
    // }else{
    //   RCLCPP_INFO(this->get_logger(), "No plugins loaded for geotiff node");
    // }
    RCLCPP_INFO(this->get_logger(), "Geotiff node started");
  }

  ~MapGenerator()
  {
    // if (plugin_loader_){
    //   delete plugin_loader_;
    // }
  }

  void writeGeotiff()
  {
    
    rclcpp::Time start_time (this->now());

    std::stringstream ssStream;

    // nav_msgs::GetMap srv_map;
    // if (map_service_client_.call(srv_map))
    {
      RCLCPP_INFO(this->get_logger(), "GeotiffNode: Map service called successfully");
      const nav_msgs::msg::OccupancyGrid& map (map_);

      std::string map_file_name = p_map_file_base_name_;

      rclcpp::Parameter competition_name;
      rclcpp::Parameter team_name;
      rclcpp::Parameter mission_name;
      rclcpp::Parameter postfix;

      if (this->get_parameter("/competition", competition_name) && !competition_name.as_string().empty())
        map_file_name = map_file_name + "_" + competition_name.as_string();
      if (this->get_parameter("/team", team_name) && !team_name.as_string().empty())
        map_file_name = map_file_name + "_" + team_name.as_string();
      if (this->get_parameter("/mission", mission_name) && !mission_name.as_string().empty())
        map_file_name = map_file_name + "_" + mission_name.as_string();
      if (this->get_parameter("map_file_postfix", postfix) && !postfix.as_string().empty())
        map_file_name = map_file_name + "_" + postfix.as_string();

      if (map_file_name.substr(0, 1) == "_") map_file_name = map_file_name.substr(1);
      if (map_file_name.empty()) map_file_name = "GeoTiffMap";
      geotiff_writer_.setMapFileName(map_file_name);
      bool transformSuccess = geotiff_writer_.setupTransforms(map);

      if(!transformSuccess){
        RCLCPP_INFO(this->get_logger(), "Couldn't set map transform");
        return;
      }

      geotiff_writer_.setupImageSize();

      if (p_draw_background_checkerboard_){
        geotiff_writer_.drawBackgroundCheckerboard();
      }

      geotiff_writer_.drawMap(map, p_draw_free_space_grid_);
      geotiff_writer_.drawCoords();

      int cnt = 0;
      for(auto& e : victims_.poses)
      {
        std::string id = std::to_string(cnt++);
        geotiff_writer_.drawObjectOfInterest(Eigen::Vector2f(e.position.x,e.position.y), id , francor_geotiff::MapWriterInterface::Color(255, 0,0));
      }

      cnt = 0;
      for(const auto& qr : qr_codes_)
      {
        geotiff_writer_.drawObjectOfInterest(Eigen::Vector2f(qr.second.pose.position.x,qr.second.pose.position.y), std::to_string(qr.second.id), francor_geotiff::MapWriterInterface::Color(0, 0,255));
        qr_codes_logger_ << qr.second;
      }

      // geotiff_writer_.drawObjectOfInterest(Eigen::Vector2f(10.0,10.0), "hans" , francor_geotiff::MapWriterInterface::Color(255, 0,0));

      //RCLCPP_INFO(this->get_logger(), "Sum: %ld", (long int)srv.response.sum);
    }
    // else
    // {
    //   ROS_ERROR("Failed to call map service");
    //   return;
    // }

    // for (size_t i = 0; i < plugin_vector_.size(); ++i){
    //   plugin_vector_[i]->draw(&geotiff_writer_);
    // }

    /**
      * No Victims for now, first  agree on a common standard for representation
      */
    /*
    if (req_object_model_){
      worldmodel_msgs::GetObjectModel srv_objects;
      if (object_service_client_.call(srv_objects))
      {
        RCLCPP_INFO(this->get_logger(), "GeotiffNode: Object service called successfully");

        const worldmodel_msgs::ObjectModel& objects_model (srv_objects.response.model);

        size_t size = objects_model.objects.size();


        unsigned int victim_num  = 1;

        for (size_t i = 0; i < size; ++i){
          const worldmodel_msgs::Object& object (objects_model.objects[i]);

          if (object.state.state == worldmodel_msgs::ObjectState::CONFIRMED){
            geotiff_writer_.drawVictim(Eigen::Vector2f(object.pose.pose.position.x,object.pose.pose.position.y),victim_num);
            victim_num++;
          }
        }
      }
      else
      {
        ROS_ERROR("Failed to call objects service");
      }
    }
    */

    /*
    hector_nav_msgs::GetRobotTrajectory srv_path;

    if (path_service_client_.call(srv_path))
    {
      RCLCPP_INFO(this->get_logger(), "GeotiffNode: Path service called successfully");

      std::vector<geometry_msgs::PoseStamped>& traj_vector (srv_path.response.trajectory.poses);

      size_t size = traj_vector.size();

      std::vector<Eigen::Vector2f> pointVec;
      pointVec.resize(size);

      for (size_t i = 0; i < size; ++i){
        const geometry_msgs::PoseStamped& pose (traj_vector[i]);

        pointVec[i] = Eigen::Vector2f(pose.pose.position.x, pose.pose.position.y);
      }

      if (size > 0){
        //Eigen::Vector3f startVec(pose_vector[0].x,pose_vector[0].y,pose_vector[0].z);
        Eigen::Vector3f startVec(pointVec[0].x(),pointVec[0].y(),0.0f);
        geotiff_writer_.drawPath(startVec, pointVec);
      }
    }
    else
    {
      ROS_ERROR("Failed to call path service");
    }
    */


    geotiff_writer_.writeGeotiffImage();
    running_saved_map_num_++;

    rclcpp::Duration elapsed_time (this->now() - start_time);

    RCLCPP_INFO(this->get_logger(), "GeoTiff created in %f seconds", elapsed_time.seconds());

    qr_codes_logger_.clear();
    qr_codes_.clear();
  }

  void timerSaveGeotiffCallback()
  {
    this->writeGeotiff();
  }

  void sysCmdCallback(const std_msgs::msg::String::SharedPtr sys_cmd)
  {
    if ( !(sys_cmd->data == "savegeotiff")){
      return;
    }

    this->writeGeotiff();
  }

  void serviceSaveGeotiffCall(
    const std_srvs::srv::Empty::Request::SharedPtr  req,
    std_srvs::srv::Empty::Response::SharedPtr res)
  {
    this->writeGeotiff();
  }

  void subMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Got Map");
    map_ = *msg;
    map_.info.origin.orientation.w = 1.0;
    //RCLCPP_INFO_STREAM(this->get_logger(), "map info: " << map_.info.origin);
  } 

  void subVictimsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    victims_ = *msg;
  }

  void subQrDetectionCallback(const ohm_rrl_perception_msgs::msg::Qr::SharedPtr qr)
  {
    if (qr_codes_.find(qr->data) == qr_codes_.end()) {
      auto qr_copy = *qr;
      qr_copy.id = qr_codes_.size() + 1;
      qr_codes_[qr->data] = qr_copy;      
    }
  }

  std::string p_map_file_path_;
  std::string p_map_file_base_name_;
  std::string p_plugin_list_;
  bool p_draw_background_checkerboard_;
  bool p_draw_free_space_grid_;

  //double p_geotiff_save_period_;

  GeotiffWriter geotiff_writer_;

  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_service_client_;
  //rclcpp::Client<worldmodel_msgs::srv::GetObjectModel>::SharedPtr object_service_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_save_geotiff_;


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sys_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_victims_;
  rclcpp::Subscription<ohm_rrl_perception_msgs::msg::Qr>::SharedPtr sub_qr_detection_;
  
  nav_msgs::msg::OccupancyGrid map_;

  geometry_msgs::msg::PoseArray victims_;

  std::unordered_map<std::string, ohm_rrl_perception_msgs::msg::Qr> qr_codes_;
  ObjectLog qr_codes_logger_;

  //std::vector<boost::shared_ptr<francor_geotiff::MapWriterPluginInterface> > plugin_vector_;

  //pluginlib::ClassLoader<francor_geotiff::MapWriterPluginInterface>* plugin_loader_;

  rclcpp::TimerBase::SharedPtr map_save_timer_;

  unsigned int running_saved_map_num_;
};

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<francor_geotiff::MapGenerator>());
  rclcpp::shutdown();
  return 0;
}

