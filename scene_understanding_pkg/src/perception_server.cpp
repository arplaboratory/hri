#include "ros/ros.h"
//#include </home/arpl/luca_ws/devel/include/scene_understanding_pkg_msgs/MeshPos.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <string>
#include <chrono>
#include <sys/stat.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <algorithm>
#include "perception_utils.h"

#include "perception_server.h"
#include "std_msgs/Float64.h"
#include "SU_Unity_comm.h"
#include "Mesh_constructor.h"
#include <shape_msgs/Mesh.h>
#include <typeinfo>
#include <nav_msgs/Odometry.h>
//Voxblox Libraries
#include <voxblox/core/esdf_map.h>

#include <pcl/common/transforms.h>


namespace perception {
    PerceptionServer::PerceptionServer(const ros::NodeHandle& nh,
                       const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      in_simulation(true),
      horizon_force_perception(1.5),
      lamda(1.0),
      Fs(5.0),
      avg_stamp(10),
      mesh_publish_period(3),
      perception_utils(nh, nh_private)
      {//in sec
      //transformer_(nh, nh_private) { // transformer e una classe che va inizializzata e definita (se necessario)
    //Publisher
    triangles_mesh = nh_.advertise<visualization_msgs::Marker>("/surface_mesh", 1);
    triangles_mesh_array = nh_.advertise<visualization_msgs::MarkerArray>("/surface_mesh_array", 1);

    marker_pub_force_arrow_rviz = nh_.advertise<visualization_msgs::Marker>("f_obs_vector", 10);
    mesh_to_unity = nh_.advertise<scene_understanding_pkg_msgs::MeshVertexPosition>("drone_mesh_to_unity", 1);
    cloud_in_rep = nh_.advertise<sensor_msgs::PointCloud2>("cloud_in_rep_", 1);
    pub_occupied_pc_to_unity = nh_.advertise<sensor_msgs::PointCloud2>("/voxblox_node/occupied_pc_to_unity", 1);
    mesh_pointcloud_to_unity = nh_.advertise<sensor_msgs::PointCloud2>("drone_mesh_to_unity_as_pointcloud", 1); //verificare se utile
    holo_position_rviz = nh_.advertise<visualization_msgs::Marker>("hololens_frame", 1);
    obst_force = nh_.advertise<scene_understanding_pkg_msgs::ObstacleRepForce>("obstacles_force_field", 1);
    hololens_pointcloud = nh_.advertise<sensor_msgs::PointCloud2>("/hololens/pointcloud_out", 1); // "/race11/pointcloud_manager/cloud_in"
    hololens_pc_latency = nh_.advertise<std_msgs::Float64>("/hololens_pc_latency",1);
    drone_mesh_to_unity_time = nh_.advertise<std_msgs::Float64>("/drone_mesh_to_unity_time_topic", 1);
    //Subscribers
    cloud_in = nh_.subscribe("/cloud_in", 5, &PerceptionServer::cloud_in_callback, this);
    hololens_cloud_in = nh_.subscribe("/hololens_pointcloud", 5, &PerceptionServer::pointcloud_bf_hololens_callback_any_camera, this);
    quadrotor_pose = nh_.subscribe("/odom", 10, &PerceptionServer::quadrotor_pose_callback, this); //define the topic from the remapping
    start_stop_mapping = nh_.subscribe("/from_Unity/start_stop_mapping", 5, &PerceptionServer::from_unity_start_stop_mapping_callback, this);
    restart_mapping = nh_.subscribe("/from_Unity/restart_mapping", 5, &PerceptionServer::from_unity_reset_map_callback, this);
    voxblox_mesh = nh_.subscribe("/voxblox_node/mesh", 5, &PerceptionServer::voxblox_mesh_callback, this);
    voxblox_pc_surface = nh_.subscribe("/voxblox_node/surface_pointcloud", 1, &PerceptionServer::voxblox_surface_cloud_callback, this);
    mocap_scene_root_in_unity_world = nh_.subscribe("/from_unity/mocap_frame_in_unity_world_coo", 5, &PerceptionServer::mocap_scene_root_in_unity_frame_callback, this);
    holoPose = nh_.subscribe("/hololens_position", 5, &PerceptionServer::holo_position_callback, this);
    mocapSrPose = nh_.subscribe("/scene_root_mocap_position", 5, &PerceptionServer::mocap_sr_position_callback, this);
    virtual_obstacles = nh_.subscribe("/rrt/virtual_obstacles_coo", 5, &PerceptionServer::virtual_obs_callback, this);
    enable_assistive_mode = nh_.subscribe("/rrt/start_assistive_guidance", 1, &PerceptionServer::planner_start_assistive_guidance_callback, this);

    //add the marker for the visualization of the virtual obstacles in rviz keep it out from drone teleop

     // Services
    clear_map_service = nh_.serviceClient<std_srvs::Empty>("/race10/voxblox_node/clear_map");
    
    //Parameters 
    nh_private_.param("/tele_control_params/scenario", in_simulation, in_simulation);
    nh_private_.param("/tele_control_params/obstacle_force_function", obstacle_force_function, obstacle_force_function);
    nh_private_.param("/tele_control_params/horizon", horizon_force_perception, horizon_force_perception);
    nh_private_.param("/tele_control_params/lamda", lamda, lamda);
    nh_private_.param("/tele_control_params/Fs", Fs, Fs);
    nh_private_.param("/tele_control_params/avg_stamps", avg_stamp, avg_stamp);
    nh_private_.param("/tele_control_params/mesh_publish_period", mesh_publish_period, mesh_publish_period);
   

    //Initialize visualization messages
    initialize_visualization_messages();

    // mesh_publish_period = 3;
    ROS_WARN("Mesh publish peroid is %d", mesh_publish_period);


    if (mesh_publish_period > 0.0) 
    {
    publish_mesh_timer_ =
        nh_private_.createTimer(ros::Duration(0.1),
                                &PerceptionServer::publishMesh, this);

    ROS_INFO("Mesh timer and callback is done");

    publish_occupied_pc_to_unity = 
        nh_private_.createTimer(ros::Duration(mesh_publish_period),
                                &PerceptionServer::publishOccupiedPC_to_unity, this);
    }

  // ros::Time current_time_tf;
  // current_time_tf = ros::Time::now();

  // geometry_msgs::TransformStamped mocap_sr_tf;
  // mocap_sr_tf.header.stamp = current_time_tf;
  // mocap_sr_tf.header.frame_id = "world";
  // mocap_sr_tf.child_frame_id = "mocap_sr";
  // mocap_sr_tf.transform.rotation.w = 1.0;
  // mocap_sr_to_world.sendTransform(mocap_sr_tf);

  perception_utils.avg_stamp = avg_stamp;
  bool call_restart_mapping_service = false;
 
  }


void PerceptionServer::initialize_visualization_messages()
{
  triangles.points.clear();
  obs_magn_force_b_frame.points.clear();

  triangles.header.frame_id = mesh_vis_msgs_frame;//"/mocap_sr";
  obs_magn_force_b_frame.header.frame_id = force_vis_msgs_frame;
  triangles.header.stamp = ros::Time::now();
  obs_magn_force_b_frame.header.stamp = ros::Time::now();

  // Markers Namespace
  // triangles.ns = "triangle_list";
  obs_magn_force_b_frame.ns = "force_arrow";
  // Assign a type of action to each marker
  triangles.action = visualization_msgs::Marker::ADD;
  obs_magn_force_b_frame.action = visualization_msgs::Marker::ADD;

  // Pose -- Since are points the orientation is not important
  triangles.pose.orientation.w = 1.0;
  obs_magn_force_b_frame.pose.orientation.w = 1.0;

  // Marker Type (Arrow, Sphere, points )
  triangles.type = visualization_msgs::Marker::TRIANGLE_LIST;
  obs_magn_force_b_frame.type = visualization_msgs::Marker::ARROW;

  // triangles.id = 0;
  obs_magn_force_b_frame.id = 0;

  // POINTS markers use x and y scale for width/height respectively
  triangles.scale.x = 1;
  triangles.scale.y = 1;
  triangles.scale.z = 1;

  obs_magn_force_b_frame.scale.x = 1;
  obs_magn_force_b_frame.scale.y = 0.05;
  obs_magn_force_b_frame.scale.z = 0.05;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width

  // Points are green
  triangles.color.g = 1.0f;
  triangles.color.a = 0.3;
  
  obs_magn_force_b_frame.color.g = 1.0f;
  obs_magn_force_b_frame.color.a = 1.0;

}

void PerceptionServer::publishMesh(const ros::TimerEvent& /*event*/) {
    
    // ROS_ERROR("Inside publish mesh function");
    
    if (triangles.points.size() > 0)
    {
      triangles.id = traiangle_id_count;

      std::string namespace_string = "triangle_list" + std::to_string(traiangle_id_count);
      triangles.ns = namespace_string;

      traiangle_id_count ++;    

      // this->triangles_mesh.publish(triangles);
      // publish_mesh_to_unity(triangles);

      // Adding into marker array
      if(traiangle_id_count%5 == 1)
      {      
        this->triangles_mesh.publish(triangles);
        publish_mesh_to_unity(triangles);

        triangles_array.markers.push_back(triangles);
        this->triangles_mesh_array.publish(triangles_array);
      }
      
      cout << "-----------------------------Data Published-------------------------------------------" << endl;
    }
    
    triangles.points.clear();      
  //Clear the voxblox Vector otherwise the same triamngles are added to the Visualization 
  //Marker Message 
    voxblox_mesh_vertices.clear();

}


void PerceptionServer::publishOccupiedPC_to_unity(const ros::TimerEvent& /*event*/) {

  bool point_existing = false;  
  pc_to_unity_new.points.clear();
  cout << "pc_surface_ size: " << pc_surface_.points.size() << endl;
  cout << "pc_to_unity size: " << pc_to_unity.points.size() << endl;
//Every time a pointcloud is received its required to filtering it publishing only the added points before republishing to unity 
  if (pc_surface_.points.size() > pc_to_unity.points.size())
  {
   //iterate over the pointcloud and ublishing to unity only the different points 
    pcl::PointCloud<pcl::PointXYZ>::const_iterator item;
    pcl::PointCloud<pcl::PointXYZ>::const_iterator item2;

   //Iterate on the new pointcloud 
    for (item = pc_surface_.begin(); item != pc_surface_.end(); item++) 
    {
      point_existing = false;
      //iterate on the old pointcloud 
      for (item2 = pc_to_unity.begin(); item2 != pc_to_unity.end(); item2++) 
      {
      float dx = item->x - item2->x; 
      float dy = item->y - item2->y; 
      float dz = item->z - item2->z; 
      float distance_3D = sqrt(pow((double)dx, 2) + pow((double)dy, 2) + pow((double)dz, 2) );
       
       if (distance_3D == 0){
        point_existing = true;
        break;
        }
       }
     
      //check if the program arrives here with point_existing = false so i can add the point to the pointcloud to publish 
      if (point_existing == false)
      {
          //add the point to the new pointcloud

          pc_to_unity_new.points.push_back(pcl::PointXYZ(item->x, item->y, item->z));
      }
  
     }
   pc_to_unity.points.clear();
   pc_to_unity = pc_surface_;
  }
 
  //create the pointcloud to publish
  pcl::PCLPointCloud2 pointCloud;
  sensor_msgs::PointCloud2 pointCloud_out;

  std_msgs::Header header_msg;
  header_msg.frame_id = frame_pc;
  header_msg.stamp = ros::Time::now();

  /* prepare pointCloud msg to publish */
  pointCloud_out.header = header_msg;

  pcl::toPCLPointCloud2(pc_to_unity_new, pointCloud);
  pcl_conversions::moveFromPCL(pointCloud, pointCloud_out);
  pointCloud_out.header = header_msg;
   
  cout << "###########################################################" << endl;
  cout << "pc_to_unity_new size: " << pc_to_unity_new.points.size() << endl;
  pub_occupied_pc_to_unity.publish(pointCloud_out);


}




void PerceptionServer::publish_hololens_pose_in_RVIZ()
{
    visualization_msgs::Marker sphere;
    sphere.points.clear();

    sphere.header.frame_id = "mocap";
    sphere.header.stamp  = ros::Time::now();
    //Markers Namespace 
     sphere.ns = "holo_position_in_unity";
    // Assign a type of action to each marker
     sphere.action =  visualization_msgs::Marker::ADD;
    //Pose -- Since are points the orientation is not important

     sphere.pose.position =  hololens_position;
     sphere.pose.orientation = hololens_orientation;
     sphere.pose.orientation.w = 1.0;

    // Marker Type (Arrow, Sphere, points )
     sphere.type = visualization_msgs::Marker::SPHERE;   
     sphere.id = 0;
  
    // POINTS markers use x and y scale for width/height respectively
     sphere.scale.x = 0.3;
     sphere.scale.y = 0.3;
     sphere.scale.z = 0.3;
    
     sphere.color.b = 1.0f;
     sphere.color.a = 1.0;

    holo_position_rviz.publish(sphere);
   

}

void PerceptionServer::evaluate_obstacle_force_field(double dt_)
{
  //The pointcloud received from Voxblox is expressed on the frame Map which is aliigne with the frame world.
  //Take the robot position and the surface pointcloud position 
  float horizon = horizon_force_perception; 
  float theta = 0.0;

  float dx_sum = 0.0;
  float dy_sum = 0.0;
  float sat_x_value = Fs;
  float sat_y_value = Fs;
  vector<float> theta_vec;


  //Looking for voxels insde the robot horizon 
  pcl::PointCloud<pcl::PointXYZ>::const_iterator item;
  /*
  if (pc_surface_.size() > 0)
  {
    distance_vec.clear();
    F_magn = 0.0;
  }
  */

 distance_vec.clear();
 F_magn = 0.0;
  for (item = pc_surface_.begin(); item != pc_surface_.end(); item++) 
  {
   
    float dx = item->x - (mocap_sr_unity_world_position.x + quad_position.x); 
    float dy = item->y - (mocap_sr_unity_world_position.y + quad_position.y); 
    float dz = item->z - quad_position.z; 
    float distance_2D = sqrt(pow((double)dx, 2) + pow((double)dy, 2));
     
    //cout << "distance_2D: " <<distance_2D << endl;
    
    if (distance_2D < horizon && abs(dz) < 0.1) 
    {  
     
      //Define the Force Vector representend in a frame S located on the point and aligned with the World frame (the origin of the vector is in the world frame)
      //The Force vector is defined by the Fx and Fy versor on S, Pointing to the robot with intensity inversely proportional to the distance. 
      //To Evaluate the for ce vector direction i need to find the angle theta on the frame S whic the dscribe the direction of the vector F pointing to the drone 
      //from the origin of the gframe S. 
      Eigen::Vector2f Force_not_scaled; 
      Force_not_scaled << dx, dy; 
       //Evaluate the angle between the world frame and the surface point considered 
      theta =  acos(Force_not_scaled(1)/ distance_2D);
      //Theta needs to be represented as the yaw angle is shown in  the world frame. 
      theta = perception_utils.align_theta_with_yaw(theta, dx, dy);
     
     // //Add all the d
      if (obstacle_force_function == "linear")
      {
         float m = -1*Fs/horizon; //line slope
         F_magn = m*distance_2D + Fs;

      }
      else
      {
        //Exponential
        float k = 1 - exp(horizon);
        F_magn = (Fs/k)*exp(-lamda*distance_2D)*(1-exp(horizon - distance_2D)); //k_coeff/pow((double)distance_2D, 3);
      }
      
      
      //EValuate the component of teh force with the new magnitude on the frame S located on the voxel
      float F_magn_squared = pow((double)F_magn, 2);
      float dy_ = F_magn * sin(theta); 
      float dy_squared = pow((double)dy_, 2);
      float dx_ = sqrt(F_magn_squared - dy_squared);
      
      

      //Change properly the sign of the x component depending 
      if (dx > 0)
      {
        dx_ = -1*dx_;
      }

      dx_sum += dx_;
      dy_sum += dy_;

     
      
      distance_vec.push_back(distance_2D);
      theta_vec.push_back(theta);

    }
  }

  //add to the force also the force generated by the virtual obstacles if presents 
    int counter_obs = 0;
    for (int ii = 0; ii <  virtual_obstacles_vec.size()/2; ii++)
    {
        float min_x = virtual_obstacles_vec[counter_obs].x;
        float min_y = virtual_obstacles_vec[counter_obs].y;

        float max_x = virtual_obstacles_vec[counter_obs + 1].x;
        float max_y = virtual_obstacles_vec[counter_obs + 1].y;

        float x_center = (max_x - min_x)/2;
        x_center = max_x -x_center;

        float y_center = (max_y - min_y)/2;
        y_center = max_y -y_center;


       float dx = x_center - quad_position.x; 
       float dy = y_center - quad_position.x; 

      
       float distance_2D = sqrt(pow((double)dx, 2) + pow((double)dy, 2));
      
        //Samples the points lower that the drone horizon 
    // if (distance_2D < horizon ) 
    // {  
     
     
    //   Eigen::Vector2f Force_not_scaled; 
    //   Force_not_scaled << dx, dy; 
    //    //Evaluate the angle between the world frame and the surface point considered 
    //   theta =  acos(Force_not_scaled(1)/ distance_2D);
    //   //Theta needs to be represented as the yaw angle is shown in  the world frame. 
    //   theta = align_theta_with_yaw(theta, dx, dy);
      
    //   // //Add all the d

    //   float k = 1 - exp(horizon);
    //   F_magn = 0.3*(Fs/k)*exp(-lamda*distance_2D)*(1-exp(horizon - distance_2D)); //k_coeff/pow((double)distance_2D, 3);
      
    //   //EValuate the component of teh force with the new magnitude on the frame S located on the voxel
    //   float F_magn_squared = pow((double)F_magn, 2);
    //   float dy_ = F_magn * sin(theta); 
    //   float dy_squared = pow((double)dy_, 2);
    //   float dx_ = sqrt(F_magn_squared - dy_squared);
      
    //   //Change properly the sign of the x component depending 
    //   if (dx > 0)
    //   {
    //     dx_ = -1*dx_;
    //   }
      
    //   dx_sum += dx_;
    //   dy_sum += dy_;

    //   distance_vec.push_back(distance_2D);
    //   theta_vec.push_back(theta);
    
    // }
    counter_obs = counter_obs + 2; 
   
    }
 virtual_obstacles_vec.clear();
    //  //Normalize the resultant vector
  float ip = sqrt(pow((double)dx_sum, 2) + pow((double)dy_sum, 2));
  float dx_sum_norm = dx_sum/ip;
  float dy_sum_norm = dy_sum/ip;
  float norm_ip = sqrt(pow(dx_sum_norm,2) + pow(dy_sum_norm,2));

  //Evaluate theta of the current sum vector 
  theta_res = acos(dx_sum_norm/norm_ip);
 
  if (dy_sum_norm <= 0)
  {
    theta_res = -theta_res;
  }

   //Search for the minimum value in the distance vector
  float min_dist = 0.0;
  int index_min_distance = 0.0;
  float theta_min_distance = 0.0;
  float dx_new = 0.0;
  float dy_new = 0.0;
  float dx_fil = 0.0;
  float dy_fil = 0.0;
  float dt = 0.02;
  float theta_dot = 0.0;
  float safe_distance = 0.0;
  Eigen::Vector2f p_distance = Eigen::Vector2f(0.0f, 0.0f);
  Eigen::Vector2f p_distance_dot = Eigen::Vector2f(0.0f, 0.0f);

  if (distance_vec.size() > 0 || F_magn > 0.01)
   {

    //average the value of theta_res to avoid big oscillation 
    //theta_res = perception_utils.average_theta_val(theta_res, &theta_vec);
   
      // //Compute average of the distances 
    float sum = accumulate(distance_vec.begin(), distance_vec.end(), 0);
    float d_vec_avg = sum/distance_vec.size();

   
    min_dist = *min_element(distance_vec.begin(), distance_vec.end());
    // //Find the index related to the minimum distance element 
    index_min_distance = perception_utils.getIndex(&distance_vec, min_dist);

     //Copute and check the derivative of theta to avoid big oscillation where in wall cavity or near corners 
    theta_dot = perception_utils.compute_theta_dot(theta_res, theta_min_distance_old, dt);

    
    //The new Magnitude of the vector is given by the exponetial decay relationship evaluated to the element in the surface with the ckoser distance 
   
    if (obstacle_force_function == "linear")
      {
         float m = -1*Fs/horizon; //line slope
         F_magn = m*min_dist + Fs;

      }
      else
      {
        //Exponential
       float k = 1 - exp(horizon);
       F_magn = (Fs/k)*exp(-lamda*min_dist)*(1-exp(horizon - min_dist)); 
      }
    
   
    dx_new = F_magn * cos(theta_res);
    dy_new = F_magn * sin(theta_res);
    theta_min_distance_ = theta_res;

    float p_distance_ = horizon - min_dist;
    float min_dist_x = p_distance_* cos(theta_res);
    float min_dist_y = p_distance_* sin(theta_res);
    
    p_distance = Eigen::Vector2f(min_dist_x, min_dist_y);
    p_distance_dot = (p_distance - p_distance_old)/(float)dt_;
    if(APVI)
    {
      dx_new = 0.2*dx_new;
      dy_new = 0.2*dy_new;
      F_magn = sqrt(pow(dx_new,2) + pow(dy_new, 2));
    }
    //FIlter the components of the force to avoid big change in the direction of the force 
    perception_utils.average_marker_postions_on_multiple_stamps(&dx_fil, &dy_fil, dx_new, dy_new);

    // cout << "theta_min_distance: " << theta_res << endl;
    // cout << "dx_new: " << dx_fil << endl;
    // cout << "dy_new: " << dy_fil << endl;

    theta_min_distance_old = theta_res;
    p_distance_old = p_distance;

  }
  else
  {
    F_magn = 0.0;
    p_distance = Eigen::Vector2f(0.0, 0.0);
    p_distance_old = Eigen::Vector2f(0.0, 0.0);
    p_distance_dot = Eigen::Vector2f(0.0, 0.0);
    distance_vec.clear();

  }

  float dx_sum_sat = 0.0;
  float dy_sum_sat = 0.0;
  perception_utils.saturate_force_value(&dx_sum_sat, &dy_sum_sat, dx_fil,dy_fil, sat_x_value, sat_y_value );
  
  //Publosh the arrow representing the magnitude of the onstacle ofrce field centered on the robot frame 
  publish_obs_force_array_marker(theta_res);
 
  //This is fine, try with the sqaured of the cubic of the distance. 
  //If still to high, normalie and put a gain or saturate the Force up to a certain value on dx and dy 
  // cout << "F_magn: " << F_magn << endl;
  // cout << "dx_sum: " << dx_sum_sat << " dy_sum: " << dy_sum_sat <<  endl;
  // cout << "penetration distance " << p_distance << endl;
  // cout << "penetration distance dot" << p_distance_dot << endl;
  publish_obstacle_force_resultant(dx_sum_sat, dy_sum_sat, F_magn, p_distance, p_distance_dot);
  
//   kF_mag = kimera_data->F_magn;
  
}

/*
*  Services
*/



/* 
* Callbacks 
*/
void PerceptionServer::voxblox_surface_cloud_callback(const sensor_msgs::PointCloud2 &input){
     pc_surface_.points.clear();
     pcl::PCLPointCloud2 pc_surface;//(new pcl::PCLPointCloud2);
     pcl_conversions::toPCL(input, pc_surface);      
     pcl::fromPCLPointCloud2(pc_surface, pc_surface_);
     frame_pc = input.header.frame_id;
    
}

void PerceptionServer::cloud_in_callback(const sensor_msgs::PointCloud2 &input) //const boost::shared_ptr<const sensor_msgs::PointCloud2>
{   

    start_mapping = true;
    if (start_mapping == true)
    {
       //Filter out the floor 
      pcl::PCLPointCloud2::Ptr pc_in(new pcl::PCLPointCloud2());
      pcl_conversions::toPCL(input, *pc_in); 
      pcl::PointCloud<pcl::PointXYZ>::Ptr  pc_in_(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr   pc_in_floor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*pc_in, *pc_in_);

      //Rotation Variables in W frame to filter out not required points 
      float tx = 0.0;
      float ty = 0.0; 
      float tz = 0.0;
      float theta = 0.0;
      float alfa = 0.0;
      float yaw = 0.0;

      tx =  quad_position.x; // x_camera
      ty =  quad_position.y; // y_camera
      tz =  quad_position.z; // z_camera

      theta = quad_euler_orientation.x ; // theta_camera; rotation of the camera frame around the x axis of the world frame
      alfa = quad_euler_orientation.y; // alfa_camera;
      yaw = quad_euler_orientation.z;  // yaw_camera;

      Eigen:: Matrix4f T;
  T << 1, 0, 0, tx,
      0, 1, 0, ty,
      0, 0, 1, tz,
      0, 0, 0, 1;

 // theta = -theta;
 
 Eigen:: Matrix4f Rx;
  Rx << 1, 0, 0, 0,
      0, cos(theta), -sin(theta), 0,
      0, sin(theta), cos(theta), 0,
      0, 0, 0, 1;

  //alfa = - alfa;
 Eigen:: Matrix4f Ry;
  Ry << cos(alfa), 0, sin(alfa), 0,
      0, 1, 0, 0,
      -sin(alfa), 0, cos(alfa), 0,
      0, 0, 0, 1;
  
  
  Eigen::Matrix4f Rz;
  Rz << cos(yaw), -sin(yaw), 0, 0,
      sin(yaw), cos(yaw), 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Eigen::Matrix3f Rz_final;
  Rz_final << cos(yaw), -sin(yaw), 0,
      sin(yaw), cos(yaw), 0,
      0, 0, 1;
      
      Eigen::Vector4f P_C;
  P_C << 0.0, 0.0, 0.0, 0.0;

    Eigen::Vector4f P_W;
  P_W << 0.0, 0.0, 0.0, 0.0;

      //Iterate over the points to create a new pointcloud without the floor 
      pcl::PointCloud<pcl::PointXYZ>::const_iterator item;
      distance_vec.clear();
      int counter = 0;
    for (item = pc_in_->begin(); item != pc_in_->end(); item++) 
    {
      
       P_C << item->z, -1*item->x, -1*item->y, 1; 
       //Define the Final ROtation for the tf transform 
       P_W = T*Rz* Ry* Rx * P_C;

       if (P_W(2) < 0.30)
       {
          counter = counter+ 1;
          continue;
       }
          
        pc_in_floor_filtered->push_back(pc_in_->points[counter]);
        counter = counter + 1;
    }

      
      sensor_msgs::PointCloud2 pointCloud_out;
      pcl::PCLPointCloud2::Ptr pc_out(new pcl::PCLPointCloud2());;
      pcl::toPCLPointCloud2(*pc_in_floor_filtered, *pc_out);
      pcl::PCLPointCloud2 pcl_pc2_n;
      pcl_pc2_n = *pc_out;
      pcl_conversions::moveFromPCL(pcl_pc2_n, pointCloud_out);
      //pointCloud_out = input;
      pointCloud_out.header.stamp = ros::Time::now();
      cloud_in_rep.publish(pointCloud_out);
      cout << "START MAPPING" << endl;
    }
      else
    {
      cout << "STOP MAPPING" << endl;
    }
}



void PerceptionServer::from_unity_start_stop_mapping_callback(const std_msgs::Bool &msg)
{
    /*Provide a way to pause the map */
    start_mapping = msg.data;
    
}


void PerceptionServer::mocap_scene_root_in_unity_frame_callback(const geometry_msgs::PoseStamped &msg)
{
     /*
     Callback to receive the position of the MSR Unity frame respect the world frame.
     */
     //!!!!! REMEBER
     mocap_sr_unity_world_position.x = msg.pose.position.x;
     mocap_sr_unity_world_position.y = msg.pose.position.y;
     mocap_sr_unity_world_position.z = msg.pose.position.z;
      
     mocap_sr_orientation_unity_world = msg.pose.orientation;
     // Covert to euler angles
     tf::Quaternion quat;
     tf::quaternionMsgToTF(msg.pose.orientation, quat);
     // the tf::Quaternion has a method to acess roll pitch and yaw
     double roll, pitch, yaw;
     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
     // the found angles are written in a geometry_msgs::Vector3
     mocap_sr_euler_angles_unity_world.x = roll;
     mocap_sr_euler_angles_unity_world.y = pitch;
     mocap_sr_euler_angles_unity_world.z = yaw;


    //  publishing mocap_sr to world to unity_frame transformation
    // mocap_sr to world

    ros::Time current_time_tf;
    current_time_tf = ros::Time::now();

    geometry_msgs::TransformStamped mocap_sr_tf;
    mocap_sr_tf.header.stamp = current_time_tf;
    mocap_sr_tf.header.frame_id = "world";
    mocap_sr_tf.child_frame_id = "mocap_sr";
    mocap_sr_tf.transform.rotation.w = 1.0;
    mocap_sr_to_world.sendTransform(mocap_sr_tf);

    // unity_frame to mocap_sr
    tf::Transform unity_frame_tf;
    unity_frame_tf.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
    unity_frame_tf.setRotation(quat);
    tf::Transform unity_frame_inv_tf = unity_frame_tf.inverse();
    unity_frame_to_mocap_sr.sendTransform(tf::StampedTransform(unity_frame_inv_tf, ros::Time::now(), "mocap_sr", "unity_frame"));


}


void PerceptionServer::holo_position_callback(const geometry_msgs::Pose &msg)
{

  // Publishing transformation of hololens wrt to unity_fr
  tf::Transform hololens_frame_tf;
  tf::Quaternion quat_hololens;
  tf::quaternionMsgToTF(msg.orientation, quat_hololens);

  // convert into rotation
  // tf::Matrix3x3 rotation_holo(quat_mocap);

  // double roll_mocap, pitch_mocap, yaw_mocap;
  // rotation_holo.getRPY(roll_mocap, pitch_mocap, yaw_mocap);

 
  hololens_frame_tf.setOrigin(tf::Vector3(msg.position.x, msg.position.y, msg.position.z));
  hololens_frame_tf.setRotation(quat_hololens);
  unity_frame_to_hololens.sendTransform(tf::StampedTransform(hololens_frame_tf, ros::Time::now(), "unity_frame", "hololens"));
}


void PerceptionServer::mocap_sr_position_callback(const geometry_msgs::Pose &msg)
{
  // Publishing static transform between world and mocap_sr i.e. identity
  geometry_msgs::TransformStamped mocap_sr_tf;
  mocap_sr_tf.header.stamp = ros::Time::now();
  mocap_sr_tf.header.frame_id = "world";
  mocap_sr_tf.child_frame_id = "mocap_sr";
  mocap_sr_tf.transform.rotation.w = 1.0;
  mocap_sr_to_world.sendTransform(mocap_sr_tf);
  
  // Publishing transformation of unity_frame wrt mocap_sr using inverse
  tf::Transform unity_frame_tf;
  tf::Quaternion quat_mocap;
  tf::quaternionMsgToTF(msg.orientation, quat_mocap);

  // // convert into rotation
  // tf::Matrix3x3 rotation_mocap(quat_mocap);

  // double roll_mocap, pitch_mocap, yaw_mocap;
  // rotation_mocap.getRPY(roll_mocap, pitch_mocap, yaw_mocap);

  // tf::Matrix3x3 rotation_mocap_new;
  // // rotation_mocap_new.setRPY(yaw_mocap, -pitch_mocap, -roll_mocap);      
  // rotation_mocap_new.setRPY(pitch_mocap, roll_mocap, yaw_mocap);

  // tf::Quaternion quat_mocap_new;
  // rotation_mocap_new.getRotation(quat_mocap_new);

  // // unity_frame_tf.setOrigin(tf::Vector3(msg.position.x, -msg.position.y, msg.position.z));
  // unity_frame_tf.setOrigin(tf::Vector3(msg.position.z, -msg.position.y, msg.position.x));


  // unity_frame_tf.setRotation(quat_mocap_new);
  // tf::Transform unity_frame_inv_tf = unity_frame_tf.inverse();


  unity_frame_tf.setOrigin(tf::Vector3(msg.position.x, msg.position.y, msg.position.z));
  unity_frame_tf.setRotation(quat_mocap);
  tf::Transform unity_frame_inv_tf = unity_frame_tf.inverse();

  tf::Quaternion current_rotation = unity_frame_inv_tf.getRotation();
  tf::Quaternion rotX;
  rotX.setRPY(-M_PI/2.0, 0.0, 0.0);
  tf::Quaternion updated_rotation = rotX * current_rotation;
  unity_frame_inv_tf.setRotation(updated_rotation);

  tf::Vector3 current_position = unity_frame_inv_tf.getOrigin();
  double z_pose = current_position.getZ();
  current_position.setZ(-z_pose);
  unity_frame_inv_tf.setOrigin(current_position);

  unity_frame_to_mocap_sr.sendTransform(tf::StampedTransform(unity_frame_inv_tf, ros::Time::now(), "mocap_sr", "unity_frame"));
}


void PerceptionServer::virtual_obs_callback(const scene_understanding_pkg_msgs::RRTObstaclesCoo &msg)
{
  /*
  Obtain the pose of the virtual obstacle if required 
  */
  for (int i = 0; i < msg.point.size(); i++)
 {
   //Take obstacles data
   virtual_obstacles_vec.push_back(msg.point[i]);
 }

}

void PerceptionServer::planner_start_assistive_guidance_callback(const std_msgs::Bool flag)
{
    APVI = flag.data;
}

void PerceptionServer::voxblox_mesh_callback(const voxblox_msgs::Mesh::ConstPtr &msg){
  /* Function Description
  *
  */
    geometry_msgs::Point p;
    geometry_msgs::Point p_new;
    for (const voxblox_msgs::MeshBlock &mesh_block : msg->mesh_blocks)
    {
        const voxblox::BlockIndex index(mesh_block.index[0], mesh_block.index[1], mesh_block.index[2]);

        size_t vertex_index = 0u;
        voxblox::Mesh mesh;
        mesh.vertices.reserve(mesh_block.x.size());
        mesh.indices.reserve(mesh_block.x.size());

        // translate vertex data from message to voxblox mesh
        for (size_t i = 0; i < mesh_block.x.size(); ++i)
        {
            // Each vertex is given as its distance from the blocks origin in units of
            // (2*block_size), see mesh_vis.h for the slightly convoluted
            // justification of the 2.
            constexpr float point_conv_factor =
                2.0f / std::numeric_limits<uint16_t>::max();
            const float mesh_x =
                (static_cast<float>(mesh_block.x[i]) * point_conv_factor +
                 static_cast<float>(index[0])) *
                msg->block_edge_length;
            const float mesh_y =
            (static_cast<float>(mesh_block.y[i]) * point_conv_factor +
                static_cast<float>(index[1])) *
                msg->block_edge_length;
            const float mesh_z =
                (static_cast<float>(mesh_block.z[i]) * point_conv_factor +
                 static_cast<float>(index[2])) *
                msg->block_edge_length;

            p_new.x = mesh_x;
            p_new.y = mesh_y;
            p_new.z = mesh_z; 

            voxblox_mesh_vertices.push_back(p_new);
            
            mesh.indices.push_back(vertex_index++);
            mesh.vertices.emplace_back(p_new.x, p_new.y, p_new.z); //mesh_x, mesh_y, mesh_z
        }
    }
  //This part should be inside the visualizer function populate_RVIZ_triangle_list_with_voxblox_pc
    int counter = 0;
    int counter_clock_wise =0;
    for (int tri_index = 0; tri_index < voxblox_mesh_vertices.size() / 3; ++tri_index)  
    {
      geometry_msgs::Point p;
      p = voxblox_mesh_vertices[counter]; 
      triangles.points.push_back(p);
      p = voxblox_mesh_vertices[counter + 1]; 
      triangles.points.push_back(p);
      p = voxblox_mesh_vertices[counter + 2]; //counter_clock_wise
      triangles.points.push_back(p);


      // ROS_ERROR("Ponint is publisheed ");
      
      counter = counter + 3;
      counter_clock_wise = counter;
    }

}

void  PerceptionServer::quadrotor_pose_callback(const nav_msgs::Odometry &msg)
{
   
    quad_position.x = msg.pose.pose.position.x;
    quad_position.y = msg.pose.pose.position.y;
    quad_position.z = msg.pose.pose.position.z;
    //orientation
     
    // Covert to euler angles
    quad_quat_orientation = msg.pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    quad_euler_orientation.x = roll;
    quad_euler_orientation.y = pitch;
    quad_euler_orientation.z = yaw;
    double time_stamp = double(msg.header.stamp.sec) +  double(msg.header.stamp.nsec)*1e-9;
    double dt = time_stamp - t_stamp_old;
    //Computing the Obstacle Force repulsion every time a new pose is received 
    evaluate_obstacle_force_field(dt);

    //Set up the tf frames depending if running in simulation or real environment
    if (in_simulation)
    {
      publish_mocap_sim_tf();
    }
    else
    {
      publish_hololens_pose_in_RVIZ();
    }
    t_stamp_old = time_stamp;
}

void PerceptionServer::from_unity_reset_map_callback(const std_msgs::Bool &msg)
{
    bool reset_map_flag = msg.data;

    if (reset_map_flag)
    {
    std_srvs::Empty srv;
    clear_map_service.waitForExistence();
    
    if (clear_map_service.call(srv))
    {
    ROS_ERROR("Successfully called service clear_map");
    }
    else
    {
    ROS_ERROR("Failed to call service clear_map");
    }
     pc_to_unity.points.clear();
    }
   
}

void PerceptionServer::pointcloud_bf_hololens_callback_any_camera(const scene_understanding_pkg_msgs::MeshPosArray &msg){
  // This function is used to convert the mesh coordinate from hololens to pointcloud2 message and then to pointcloud in the GF frame
  // The mesh coordinate is in the form of rosmsg show scene_understanding_pkg_msgs/MeshPosArray 
  // float32[] data_x
  // float32[] data_y
  // float32[] data_z
   //ros::Time message_generation_time = msg.header.stamp;
   //ros::Time current_time_tf;
   //current_time_tf = ros::Time::now();
   //ros::Duration pc_holo_dur = current_time_tf - message_generation_time;
   //std_msgs::Float64 dur;
   //dur.data = pc_holo_dur.toSec();
   //hololens_pc_latency.publish(dur);

   // Publishing static transform between world and mocap_sr i.e. identity
   geometry_msgs::TransformStamped mocap_sr_tf;
   mocap_sr_tf.header.stamp = ros::Time::now();
   mocap_sr_tf.header.frame_id = "world";
   mocap_sr_tf.child_frame_id = "mocap_sr";
   mocap_sr_tf.transform.rotation.w = 1.0;
   mocap_sr_to_world.sendTransform(mocap_sr_tf);

  
   // Publishing transformation of unity_frame wrt mocap_sr using inverse
   tf::Transform unity_frame_tf;
   tf::Quaternion quat_mocap;
   tf::quaternionMsgToTF(msg.SR_Mocap.orientation, quat_mocap);
   unity_frame_tf.setOrigin(tf::Vector3(msg.SR_Mocap.position.x, msg.SR_Mocap.position.y, msg.SR_Mocap.position.z));
   unity_frame_tf.setRotation(quat_mocap);
   tf::Transform unity_frame_inv_tf = unity_frame_tf.inverse();
   unity_frame_to_mocap_sr.sendTransform(tf::StampedTransform(unity_frame_inv_tf, ros::Time::now(), "mocap_sr", "unity_frame"));


   // Publishing transformation of hololens wrt to unity_fr
   tf::Transform hololens_frame_tf;
   tf::Quaternion quat_hololens;
   tf::quaternionMsgToTF(msg.HoloLens.orientation, quat_hololens);
  hololens_frame_tf.setOrigin(tf::Vector3(msg.HoloLens.position.x, msg.HoloLens.position.y, msg.HoloLens.position.z));
  hololens_frame_tf.setRotation(quat_hololens);
  unity_frame_to_hololens.sendTransform(tf::StampedTransform(hololens_frame_tf, ros::Time::now(), "unity_frame", "hololens"));
  
  // ---------------------------------- TF Done -------------------------------------------------

 //  pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered_pc(new pcl::PointCloud<pcl::PointXYZ>);
 //  unfiltered_pc->width = msg.data_x.size();
 //  unfiltered_pc->height = 1;
 //  int filtered_pc_size = 0;
 //  int input_data_size = msg.data_x.size();
 //  unfiltered_pc->points.resize(input_data_size);
 //  for (int i = 0; i<input_data_size; i++){

 //        if(msg.data_y[i]<=1.2)
 //        {
 //        unfiltered_pc->points[i].x = msg.data_x[i];
 //        unfiltered_pc->points[i].z = msg.data_y[i];
 //        unfiltered_pc->points[i].y = msg.data_z[i];
 //        filtered_pc_size++;
 //        }
 //  }

 //  pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
 //  pt_cloud->width = filtered_pc_size;
 //  pt_cloud->height = 1;
 //  int filtered_input_data_size = filtered_pc_size;
 //  pt_cloud->points.resize(filtered_pc_size);

  // Fill the pointcloud with the data from the mesh coordinate
 //  for (int i = 0; i<filtered_input_data_size; i++){

	// pt_cloud->points[i].x = unfiltered_pc->points[i].x;
 //   	pt_cloud->points[i].z = unfiltered_pc->points[i].z;
 //    	pt_cloud->points[i].y = unfiltered_pc->points[i].y;
 //  }


// ------------------------------ celing Removal---------------------------------
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pt_cloud->width = msg.data_x.size();
  pt_cloud->height = 1;
  int input_data_size = msg.data_x.size();;
  pt_cloud->points.resize(input_data_size);
  
  
  for (int i = 0; i<input_data_size; i++){

	pt_cloud->points[i].x = msg.data_x[i];
   	pt_cloud->points[i].z = msg.data_y[i];
    	pt_cloud->points[i].y = msg.data_z[i];
  }
  

  Eigen::Transform<float, 3, Eigen::Affine> transform = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
  transform.translation() = Eigen::Vector3f(0.0, 0.0, 0.2); //-msg.SR_Mocap.position.y-0.7

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::transformPointCloud(*pt_cloud, *transformed_cloud, transform);

  // unity_frame to camera_left frame
  tf::TransformListener unity_frame_listener;
  tf::StampedTransform unity_frame_transform;

  ros::Duration timeout(5.0);
  ros::Time start_time = ros::Time::now();

  while(ros::ok() && (ros::Time::now() - start_time) < timeout)
  {

    try
    {
      // getting transformation between world to left_stereo frames
      unity_frame_listener.waitForTransform("world", "race10/left_stereo", ros::Time(0), ros::Duration(3.0));
      unity_frame_listener.lookupTransform("world", "race10/left_stereo", ros::Time(0), unity_frame_transform);

      tf::Vector3 tf_translation = unity_frame_transform.getOrigin();
      tf::Quaternion tf_rotation = unity_frame_transform.getRotation();

      Eigen::Quaternionf eigen_rotation(tf_rotation.w(), tf_rotation.x(), tf_rotation.y(), tf_rotation.z());
      Eigen::Transform<float, 3, Eigen::Affine> world_wrt_stereo_transform = 
            Eigen::Translation3f(tf_translation.x(), tf_translation.y(), tf_translation.z()) * eigen_rotation;
      
      
      
      // getting transformation between unity wrt world
      Eigen::Quaternionf unity_eigen_rotation(msg.SR_Mocap.orientation.w, msg.SR_Mocap.orientation.x, msg.SR_Mocap.orientation.y, msg.SR_Mocap.orientation.z);
      Eigen::Transform<float, 3, Eigen::Affine> unity_wrt_world_transform = 
            Eigen::Translation3f(msg.SR_Mocap.orientation.x, msg.SR_Mocap.orientation.y, msg.SR_Mocap.orientation.z) * unity_eigen_rotation;
      
      
      
      // getting final transformation
      Eigen::Transform<float, 3, Eigen::Affine> unity_frame_wrt_left_stereo_transform = world_wrt_stereo_transform * unity_wrt_world_transform * transform;
      
      
      // applying transfrom on pointcloud 
      pcl::transformPointCloud(*pt_cloud, *transformed_cloud, transform);
      
      
       // filtering from celling
      //pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      //double ceiling_height = 1.0;
       //for (size_t i = 0; i < transformed_cloud->points.size(); ++i)
       //{
           // Check the Z-coordinate (height) of each point
        //   if (transformed_cloud->points[i].y <= ceiling_height)
         //  {
               // Add the point to the filtered point cloud
          //     filtered_cloud->points.push_back(transformed_cloud->points[i]);
          // }
      // }

      
      
      // Publishing point cloud to cloud_in
      sensor_msgs::PointCloud2 output_pc;

      pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2());
      pcl::toPCLPointCloud2(*transformed_cloud, *pcl_pc2); // need to create pc_BF_
      pcl_conversions::moveFromPCL(*pcl_pc2, output_pc);

      output_pc.header.frame_id = "unity_frame";
      output_pc.header.stamp = ros::Time::now();
      output_pc.width = msg.data_x.size();
      
      hololens_pointcloud.publish(output_pc);

      ROS_ERROR("Found transformation");
      break;

    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("Transform not found %s",ex.what());
      ros::Duration(0.1).sleep();
    }



  }
  
 

  // sensor_msgs::PointCloud2 output_pc;
  
  // pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2());
  // pcl::toPCLPointCloud2(*transformed_cloud, *pcl_pc2); // need to create pc_BF_
  // pcl_conversions::moveFromPCL(*pcl_pc2, output_pc);

  // output_pc.header.frame_id = "unity_frame";
  // output_pc.header.stamp = ros::Time::now();
  // output_pc.width = msg.data_x.size();
  
  // hololens_pointcloud.publish(output_pc);

  ROS_ERROR("Holo point published");
  
}

/*
* Publisher
*/

 void PerceptionServer::publish_mesh_to_unity(visualization_msgs::Marker triangles) {
    //Iterate on the triangles and republish them as geometry messsage array
    scene_understanding_pkg_msgs::MeshVertexPosition msg_point;
    geometry_msgs::Point p;
    
    cout << "Points Published to Unity: " << triangles.points.size() << endl;
    for (int ii = 0; ii < triangles.points.size(); ii++)
    {
        p = triangles.points[ii];
        msg_point.point.push_back(p);
    }
    
    if (msg_point.point.size() > 0)
    {
    	 std_msgs::Float64 timeMsg;
        timeMsg.data = ros::Time::now().toSec();
        this->mesh_to_unity.publish(msg_point);
        drone_mesh_to_unity_time.publish(timeMsg);
    }
       
   }

void PerceptionServer::publish_obs_force_array_marker(float theta)
{
   float roll = 0.0;
   float pitch = 0.0;
   float yaw = theta;

   Eigen::Vector4f quat;
   perception_utils.euler_to_quat(&quat, roll, pitch, yaw);
  //Convert theta from radians orientation (0,0, theta) only on the yaw to a quaternion value. 

  obs_magn_force_b_frame.pose.orientation.x = quat(0);
  obs_magn_force_b_frame.pose.orientation.y = quat(1);
  obs_magn_force_b_frame.pose.orientation.z = quat(2);
  obs_magn_force_b_frame.pose.orientation.w = quat(3);

 
  
  //Position is the position of the robot
  obs_magn_force_b_frame.pose.position.x  = ( mocap_sr_unity_world_position.x + quad_position.x); 
  obs_magn_force_b_frame.pose.position.y  = ( mocap_sr_unity_world_position.y + quad_position.y); 
  obs_magn_force_b_frame.pose.position.z  = quad_position.z; 

  obs_magn_force_b_frame.scale.x = 0.2 * F_magn;
  marker_pub_force_arrow_rviz.publish(obs_magn_force_b_frame);


}

void PerceptionServer::publish_obstacle_force_resultant(float fx, float fy, float Fm, Eigen::Vector2f pd, Eigen::Vector2f pd_dot)
{
    scene_understanding_pkg_msgs::ObstacleRepForce msg;
    msg.Fx = fx;
    msg.Fy = fy;
    msg.F_magn = Fm;
    msg.pd[0] = pd(0);
    msg.pd[1] = pd(1);
    msg.pd_dot[0] = pd_dot(0);
    msg.pd_dot[1] = pd_dot(1);
    obst_force.publish(msg);
}


/*
* Publish TF tranform
*/
void PerceptionServer::publish_mocap_sim_tf()
{
    //TF Publisher  
    ros::Time current_time_tf, last_time_tf;
    current_time_tf = ros::Time::now();
    last_time_tf = ros::Time::now();

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_tf;
    odom_trans.header.frame_id = "mocap";
    odom_trans.child_frame_id = "simulator";

    odom_trans.transform.rotation.w = 1.0;

    //send the transform
    mocap_to_sim_broadcaster.sendTransform(odom_trans);
}

} //namespace
