//box_inspector.cpp implementation of class/library
#include <box_inspector/box_inspector.h>
//#include "box_inspector_fncs.cpp" //more code, outside this file
#include "box_inspector_fncs.cpp" //more code, outside this file
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <cmath>

BoxInspector::BoxInspector(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
  //set up camera subscriber:
   box_camera_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1, &BoxInspector::box_camera_callback, this);
   got_new_snapshot_=false; //trigger to get new snapshots

}

//to request a new snapshot, set need_new_snapshot_ = true, and make sure to give a ros::spinOnce()
void BoxInspector::box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    if (!got_new_snapshot_) {
        box_inspector_image_ = *image_msg;  //copy the current message to a member data var, i.e. freeze the snapshot
        got_new_snapshot_ =  true;
        ROS_INFO_STREAM("received box-camera image of: "<<box_inspector_image_<<endl);
        int n_models = box_inspector_image_.models.size();
        ROS_INFO("%d models seen ",n_models);
    }
}

//method to request a new snapshot from logical camera; blocks until snapshot is ready,
// then result will be in box_inspector_image_
void BoxInspector::get_new_snapshot_from_box_cam() {
  got_new_snapshot_= false;
  ROS_INFO("waiting for snapshot from camera");
  while (!got_new_snapshot_) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("got new snapshot");
}


//here is  the main fnc; provide a list of models, expressed as desired parts w/ poses w/rt box;
//get a box-camera logical image and  parse it
//populate the vectors as follows:
// satisfied_models_wrt_world: vector of models that match shipment specs, including precision location in box
// misplaced_models_wrt_world: vector of models that belong in the shipment, but are imprecisely located in box
// missing_models_wrt_world:  vector of models that are requested in the shipment, but not yet present in the box
// orphan_models_wrt_world: vector of models that are seen in the box, but DO NOT belong in the box
  void BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
       vector<osrf_gear::Model> &satisfied_models_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
       vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
       vector<osrf_gear::Model> &missing_models_wrt_world,
       vector<osrf_gear::Model> &orphan_models_wrt_world) {
  //WRITE ME!!!
  ROS_WARN("NEED TO WRITE update_inspection() ");
  got_new_snapshot_=false;
  while(!got_new_snapshot_) {
     ros::spinOnce(); // refresh camera image
     ros::Duration(0.5).sleep();
     ROS_INFO("waiting for logical camera image");
  }
  ROS_INFO("got new image");
  int num_parts_seen =  box_inspector_image_.models.size()-1;//one item is the BOXXXXX
  int num_parts_desired = desired_models_wrt_world.size();

  std::map<std::string, int> mappings =
{
   {"gear_part",1},
   {"piston_rod_part",2},
   {"gasket_part",3},
   {"disk_part",4},
   {"pulley_part",5}
};

  osrf_gear::Model model;
  osrf_gear::Model model_desired;
  
 
  ROS_INFO("update_inspection: box camera saw %d objects",num_parts_seen);
  ROS_INFO("update_inspection: Ordered  %d objects",num_parts_desired);
  orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
  satisfied_models_wrt_world.clear();  //shipment will be complete when this matches parts/poses specified in shipment
  misplaced_models_actual_coords_wrt_world.clear();
  misplaced_models_desired_coords_wrt_world.clear();
  missing_models_wrt_world.clear();
  int position_tolerance =  1;
  int orientation_tolerance = 0.25; // if I multiply an inverse of a quaternion by the desired orientation it should be approximately 0. I found this number by finding the result when multiplying a quaterion which was 4 degrees off

  int diff_num_seen = num_parts_seen-num_parts_desired;
  
 //tally for how many are ordered
  int array_of_products_ordered[5] = {};

    for(int i = 0; i < num_parts_desired;i++){ 
      model = desired_models_wrt_world[i];
      string model_name(model.type);
      if(model_name == "gear_part"){
        array_of_products_ordered[0] = array_of_products_ordered[0]+1;
      }
      else if(model_name == "piston_rod_part"){
        array_of_products_ordered[1] = array_of_products_ordered[1]+1;
      }
      else if(model_name == "gasket_part"){
        array_of_products_ordered[2] = array_of_products_ordered[2]+1;
      }
      else if(model_name == "disk_part"){
        array_of_products_ordered[3] = array_of_products_ordered[3]+1;
      }
      else if(model_name == "pulley_part"){
        array_of_products_ordered[4] = array_of_products_ordered[4]+1;
      }


    }
 int ID;
  
  for(int i = 0;i<num_parts_seen;i++){


    model = box_inspector_image_.models[i];
    string model_name(model.type);//part I see
      if(model_name == "gear_part"){
        ID = 0;
      }
      else if(model_name == "piston_rod_part"){
        ID = 1;
      }
      else if(model_name == "gasket_part"){
       ID = 2;
      }
      else if(model_name == "disk_part"){
        ID = 3;
      }
      else if(model_name == "pulley_part"){
        ID = 4;
      }

    int number_ordered = array_of_products_ordered[ID]; //number ordered of particular part

    int number_seen = 0; //number seen of particular part;
 
    
    
    for(int j =0;j<num_parts_desired;j++){
      model_desired = desired_models_wrt_world[j];
      string model_desired_name(model_desired.type);//part I ordered
      
      if(model_name == model_desired_name){ //if this is the right part...
        //is it where its supposed to be?
        number_seen = number_seen+1;
        double x_ = model_desired.pose.position.x;
        double y_ = model_desired.pose.position.y;
        double x = model.pose.position.x;
        double y = model.pose.position.y;
        double dx = x_ - x;
        double dy = y_ - y;
        double distance = sqrt(dx*dx +dy*dy);
        
        double quat_phi_x_desired = model_desired.pose.orientation.x;
        double quat_phi_y_desired = model_desired.pose.orientation.y;
        double quat_phi_z_desired = model_desired.pose.orientation.z;
        double quat_phi_w_desired = model_desired.pose.orientation.w;
        double quat_phi_x = model.pose.orientation.x;
        double quat_phi_y = model.pose.orientation.y;
        double quat_phi_z = model.pose.orientation.z;
        double quat_phi_w = model.pose.orientation.w;
        Eigen::Quaterniond quat_actual = {quat_phi_x,quat_phi_y,quat_phi_z,quat_phi_w};
        Eigen::Quaterniond quat_desired = {quat_phi_x_desired,quat_phi_y_desired,quat_phi_z_desired,quat_phi_w_desired};
        

        quat_actual = quat_actual.inverse();

        Eigen::Quaterniond orientation_check_quat = quat_actual*quat_desired;
        double A =  orientation_check_quat.norm()-1;
        





        //now to do the orientation, try using quaternions and if not possible, use a dirty quick method.
      if(A != 0 ){

         if(model.type =="gear_part" || model.type =="gasket_part" || model.type =="disk_part" || model.type =="piston_rod_part" || model.type =="pulley_part" ){
          satisfied_models_wrt_world.push_back(model); 
          model.type = "seen";



         }
      }
      else{
          //is it misplaced or orphaned?
        if(number_seen > number_ordered){ //have an orphaned part.pick first and say and say it's orphaned.
            orphan_models_wrt_world.push_back(model);
            number_seen = number_seen - 1;
        }
        else if(number_seen == number_ordered){ //misplaced parts.
            misplaced_models_actual_coords_wrt_world.push_back(model);
            model.type = "seen";
        }
        else{ //missing
            missing_models_wrt_world.push_back(model_desired);
        }
      }
        
    }
      
   }
  }
  
}

//intent of this function is, get a snapshot from the box-inspection camera;
//parse the image data to see if a shipping box is present
//if not, return false
//if seen, transform box pose to box pose w/rt world frame,
//    copy data to reference arg box_pose_wrt_world
//    and return "true"

bool BoxInspector::get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world) {
    geometry_msgs::Pose cam_pose, box_pose; //cam_pose is w/rt world, but box_pose is w/rt camera

    //get a new snapshot of the box-inspection camera:
    get_new_snapshot_from_box_cam();

    //ROS_INFO("got box-inspection camera snapshot");
    //look for box in model list:
    int num_models = box_inspector_image_.models.size(); //how many models did the camera see?
    if (num_models == 0) return false;
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;
    ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose = model.pose;
            ROS_INFO_STREAM("get_box_pose_wrt_world(): found box at pose " << box_pose << endl);
            //ROS_WARN("USE THIS INFO TO COMPUTE BOX POSE WRT WORLD AND  POPULATE box_pose_wrt_world");
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose);
            ROS_INFO_STREAM("box_pose_wrt_world: " << box_pose_wrt_world << endl);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    ROS_WARN("get_box_pose_wrt_world(): shipping-box not seen!");
    return false;
}

//helper  function:
//given a camera pose and a part-pose (or box-pose) w/rt camera, compute part pose w/rt world
//xform_utils library should help here

geometry_msgs::PoseStamped BoxInspector::compute_stPose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {

    geometry_msgs::PoseStamped stPose_part_wrt_world;
    //compute part-pose w/rt world and return as a pose-stamped message object
    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;
    
    cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(cam_pose);
    part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(part_pose);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    geometry_msgs::Pose pose_part_wrt_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.stamp = ros::Time::now();
    part_pose_stamped.header.frame_id = "world";
    part_pose_stamped.pose = pose_part_wrt_world;
    return part_pose_stamped;
}


//rosmsg show osrf_gear/LogicalCameraImage: 
/*
osrf_gear/Model[] models
  string type
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/
