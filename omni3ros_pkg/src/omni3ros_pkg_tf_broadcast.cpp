 #include <ros/ros.h>
   #include <tf/transform_broadcaster.h>
   #include <turtlesim/Pose.h>
   
   std::string omni3ros_pkg;
   
   
   
   void poseCallback(const turtlesim::PoseConstPtr& msg){
     static tf::TransformBroadcaster br;
     tf::Transform transform;
     transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
     tf::Quaternion q;
     q.setRPY(0, 0, msg->theta);
     transform.setRotation(q);
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", omni3ros_pkg));
   }
   
   int main(int argc, char** argv){
     ros::init(argc, argv, "my_tf_broadcaster");
     if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
     omni3ros_pkg = argv[1];
   
     ros::NodeHandle node;
     ros::Subscriber sub = node.subscribe(omni3ros_pkg+"/pose", 10, &poseCallback);
   
     ros::spin();
     return 0;
   };
