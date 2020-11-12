#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

int main(int argc, char* argv[]){

  std::string input_bag_name = argv[1];
  rosbag::Bag input_bag{input_bag_name};

  rosbag::View img_view(input_bag, rosbag::TopicQuery("/ladybug/camera4/rectified/cropped/image_mono"));
  rosbag::View imu_view(input_bag, rosbag::TopicQuery("/imu/data"));

  rosbag::Bag output_bag{"test.bag", rosbag::bagmode::Write};


  using ViewIter = rosbag::View::iterator;
  using ViewIterVec = std::vector<rosbag::View::iterator>;
  using MsgInst = rosbag::MessageInstance;
  using MsgInstVec =  std::vector<rosbag::MessageInstance>;

  MsgInstVec msg_instances;

  ViewIter imu_iter_last = imu_view.begin();

  std::cout << "Re-writing bag" << std::endl;

   for (ViewIter img_inst = img_view.begin(); img_inst != img_view.end();++img_inst){
    auto img_msg = img_inst->instantiate<sensor_msgs::Image>();

     for (ViewIter imu_inst = imu_iter_last; imu_inst != imu_view.end(); ++imu_inst){
       auto imu_msg = imu_inst->instantiate<sensor_msgs::Imu>();

       // If IMU msg came before Image message, write it to bag first
       if (imu_msg->header.stamp < img_msg->header.stamp){
         output_bag.write("/imu/data", imu_msg->header.stamp, imu_msg);
         imu_iter_last = imu_inst;
         imu_iter_last++;
         continue;
       } else {
         output_bag.write("/ladybug/camera4/rectified/cropped/image_mono", img_msg->header.stamp, img_msg);
         break;
       }
     }

  }













  /*

  std::vector<std::pair<void*, ros::Time>> msgs;

  for (auto& msg_inst : rosbag::View(input_bag)) {
    ros::Time msg_stamp;
    if(msg_inst.isType<sensor_msgs::Imu>()) {
      auto imu_msg = msg_inst.instantiate<sensor_msgs::Imu>();
      msg_stamp = imu_msg->header.stamp;
    } else if (msg_inst.isType<sensor_msgs::Image>()){
      auto img_msg = msg_inst.instantiate<sensor_msgs::Image>();
      msg_stamp = img_msg->header.stamp;
    }
//    auto p = std::make_pair((void*)(&msg_inst), msg_stamp);
    msgs.emplace_back((void*)(&msg_inst), msg_stamp);
//    msgs.push_back((void*)(&msg_inst), msg_stamp);
//    msg_instances.emplace_back(msg_inst);
  }

  std::cout << "Sorting..." << std::endl;

    std::sort(msgs.begin(), msgs.end(),
      [](const auto& k1, const auto& k2){
    return k1.second.sec < k2.second.sec;
  });

  std::cout << "Writing new bag..." << std::endl;

  for (const auto& msg : msgs){

  auto msg_inst = static_cast<MsgInst*>(msg.first);
  std::cout << msg_inst->getTime() << std::endl;
    if(msg_inst->isType<sensor_msgs::Imu>()) {
      std::cout << "Writing imu msg" << std::endl;
      auto imu_msg = msg_inst->instantiate<sensor_msgs::Imu>();
      output_bag.write("/imu/data", imu_msg->header.stamp, imu_msg);

    } else if (msg_inst->isType<sensor_msgs::Image>()){

      auto img_msg = msg_inst->instantiate<sensor_msgs::Image>();
      output_bag.write("/ladybug/camera4/rectified/image_mono", img_msg->header.stamp, img_msg);

    }

  }
  output_bag.close();

*/


/*  for (rosbag::View::iterator imu_inst = imu_view.begin(); imu_inst != imu_view.end(); ++imu_inst){
    auto imu_msg = imu_inst->instantiate<sensor_msgs::Imu>();

    for (rosbag::View::iterator img_inst = img_view.begin(); img_inst != img_view.end(); ++img_inst){
      auto img_msg = img_inst->instantiate<sensor_msgs::Image>();

      if (imu_msg->header.stamp < img_msg->header.stamp){
        output_bag.write("/imu/data", imu_msg->header.stamp, imu_msg);
        continue;
      }

    }
  }*/

  return 0;
}