
#include <Eigen/Core>
#include <nlohmann/json.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
#include <opencv2/opencv.hpp>
#pragma GCC diagnostic pop
#include <okvis/ThreadedKFVio.hpp>
#include <okvis/VioParametersReader.hpp>

class PoseRecorder {
    public:
    PoseRecorder(std::string_view outputFile) {
      os_.open(outputFile, ios::out | ios::trunc);
      outputJson_ = = R"({
        "time": 0.0,
        "position": {
          "x": 0.0,
          "y": 0.0,
          "z": 0.0
        },
        "orientation": {
          "w": 0.0,
          "x": 0.0,
          "y": 0.0,
          "z": 0.0
        }
      })"_json;
      void saveFullStateAsCallback(const okvis::Time &t,
                                  const okvis::kinematics::Transformation &T_WS,
                                  const Eigen::Matrix<double, 9, 1> &speedAndBiases,
                                  const Eigen::Matrix<double, 3, 1> & /*omega_S*/){
        // just append the path
        Eigen::Vector3d r = T_WS.r();
        Eigen::Quaterniond q = T_WS.q();                         
        // Parse the position and quaternion
        outputJson["time"] = t;
        outputJson["position"]["x"] = r(0);
        outputJson["position"]["y"] = r(1);
        outputJson["position"]["z"] = r(2);
        outputJson["orientation"]["w"] = q.w(); // w == 3
        outputJson["orientation"]["x"] = q.x();
        outputJson["orientation"]["y"] = q.y();
        outputJson["orientation"]["z"] = q.z();
        // Dump to the outputfile 
        os_ << outputJson.dump() << "\n";
      }
    }
    private:
      std::ofstream os_(outputFile);
      nlohmann::json outputJson_;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  if (argc != 3 && argc != 4) {
    LOG(ERROR) << "Usage: ./" << argv[0] << " configuration-yaml-file dataset-folder [skip-first-seconds]";
    return -1;
  }

  okvis::Duration deltaT(0.0);
  if (argc == 4) {
    deltaT = okvis::Duration(atof(argv[3]));
  }

  // read configuration file
  std::string configFilename(argv[1]);

  okvis::VioParametersReader vio_parameters_reader(configFilename);
  okvis::VioParameters parameters;
  vio_parameters_reader.getParameters(parameters);

  okvis::ThreadedKFVio okvis_estimator(parameters);

  PoseRecorder poseRecorder;
  okvis_estimator.setFullStateCallback(std::bind(&PoseRecorder::saveFullStateAsCallback,
                                                &poseRecorder,
                                                std::placeholders::_1,
                                                std::placeholders::_2,
                                                std::placeholders::_3,
                                                std::placeholders::_4));

  okvis_estimator.setBlocking(true);
    // extract the folder path
  std::string bagname(argv[2]);
  size_t pos = bagname.find_last_of("/");
  std::string path;
  if (pos == std::string::npos)
    path = ".";
  else
    path = bagname.substr(0, pos);

  const unsigned int numCameras = parameters.nCameraSystem.numCameras();

  // open the bag
  rosbag::Bag bag(argv[2], rosbag::bagmode::Read);

    // views on topics. the slash is needs to be correct, it's ridiculous...
  std::string imu_topic("/imu0");
  rosbag::View view_imu(bag, rosbag::TopicQuery(imu_topic));
  if (view_imu.size() == 0) {
    LOG(ERROR) << "no imu topic";
    return -1;
  }
  rosbag::View::iterator view_imu_iterator = view_imu.begin();
  LOG(INFO) << "No. IMU messages: " << view_imu.size();


  std::vector<std::shared_ptr<rosbag::View> > view_cams_ptr;
  std::vector<rosbag::View::iterator> view_cam_iterators;
  std::vector<okvis::Time> times;
  okvis::Time latest(0);
  for (size_t i = 0; i < numCameras; ++i) {
    std::string camera_topic("/cam" + std::to_string(i) + "/image_raw");
    std::shared_ptr<rosbag::View> view_ptr(new rosbag::View(bag, rosbag::TopicQuery(camera_topic)));
    if (view_ptr->size() == 0) {
      LOG(ERROR) << "no camera topic";
      return 1;
    }
    view_cams_ptr.push_back(view_ptr);
    view_cam_iterators.push_back(view_ptr->begin());
    sensor_msgs::ImageConstPtr msg1 = view_cam_iterators[i]->instantiate<sensor_msgs::Image>();
    times.push_back(okvis::Time(msg1->header.stamp.sec, msg1->header.stamp.nsec));
    if (times.back() > latest) latest = times.back();
    LOG(INFO) << "No. cam " << i << " messages: " << view_cams_ptr.back()->size();
  }

  for (size_t i = 0; i < numCameras; ++i) {
    if ((latest - times[i]).toSec() > 0.01) view_cam_iterators[i]++;
  }

  int counter = 0;
  okvis::Time start(0.0);
  while (ros::ok()) {
    ros::spinOnce();
    okvis_estimator.display();

    // check if at the end
    if (view_imu_iterator == view_imu.end()) {
      std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
      char k = 0;
      while (k == 0 && ros::ok()) {
        k = cv::waitKey(1);
        ros::spinOnce();
      }
      return 0;
    }
    for (size_t i = 0; i < numCameras; ++i) {
      if (view_cam_iterators[i] == view_cams_ptr[i]->end()) {
        std::cout << std::endl << "Finished. Press any key to exit." << std::endl << std::flush;
        char k = 0;
        while (k == 0 && ros::ok()) {
          k = cv::waitKey(1);
          ros::spinOnce();
        }
        return 0;
      }
    }

    // add images
    okvis::Time t;
    for (size_t i = 0; i < numCameras; ++i) {
      sensor_msgs::ImageConstPtr msg1 = view_cam_iterators[i]->instantiate<sensor_msgs::Image>();
      cv::Mat filtered(msg1->height, msg1->width, CV_8UC1);
      memcpy(filtered.data, &msg1->data[0], msg1->height * msg1->width);
      t = okvis::Time(msg1->header.stamp.sec, msg1->header.stamp.nsec);
      if (start == okvis::Time(0.0)) {
        start = t;
      }

      // get all IMU measurements till then
      okvis::Time t_imu = start;
      do {
        sensor_msgs::ImuConstPtr msg = view_imu_iterator->instantiate<sensor_msgs::Imu>();
        Eigen::Vector3d gyr(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        Eigen::Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        t_imu = okvis::Time(msg->header.stamp.sec, msg->header.stamp.nsec);

        // add the IMU measurement for (blocking) processing
        if (t_imu - start > deltaT) okvis_estimator.addImuMeasurement(t_imu, acc, gyr);

        view_imu_iterator++;
      } while (view_imu_iterator != view_imu.end() && t_imu <= t);

      // add the image to the frontend for (blocking) processing
      if (t - start > deltaT) okvis_estimator.addImage(t, i, filtered);

      view_cam_iterators[i]++;
    }
    ++counter;

    // display progress
    if (counter % 20 == 0) {
      std::cout << "\rProgress: " << int(double(counter) / double(view_cams_ptr.back()->size()) * 100) << "%  ";
    }
  }

  std::cout << std::endl;
  return 0;
}