#include <mc_nao/nao.h>
#include <mc_nao/config.h>
#include <fstream>

namespace mc_robots
{
NAOCommonRobotModule::NAOCommonRobotModule()
    : RobotModule(mc_rtc::NAO_DESCRIPTION_PATH, "nao")
{
  LOG_INFO("Loading NAO from: " << mc_rtc::NAO_DESCRIPTION_PATH);
  rsdf_dir = path + "/rsdf";
  calib_dir = path + "/calib";

  gripperLinks.push_back("l_gripper");
  gripperLinks.push_back("r_gripper");
  gripperLinks.push_back("LFinger11_link");
  gripperLinks.push_back("LFinger12_link");
  gripperLinks.push_back("LFinger13_link");
  gripperLinks.push_back("LFinger21_link");
  gripperLinks.push_back("LFinger22_link");
  gripperLinks.push_back("LFinger23_link");
  gripperLinks.push_back("LThumb1_link");
  gripperLinks.push_back("LThumb2_link");

  gripperLinks.push_back("RFinger11_link");
  gripperLinks.push_back("RFinger12_link");
  gripperLinks.push_back("RFinger13_link");
  gripperLinks.push_back("RFinger21_link");
  gripperLinks.push_back("RFinger22_link");
  gripperLinks.push_back("RFinger23_link");
  gripperLinks.push_back("RThumb1_link");
  gripperLinks.push_back("RThumb2_link");

  _bodySensors.emplace_back("Accelerometer", "torso", sva::PTransformd(Eigen::Vector3d(-0.008, 0.00606, 0.027)));
  _bodySensors.emplace_back("Gyrometer", "torso", sva::PTransformd(Eigen::Vector3d(-0.008, 0.006, 0.029)));

  halfSitting["HeadYaw"] = {0.0};
  halfSitting["HeadPitch"] = {0.0};

  halfSitting["LHipYawPitch"] = {0.0};
  halfSitting["LHipRoll"] = {0.0};
  halfSitting["LHipPitch"] = {0.0};
  halfSitting["LKneePitch"] = {0.0};
  halfSitting["LAnklePitch"] = {0.0};
  halfSitting["LAnkleRoll"] = {0.0};

  halfSitting["RHipYawPitch"] = {0.0};
  halfSitting["RHipRoll"] = {0.0};
  halfSitting["RHipPitch"] = {0.0};
  halfSitting["RKneePitch"] = {0.0};
  halfSitting["RAnklePitch"] = {0.0};
  halfSitting["RAnkleRoll"] = {0.0};

  halfSitting["LShoulderPitch"] = {1.49 * 180 / M_PI};
  halfSitting["LShoulderRoll"] = {0.30 * 180 / M_PI};
  halfSitting["LElbowYaw"] = {0.0};
  halfSitting["LElbowRoll"] = {-0.28 * 180 / M_PI};
  halfSitting["LWristYaw"] = {0.0};
  halfSitting["LHand"] = {0.50 * 180 / M_PI};
  halfSitting["LFinger11"] = {0.0};
  halfSitting["LFinger12"] = {0.0};
  halfSitting["LFinger13"] = {0.0};
  halfSitting["LFinger21"] = {0.0};
  halfSitting["LFinger22"] = {0.0};
  halfSitting["LFinger23"] = {0.0};
  halfSitting["LThumb1"] = {0.0};
  halfSitting["LThumb2"] = {0.0};

  halfSitting["RShoulderPitch"] = {1.49 * 180 / M_PI};
  halfSitting["RShoulderRoll"] = {-0.30 * 180 / M_PI};
  halfSitting["RElbowYaw"] = {0.0};
  halfSitting["RElbowRoll"] = {0.28 * 180 / M_PI};
  halfSitting["RWristYaw"] = {0.0};
  halfSitting["RHand"] = {0.50 * 180 / M_PI};
  halfSitting["RFinger13"] = {0.0};
  halfSitting["RFinger12"] = {0.0};
  halfSitting["RFinger11"] = {0.0};
  halfSitting["RFinger21"] = {0.0};
  halfSitting["RFinger22"] = {0.0};
  halfSitting["RFinger23"] = {0.0};
  halfSitting["RThumb1"] = {0.0};
  halfSitting["RThumb2"] = {0.0};

  // Foot force sensors.
  // XXX parse position from URDF
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LFsrFR", "l_ankle", sva::PTransformd(Eigen::Vector3d(0.07025, -0.0231, -0.04511))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LFsrRR", "l_ankle", sva::PTransformd(Eigen::Vector3d(-0.02965, -0.0191, -0.04511))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LFsrFL", "l_ankle", sva::PTransformd(Eigen::Vector3d(0.07025, 0.0299, -0.04511))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LFsrRL", "l_ankle", sva::PTransformd(Eigen::Vector3d(-0.03025, 0.0299, -0.04511))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RFsrFL", "r_ankle", sva::PTransformd(Eigen::Vector3d(0.07025, 0.0231, -0.04511))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RFsrRL", "r_ankle", sva::PTransformd(Eigen::Vector3d(-0.03025, 0.0191, -0.04511))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RFsrRR", "r_ankle", sva::PTransformd(Eigen::Vector3d(-0.02965, -0.0299, -0.04511))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RFsrFR", "r_ankle", sva::PTransformd(Eigen::Vector3d(0.07025, -0.0299, -0.04511))));
  // XXX fix position
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LF_TOTAL_WEIGHT", "l_ankle", sva::PTransformd(Eigen::Vector3d(0.07025, -0.0299, -0.04511))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RF_TOTAL_WEIGHT", "r_ankle", sva::PTransformd(Eigen::Vector3d(0.07025, -0.0299, -0.04511))));

  Eigen::Matrix3d rotation_wrist_sensor;
  rotation_wrist_sensor << 0.0, 0.0, 1.0,
                           0.0, -1.0, 0.0,
                           1.0, 0.0, 0.0;

  _forceSensors.push_back(mc_rbdyn::ForceSensor("LH_FORCE", "l_wrist", sva::PTransformd(rotation_wrist_sensor)));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RH_FORCE", "r_wrist", sva::PTransformd(rotation_wrist_sensor)));

  _minimalSelfCollisions = {
      mc_rbdyn::Collision("Head", "l_wrist", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("Head", "r_wrist", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("Head", "LForeArm", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("Head", "RForeArm", 0.02, 0.01, 0.),
      // mc_rbdyn::Collision("Head", "LBicep", 0.02, 0.01, 0.),
      // mc_rbdyn::Collision("Head", "RBicep", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("xtion_link", "l_wrist", 0.02, 0.02, 0.),
      mc_rbdyn::Collision("xtion_link", "r_wrist", 0.02, 0.02, 0.),
      mc_rbdyn::Collision("xtion_link", "LForeArm", 0.02, 0.02, 0.),
      mc_rbdyn::Collision("xtion_link", "RForeArm", 0.02, 0.02, 0.),
      mc_rbdyn::Collision("LThigh", "l_wrist", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("RThigh", "r_wrist", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("l_wrist", "r_wrist", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("l_wrist", "torso", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("r_wrist", "torso", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("l_ankle", "r_ankle", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("l_ankle", "RTibia", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("r_ankle", "LTibia", 0.02, 0.01, 0.)
      // mc_rbdyn::Collision("LThigh", "RThigh", 0.01, 0.001, 0.),
      // mc_rbdyn::Collision("LTibia", "RTibia", 0.01, 0.001, 0.)
  };

  _commonSelfCollisions = _minimalSelfCollisions;

  // Gripper's name,  Active joints in the gripper,
  // Whether the limits should be reversed, see mc_control::Gripper
  // _grippers = {};
  _grippers =
      {
          {"l_gripper", {"LHand"}, false},
          {"r_gripper", {"RHand"}, false},
      };

  // _springs.springsBodies = {"l_ankle", "r_ankle"};  //TODO: check these are the correct bodies
  _springs.springsBodies = {};  //TODO: check these are the correct bodies

  _ref_joint_order = {
      "HeadPitch", "HeadYaw", "LAnklePitch", "LAnkleRoll", "LElbowRoll", "LElbowYaw", "LHand", "LHipPitch", "LHipRoll", "LHipYawPitch", "LKneePitch", "LShoulderPitch", "LShoulderRoll", "LWristYaw", "RAnklePitch", "RAnkleRoll", "RElbowRoll", "RElbowYaw", "RHand", "RHipPitch", "RHipRoll", "RKneePitch", "RShoulderPitch", "RShoulderRoll", "RWristYaw"};

  // Posture of base link in half-sitting for when no attitude is available.
  // (quaternion, translation)
  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.332674}};
  LOG_SUCCESS("NAOCommonRobotModule initialized");
}

std::map<std::string, std::pair<std::string, std::string>> NAOCommonRobotModule::getConvexHull(const std::map<std::string, std::pair<std::string, std::string>>& files) const
{
  std::string convexPath = path + "/convex/";
  std::map<std::string, std::pair<std::string, std::string>> res;
  for (const auto& f : files)
  {
    res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
  }
  return res;
}

void NAOCommonRobotModule::readUrdf(const std::string& robotName, const std::vector<std::string>& filteredLinks)
{
  std::string urdfPath = path + "/urdf/" + robotName + ".urdf";
  std::ifstream ifs(urdfPath);
  if (ifs.is_open())
  {
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), false, filteredLinks, true, "base_link");
    mb = res.mb;
    mbc = res.mbc;
    mbg = res.mbg;
    limits = res.limits;

    _visual = res.visual;
    _collisionTransforms = res.collision_tf;
  }
  else
  {
    LOG_ERROR("Could not open NAO model at " << urdfPath)
    throw("Failed to open NAO model");
  }
}

std::map<std::string, std::vector<double>> NAOCommonRobotModule::halfSittingPose(const rbd::MultiBody& mb) const
{
  std::map<std::string, std::vector<double>> res;
  for (const auto& j : mb.joints())
  {
    if (halfSitting.count(j.name()))
    {
      res[j.name()] = halfSitting.at(j.name());
      for (auto& ji : res[j.name()])
      {
        ji = M_PI * ji / 180;
      }
    }
    else if (j.name() != "Root" && j.dof() > 0)
    {
      LOG_WARNING("Joint " << j.name() << " has " << j.dof() << " dof, but is not part of half sitting posture.");
    }
  }
  return res;
}

std::vector<std::map<std::string, std::vector<double>>> NAOCommonRobotModule::nominalBounds(const mc_rbdyn_urdf::Limits& limits) const
{
  std::vector<std::map<std::string, std::vector<double>>> res(0);
  res.push_back(limits.lower);
  res.push_back(limits.upper);
  {
    auto mvelocity = limits.velocity;
    for (auto& mv : mvelocity)
    {
      for (auto& mvi : mv.second)
      {
        mvi = -mvi;
      }
    }
    res.push_back(mvelocity);
  }
  res.push_back(limits.velocity);
  {
    auto mtorque = limits.torque;
    for (auto& mt : mtorque)
    {
      for (auto& mti : mt.second)
      {
        mti = -mti;
      }
    }
    res.push_back(mtorque);
  }
  res.push_back(limits.torque);
  return res;
}

std::map<std::string, std::pair<std::string, std::string>> NAOCommonRobotModule::stdCollisionsFiles(const rbd::MultiBody& /*mb*/) const
{
  std::map<std::string, std::pair<std::string, std::string>> res;

  // Manually add all convex for bodies
  auto addBody = [&res](const std::string& body, const std::string& file) {
    res[body] = {body, file};
  };
  // Add correspondance between link and corresponding CH name
  addBody("Head", "HeadPitch");
  addBody("xtion_link", "ASUS_XTION");
  addBody("LBicep", "LShoulderRoll");
  addBody("RBicep", "RShoulderRoll");
  addBody("LForeArm", "LElbowRoll");
  addBody("RForeArm", "RElbowRoll");
  addBody("RThigh", "RHipPitch");
  addBody("LThigh", "LHipPitch");
  addBody("r_wrist", "RWristYaw");
  addBody("l_wrist", "LWristYaw");
  addBody("torso", "Torso");
  addBody("LThigh", "LHipPitch");
  addBody("LTibia", "LKneePitch");
  addBody("l_ankle", "LAnkleRoll");
  addBody("RThigh", "RHipPitch");
  addBody("RTibia", "RKneePitch");
  addBody("r_ankle", "RAnkleRoll");
  return res;
}

const std::map<std::string, std::pair<std::string, std::string>>& NAOCommonRobotModule::convexHull() const
{
  return _convexHull;
}

const std::vector<std::map<std::string, std::vector<double>>>& NAOCommonRobotModule::bounds() const
{
  return _bounds;
}

const std::map<std::string, std::vector<double>>& NAOCommonRobotModule::stance() const
{
  return _stance;
}

NAONoHandRobotModule::NAONoHandRobotModule() : NAOCommonRobotModule()
{
  for (const auto& gl : gripperLinks)
  {
    filteredLinks.push_back(gl);
  }
  readUrdf("nao", filteredLinks);
  auto fileByBodyName = stdCollisionsFiles(mb);
  _bounds = nominalBounds(limits);
  _stance = halfSittingPose(mb);
  _convexHull = getConvexHull(fileByBodyName);
  LOG_SUCCESS("NOANoHandRobotModule intialized");
}

NAOWithHandRobotModule::NAOWithHandRobotModule() : NAOCommonRobotModule()
{
  readUrdf("nao", filteredLinks);
  auto fileByBodyName = stdCollisionsFiles(mb);
  _bounds = nominalBounds(limits);
  _stance = halfSittingPose(mb);
  _convexHull = getConvexHull(fileByBodyName);
  LOG_SUCCESS("NAOWithHandRobotModule initialized");
}
}
