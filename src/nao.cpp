#include "mc_nao/nao.h"
#include <mc_nao/config.h>

#include <mc_rtc/logging.h>
#include <boost/algorithm/string.hpp>
#include <fstream>

namespace mc_nao
{
const std::string nao_urdf = "naoV50_generated_urdf/nao";

    NAOCommonRobotModule::NAOCommonRobotModule()
    : RobotModule(mc_nao::NAO_DESCRIPTION_PATH, "nao")
{
  LOG_INFO("Loading NAO from: " << mc_nao::NAO_DESCRIPTION_PATH);
  rsdf_dir = path + "/rsdf";
  calib_dir = path + "/calib";

  virtualLinks.push_back("base_link");
  // virtualLinks.push_back("base_link");
  // virtualLinks.push_back("Accelerometer");
  // virtualLinks.push_back("Gyro");
  // virtualLinks.push_back("RightFootForceSensor");
  // virtualLinks.push_back("LeftFootForceSensor");
  // virtualLinks.push_back("LeftHandForceSensor");
  // virtualLinks.push_back("RightHandForceSensor");
  // virtualLinks.push_back("gaze");
  // virtualLinks.push_back("r_gripper_sensor");
  // virtualLinks.push_back("xtion_link");
  // virtualLinks.push_back("r_gripper");
  // virtualLinks.push_back("l_gripper");
  // virtualLinks.push_back("r_sole");
  // virtualLinks.push_back("l_sole");

  // gripperLinks.push_back("R_HAND_J0_LINK");
  // gripperLinks.push_back("R_HAND_J1_LINK");
  // gripperLinks.push_back("R_F22_LINK");
  // gripperLinks.push_back("R_F23_LINK");
  // gripperLinks.push_back("R_F32_LINK");
  // gripperLinks.push_back("R_F33_LINK");
  // gripperLinks.push_back("R_F42_LINK");
  // gripperLinks.push_back("R_F43_LINK");
  // gripperLinks.push_back("R_F52_LINK");
  // gripperLinks.push_back("R_F53_LINK");
  // gripperLinks.push_back("L_HAND_J0_LINK");
  // gripperLinks.push_back("L_HAND_J1_LINK");
  // gripperLinks.push_back("L_F22_LINK");
  // gripperLinks.push_back("L_F23_LINK");
  // gripperLinks.push_back("L_F32_LINK");
  // gripperLinks.push_back("L_F33_LINK");
  // gripperLinks.push_back("L_F42_LINK");
  // gripperLinks.push_back("L_F43_LINK");
  // gripperLinks.push_back("L_F52_LINK");
  // gripperLinks.push_back("L_F53_LINK");

  _accelerometerBody = "";

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
  halfSitting["RAnkleRoll"] = {1.11};
  halfSitting["LShoulderPitch"] = {0.0};
  halfSitting["LShoulderRoll"] = {0.0};
  halfSitting["LElbowYaw"] = {0.0};
  halfSitting["LElbowRoll"] = {-0.79};
  halfSitting["LWristYaw"] = {0.0};
  halfSitting["RShoulderPitch"] = {0.0};
  halfSitting["RShoulderRoll"] = {0.0};
  halfSitting["RElbowYaw"] = {0.0};
  halfSitting["RElbowRoll"] = {0.79};
  halfSitting["RWristYaw"] = {0.0};

  halfSitting["LHand"] = {0.0};
  halfSitting["RHand"] = {0.0};
  halfSitting["RFinger23"] = {0.0};
  halfSitting["RFinger13"] = {0.0};
  halfSitting["RFinger12"] = {0.0};
  halfSitting["LFinger21"] = {0.0};
  halfSitting["LFinger13"] = {0.0};
  halfSitting["LFinger11"] = {0.0};
  halfSitting["RFinger22"] = {0.0};
  halfSitting["LFinger22"] = {0.0};
  halfSitting["RFinger21"] = {0.0};
  halfSitting["LFinger12"] = {0.0};
  halfSitting["RFinger11"] = {0.0};
  halfSitting["LFinger23"] = {0.0};
  halfSitting["LThumb1"] = {0.0};
  halfSitting["RThumb1"] = {0.0};
  halfSitting["RThumb2"] = {0.0};
  halfSitting["LThumb2"] = {0.0};

  LOG_INFO("halfSitting set");

  // _forceSensors.push_back(mc_rbdyn::ForceSensor("RightFootForceSensor", "R_ANKLE_R_LINK", sva::PTransformd(Eigen::Vector3d(0, 0, -0.093))));
  // _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftFootForceSensor", "L_ANKLE_R_LINK", sva::PTransformd(Eigen::Vector3d(0, 0, -0.093))));
  // Eigen::Matrix3d R; R << 0, -1, 0, -1, 0, 0, 0, 0, -1; // rpy="3.14159 0 -1.57079"
  // _forceSensors.push_back(mc_rbdyn::ForceSensor("RightHandForceSensor", "r_wrist", sva::PTransformd(R, Eigen::Vector3d(0, 0, -0.04435))));
  // _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftHandForceSensor", "l_wrist", sva::PTransformd(R, Eigen::Vector3d(0, 0, -0.04435))));

  _minimalSelfCollisions = {};
  // _minimalSelfCollisions = {
  //   mc_rbdyn::Collision("torso", "L_SHOULDER_Y_LINK", 0.02, 0.001, 0.),
  //   mc_rbdyn::Collision("body", "L_ELBOW_P_LINK", 0.05, 0.001, 0.),
  //   mc_rbdyn::Collision("torso", "R_SHOULDER_Y_LINK", 0.02, 0.001, 0.),
  //   mc_rbdyn::Collision("body", "R_ELBOW_P_LINK", 0.05, 0.001, 0.),
  //   mc_rbdyn::Collision("l_wrist", "L_HIP_P_LINK", 0.07, 0.05, 0.),
  //   mc_rbdyn::Collision("r_wrist", "R_HIP_P_LINK", 0.07, 0.05, 0.),
  //   mc_rbdyn::Collision("r_wrist_sub0", "R_WRIST_Y_LINK_sub0", 0.005, 0.001, 0.),
  //   mc_rbdyn::Collision("r_wrist_sub1", "R_WRIST_Y_LINK_sub0", 0.005, 0.001, 0.),
  //   mc_rbdyn::Collision("l_wrist_sub0", "L_WRIST_Y_LINK_sub0", 0.005, 0.001, 0.),
  //   mc_rbdyn::Collision("l_wrist_sub1", "L_WRIST_Y_LINK_sub0", 0.005, 0.001, 0.),
  //   mc_rbdyn::Collision("R_HIP_P_LINK", "body", 0.02, 0.01, 0.),
  //   mc_rbdyn::Collision("L_HIP_P_LINK", "body", 0.02, 0.01, 0.)
  // };

  _commonSelfCollisions = _minimalSelfCollisions;
  // _commonSelfCollisions.push_back(mc_rbdyn::Collision("L_HIP_P_LINK", "body", 0.02, 0.01, 0.));
  // _commonSelfCollisions.push_back(mc_rbdyn::Collision("L_HIP_P_LINK", "R_HIP_P_LINK", 0.02, 0.01, 0.));
  // _commonSelfCollisions.push_back(mc_rbdyn::Collision("L_HIP_P_LINK", "R_KNEE_P_LINK", 0.02, 0.01, 0.));
  // _commonSelfCollisions.push_back(mc_rbdyn::Collision("R_HIP_P_LINK", "L_KNEE_P_LINK", 0.02, 0.01, 0.));
  // _commonSelfCollisions.push_back(mc_rbdyn::Collision("L_KNEE_P_LINK", "R_KNEE_P_LINK", 0.02, 0.01, 0.));
  // _commonSelfCollisions.push_back(mc_rbdyn::Collision("l_ankle", "r_ankle", 0.02, 0.01, 0.));
  // _commonSelfCollisions.push_back(mc_rbdyn::Collision("l_ankle", "R_KNEE_P_LINK", 0.02, 0.01, 0.));
  // _commonSelfCollisions.push_back(mc_rbdyn::Collision("l_ankle", "R_HIP_P_LINK", 0.02, 0.01, 0.));
  // _commonSelfCollisions.push_back(mc_rbdyn::Collision("r_ankle", "L_KNEE_P_LINK", 0.02, 0.01, 0.));
  // _commonSelfCollisions.push_back(mc_rbdyn::Collision("r_ankle", "L_HIP_P_LINK", 0.02, 0.01, 0.));

  _grippers = {};
  // _grippers = {
  //   {"l_gripper", {"L_HAND_J0", "L_HAND_J1"}, false},
  //   {"r_gripper", {"R_HAND_J0", "R_HAND_J1"}, true}
  // };

  _ref_joint_order = {
      "HeadYaw", "HeadPitch", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand", "RFinger23", "RFinger13", "RFinger12", "LFinger21", "LFinger13", "LFinger11", "RFinger22", "LFinger22", "RFinger21", "LFinger12", "RFinger11", "LFinger23", "LThumb1", "RThumb1", "RThumb2", "LThumb2"};

  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.79216}};
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
  LOG_INFO("Loading URDF: " << urdfPath);
  std::ifstream ifs(urdfPath);
  if (ifs.is_open())
  {
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    LOG_INFO("URDF file read, processing...");
    mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), false, filteredLinks, true, "base_link");
    LOG_INFO("URDF parsed");
    mb = res.mb;
    mbc = res.mbc;
    mbg = res.mbg;
    limits = res.limits;

    _visual = res.visual;
    _collisionTransforms = res.collision_tf;
    LOG_SUCCESS("Processed URDF");
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
  // std::map<std::string, std::pair<std::string, std::string>> res;
  // for (const auto& b : mb.bodies())
  // {
  //   // Filter out virtual links without convex files
  //   if (std::find(std::begin(virtualLinks), std::end(virtualLinks), b.name()) == std::end(virtualLinks))
  //   {
  //     res[b.name()] = {b.name(), boost::algorithm::replace_first_copy(b.name(), "_LINK", "")};
  //   }
  // }

  // auto addBody = [&res](const std::string& body, const std::string& file) {
  //   res[body] = {body, file};
  // };
  // addBody("body", "WAIST_LINK");
  // addBody("torso", "CHEST_Y");

  // addBody("R_HIP_Y_LINK", "HIP_Y");
  // addBody("R_HIP_R_LINK", "CHEST_P");
  // addBody("R_ANKLE_P_LINK", "L_ANKLE_P");
  // addBody("r_ankle", "R_FOOT");

  // addBody("L_HIP_Y_LINK", "HIP_Y");
  // addBody("L_HIP_R_LINK", "CHEST_P");
  // addBody("l_ankle", "L_FOOT");

  // addBody("CHEST_P_LINK", "CHEST");

  // addBody("R_SHOULDER_Y_LINK", "SHOULDER_Y");
  // addBody("R_ELBOW_P_LINK", "ELBOW_P");
  // addBody("R_WRIST_P_LINK", "WRIST_P");
  // addBody("r_wrist", "R_WRIST_R");

  // auto finger = [&addBody](const std::string& prefix) {
  //   addBody(prefix + "_HAND_J0_LINK", prefix + "_THUMB");
  //   addBody(prefix + "_HAND_J1_LINK", prefix + "_F1");
  //   for (unsigned int i = 2; i < 6; ++i)
  //   {
  //     std::stringstream key1;
  //     key1 << prefix << "_F" << i << "2_LINK";
  //     std::stringstream key2;
  //     key2 << prefix << "_F" << i << "3_LINK";
  //     addBody(key1.str(), "F2");
  //     addBody(key2.str(), "F3");
  //   }
  // };
  // finger("R");
  // finger("L");

  // addBody("L_SHOULDER_Y_LINK", "SHOULDER_Y");
  // addBody("L_ELBOW_P_LINK", "ELBOW_P");
  // addBody("L_WRIST_P_LINK", "WRIST_P");
  // addBody("l_wrist", "L_WRIST_R");

  // auto addWristSubConvex = [&res](const std::string& prefix) {
  //   std::string wristY = prefix + "_WRIST_Y_LINK";
  //   std::string wristR = boost::algorithm::to_lower_copy(prefix) + "_wrist";
  //   res[wristY + "_sub0"] = {wristY, prefix + "_WRIST_Y_sub0"};
  //   res[wristR + "_sub0"] = {wristR, prefix + "_WRIST_R_sub0"};
  //   res[wristR + "_sub1"] = {wristR, prefix + "_WRIST_R_sub1"};
  // };
  // addWristSubConvex("L");
  // addWristSubConvex("R");

  // return res;
  return {};
}

NAONoHandRobotModule::NAONoHandRobotModule()
{
  for (const auto& gl : gripperLinks)
  {
    filteredLinks.push_back(gl);
  }
  readUrdf(nao_urdf, filteredLinks);

  // _springs.springsBodies = {"l_ankle", "r_ankle"};  //TODO: check these are the correct bodies
  _springs.springsBodies = {};
}

const std::map<std::string, std::pair<std::string, std::string>>& NAONoHandRobotModule::convexHull() const
{
  auto fileByBodyName = stdCollisionsFiles(mb);
  const_cast<NAONoHandRobotModule*>(this)->_convexHull = getConvexHull(fileByBodyName);
  return _convexHull;
}

const std::vector<std::map<std::string, std::vector<double>>>& NAONoHandRobotModule::bounds() const
{
  const_cast<NAONoHandRobotModule*>(this)->_bounds = nominalBounds(limits);
  return _bounds;
}

const std::map<std::string, std::vector<double>>& NAONoHandRobotModule::stance() const
{
  const_cast<NAONoHandRobotModule*>(this)->_stance = halfSittingPose(mb);
  return _stance;
}

NAOWithHandRobotModule::NAOWithHandRobotModule()
{
  readUrdf(nao_urdf, filteredLinks);

  // _springs.springsBodies = {"l_ankle", "r_ankle"};  //TODO: check these are the correct bodies
  _springs.springsBodies = {};  //TODO: check these are the correct bodies
}

const std::map<std::string, std::pair<std::string, std::string>>& NAOWithHandRobotModule::convexHull() const
{
  auto fileByBodyName = stdCollisionsFiles(mb);
  const_cast<NAOWithHandRobotModule*>(this)->_convexHull = getConvexHull(fileByBodyName);
  return _convexHull;
}

const std::vector<std::map<std::string, std::vector<double>>>& NAOWithHandRobotModule::bounds() const
{
  const_cast<NAOWithHandRobotModule*>(this)->_bounds = nominalBounds(limits);
  return _bounds;
}

const std::map<std::string, std::vector<double>>& NAOWithHandRobotModule::stance() const
{
  const_cast<NAOWithHandRobotModule*>(this)->_stance = halfSittingPose(mb);
  return _stance;
}
}
