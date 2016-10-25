#include "mc_nao/nao.h"
#include <mc_nao/config.h>

#include <mc_rtc/logging.h>
#include <boost/algorithm/string.hpp>
#include <fstream>

namespace mc_nao
{
const std::string nao_urdf = "nao";

NAOCommonRobotModule::NAOCommonRobotModule()
    : RobotModule(mc_nao::NAO_DESCRIPTION_PATH, "nao")
{
  LOG_INFO("Loading NAO from: " << mc_nao::NAO_DESCRIPTION_PATH);
  rsdf_dir = path + "/rsdf";
  calib_dir = path + "/calib";

  // virtualLinks = {"base_link", "Neck", "Head", "gaze", "LPelvis", "LHip", "LThigh", "LTibia", "LAnklePitch", "l_ankle", "l_sole", "RPelvis", "RHip", "RThigh", "RTibia", "RAnklePitch", "r_ankle", "r_sole", "torso",
  // "LShoulder", "LBicep", "LElbow", "LForeArm", "l_wrist", "l_gripper",
  // "RShoulder", "RBicep", "RElbow", "RForeArm", "r_wrist", "r_gripper",
  // "LFootBumperRight_frame", "RFsrRL_frame", "CameraBottom_optical_frame",
  // "RFsrRR_frame", "LFsrFR_frame", "LHandTouchBack_frame", "LHandTouchLeft_frame"};

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
  halfSitting["RAnkleRoll"] = {0.0};

  halfSitting["LShoulderPitch"] = {0.0};
  halfSitting["LShoulderRoll"] = {0.0};
  halfSitting["LElbowYaw"] = {0.0};
  halfSitting["LElbowRoll"] = {-0.79};
  halfSitting["LWristYaw"] = {0.0};
  halfSitting["LHand"] = {0.0};
  halfSitting["LFinger11"] = {0.0};
  halfSitting["LFinger12"] = {0.0};
  halfSitting["LFinger13"] = {0.0};
  halfSitting["LFinger21"] = {0.0};
  halfSitting["LFinger22"] = {0.0};
  halfSitting["LFinger23"] = {0.0};
  halfSitting["LThumb1"] = {0.0};
  halfSitting["LThumb2"] = {0.0};

  halfSitting["RShoulderPitch"] = {0.0};
  halfSitting["RShoulderRoll"] = {0.0};
  halfSitting["RElbowYaw"] = {0.0};
  halfSitting["RElbowRoll"] = {0.79};
  halfSitting["RWristYaw"] = {0.0};
  halfSitting["RHand"] = {0.0};
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
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RFsrFL", "l_ankle", sva::PTransformd(Eigen::Vector3d(0.07025, 0.0231, -0.04511))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RFsrRL", "l_ankle", sva::PTransformd(Eigen::Vector3d(-0.03025, 0.0191, -0.04511))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RFsrRR", "l_ankle", sva::PTransformd(Eigen::Vector3d(-0.02965, -0.0299, -0.04511))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RFsrFR", "l_ankle", sva::PTransformd(Eigen::Vector3d(0.07025, -0.0299, -0.04511))));

  _minimalSelfCollisions = {
      mc_rbdyn::Collision("Head", "l_wrist", 0.05, 0.02, 0.),
      mc_rbdyn::Collision("Head", "r_wrist", 0.05, 0.02, 0.),
      mc_rbdyn::Collision("Head", "LForeArm", 0.05, 0.02, 0.),
      mc_rbdyn::Collision("Head", "RForeArm", 0.05, 0.02, 0.),
      mc_rbdyn::Collision("xtion_link", "l_wrist", 0.05, 0.02, 0.),
      mc_rbdyn::Collision("xtion_link", "r_wrist", 0.05, 0.02, 0.),
      mc_rbdyn::Collision("xtion_link", "LForeArm", 0.05, 0.02, 0.),
      mc_rbdyn::Collision("xtion_link", "RForeArm", 0.05, 0.02, 0.),
      mc_rbdyn::Collision("LThigh", "l_wrist", 0.05, 0.02, 0.),
      mc_rbdyn::Collision("RThigh", "r_wrist", 0.05, 0.02, 0.),
      mc_rbdyn::Collision("l_wrist", "r_wrist", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("l_wrist", "torso", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("r_wrist", "torso", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("l_ankle", "r_ankle", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("l_ankle", "RTibia", 0.05, 0.01, 0.),
      mc_rbdyn::Collision("r_ankle", "LTibia", 0.05, 0.01, 0.)
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

  _ref_joint_order = {
      "HeadYaw", "HeadPitch", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand", "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"};

  // Posture of base link in half-sitting for when no attitude is available.
  // (quaternion, translation)
  _default_attitude = {{1., 0., 0., 0.,  0., 0., 0.333}};
  LOG_SUCCESS("NAOCommonRobotModule initialized");
}

std::map<std::string, std::pair<std::string, std::string>> NAOCommonRobotModule::getConvexHull(const std::map<std::string, std::pair<std::string, std::string>>& files) const
{
  std::string convexPath = path + "/convex/";
  std::map<std::string, std::pair<std::string, std::string>> res;
  for (const auto& f : files)
  {
    res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
    LOG_INFO("f: " << f.first << ", " << f.second.first << ", " << f.second.second);
    LOG_INFO("res: " << res[f.first].first << ", " << res[f.first].second);
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
    mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), false, filteredLinks, true, "base_link");
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
  for (const auto& j : mb.joints()) {
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
  // for (const auto& b : mb.bodies())
  // {
  //   // Filter out virtual links without convex files
  //   if (std::find(std::begin(virtualLinks), std::end(virtualLinks), b.name()) == std::end(virtualLinks))
  //   {
  //     res[b.name()] = {b.name(), boost::algorithm::replace_first_copy(b.name(), "_LINK", "")};
  //   }
  // }

  // Manually add all convex for bodies
  auto addBody = [&res](const std::string& body, const std::string& file) {
    res[body] = {body, file};
  };
  // Add correspondance between link and corresponding CH name
  addBody("Head", "HeadPitch");
  addBody("xtion_link", "ASUS_XTION");
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

  return res;
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
