#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn_urdf/urdf.h>

namespace mc_nao
{

  struct ROBOT_MODULE_API NAOCommonRobotModule : public mc_rbdyn::RobotModule
  {
  public:
    NAOCommonRobotModule();
  protected:
    std::map<std::string, std::pair<std::string, std::string> > getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const;

    void readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks);

    std::map<std::string, std::vector<double>> halfSittingPose(const rbd::MultiBody & mb) const;

    std::vector< std::map<std::string, std::vector<double> > > nominalBounds(const mc_rbdyn_urdf::Limits & limits) const;

    std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(const rbd::MultiBody & mb) const;

  public:
    std::vector<std::string> virtualLinks;
    std::vector<std::string> gripperLinks;
    std::map< std::string, std::vector<double> > halfSitting;
    mc_rbdyn_urdf::Limits limits;
  };

  struct ROBOT_MODULE_API NAONoHandRobotModule : public NAOCommonRobotModule
  {
  public:
    NAONoHandRobotModule();

    virtual const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const;

    virtual const std::vector< std::map<std::string, std::vector<double> > > & bounds() const;

    virtual const std::map<std::string, std::vector<double> > & stance() const;
  public:
    std::vector<std::string> filteredLinks;
  };

  struct ROBOT_MODULE_API NAOWithHandRobotModule : public NAOCommonRobotModule
  {
  public:
    NAOWithHandRobotModule();

    virtual const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const;

    virtual const std::vector< std::map<std::string, std::vector<double> > > & bounds() const;

    virtual const std::map<std::string, std::vector<double> > & stance() const;
  public:
    std::vector<std::string> filteredLinks;
  };

}

/*TODO Provide a different constructor to allow WithHand/NoHand instantation */
ROBOT_MODULE_DEFAULT_CONSTRUCTOR("NAO", mc_nao::NAOWithHandRobotModule)
