#ifndef GRASPGENERATOR_H
#define GRASPGENERATOR_H

#include <map>
#include <QObject>
#include <QtGui>

//GraspIt! includes
#include <include/plugin.h>

class SearchEnergy;
class ListPlanner;
class GraspPlanningState;
class GraspableBody;
class Hand;
class vec3;

//! Main class, combining a ROS node with a GraspIt! interface
/*! Note that this class inherits from GraspIt's Plugin class and implements the necessary functions to serve
  as a GraspIt plugin. See include/plugin.h in the GraspIt code for the base class.
  Provides a number of ROS services that directly operate on the GraspIt world, such as loading objects or
  obstacles, simulating 3D scans of objects, etc.
  In particular, note that this class uses the mainLoop() function to perform the ROS event management calls.
*/
class GraspGenerationPlugin : public QObject, public Plugin
{

    Q_OBJECT

public:
  //! Inits ROS, but (for now) without passing any arguments
  GraspGenerationPlugin();
  //! Deletes the node handle and the db manager
  ~GraspGenerationPlugin();
  //! Creates the node handles, advertises services, connects to the database
  virtual int init(int argc, char **argv);
  //! Simply calls ros::spinOnce() to process the ROS event loop
  virtual int mainLoop();

protected:
  void startPlanner();
  void uploadResults();
  void stepPlanner();

  void sampleFace(vec3 x, vec3 y, vec3 z,
                                  double sz1, double sz2, vec3 tln, double res);

  void boxSampling(double a, double b, double c, double res);

private:

  SearchEnergy *mEnergyCalculator;
  GraspableBody *mObject;
  Hand *mHand;

  bool plannerStarted;
  bool plannerFinished;
  bool evaluatingGrasps;

  QString mesh_filepath;
  QString result_filepath;

  bool render_it;

  unsigned int current_step;

  std::vector<GraspPlanningState*> pregrasps;

};


#endif // GRASPGENERATOR_H
