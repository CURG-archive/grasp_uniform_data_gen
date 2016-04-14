#include "graspGenerationPlugin.h"

#include <boost/foreach.hpp>
#include <cmath>
#include <fstream>

#include <include/world.h>
#include <include/body.h>
#include <include/robot.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>

#include <include/EGPlanner/egPlanner.h>
#include <include/EGPlanner/simAnnPlanner.h>
#include <include/EGPlanner/searchState.h>
#include <include/EGPlanner/searchEnergy.h>
#include <include/EGPlanner/listPlanner.h>
#include <include/matvec3D.h>
#include <include/grasp.h>
#include <include/triangle.h>

#include <cmdline/cmdline.h>

#include <Inventor/actions/SoGetBoundingBoxAction.h>


GraspGenerationPlugin::GraspGenerationPlugin() :
    plannerStarted(false),
    plannerFinished(false),
    evaluatingGrasps(false),
    current_step(0)
{
}

GraspGenerationPlugin::~GraspGenerationPlugin()
{
}


int GraspGenerationPlugin::init(int argc, char **argv)
{
    std::cout << "Starting GraspGenerationPlugin: " << std::endl ;
    cmdline::parser *parser = new cmdline::parser();

    parser->add<std::string>("mesh_filepath", 'c', "mesh_filepath",  false);
    parser->add<std::string>("result_filepath", 'z', "result_filepath", false);
    parser->add<bool>("render", 'l', "render", false);

    parser->parse(argc, argv);

    if (parser->exist("render"))
    {
        render_it = parser->get<bool>("render");

    }
    else
    {
        render_it = false;
    }

    mesh_filepath = QString::fromStdString(parser->get<std::string>("mesh_filepath"));
    result_filepath = QString::fromStdString(parser->get<std::string>("result_filepath"));

    std::cout << "render: " << render_it << "\n" ;
    std::cout << "mesh_filepath: " << mesh_filepath.toStdString().c_str() << "\n" ;

  return 0;
}

//This loop is called over and over again. We do 3 different things
// 1) First step: start the planner
// 2) Middle steps: step the planner
// 3) Last step, save the grasps
int GraspGenerationPlugin::mainLoop()
{
    //save grasps
    if(plannerStarted && plannerFinished && (!evaluatingGrasps))
    {
        uploadResults();
    }
    //start planner
    else if (!plannerStarted)
    {
        startPlanner();
    }
    //let planner run.
    else if( (plannerStarted) && !plannerFinished )
    {
        stepPlanner();
    }

  return 0;
}

void GraspGenerationPlugin::startPlanner()
{
    std::cout << "Starting Planner\n" <<std::endl;

    //TODO
    //here we need to get the hand and object from the cloud. rather than locally
    graspItGUI->getMainWorld()->importBody("GraspableBody", mesh_filepath);
    graspItGUI->getMainWorld()->importRobot("/home/timchunght/graspit/models/robots/pr2_gripper_2010/pr2_gripper_2010.xml");

    mObject = graspItGUI->getMainWorld()->getGB(0);
    mObject->setMaterial(5);//rubber

    mHand = graspItGUI->getMainWorld()->getCurrentHand();
    mHand->getGrasp()->setObjectNoUpdate(mObject);
    mHand->getGrasp()->setGravity(false);

    graspItGUI->getMainWorld()->toggleAllCollisions(false);
    mHand->autoGrasp(true, -1.0, false);
    graspItGUI->getMainWorld()->toggleAllCollisions(true);

//    mHandObjectState = new GraspPlanningState(mHand);
//    mHandObjectState->setObject(mObject);
//    mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
//    mHandObjectState->setRefTran(mObject->getTran());
//    mHandObjectState->reset();

    mEnergyCalculator = new SearchEnergy();
    mEnergyCalculator->setType(ENERGY_CONTACT_QUALITY);
    mEnergyCalculator->setContactType(CONTACT_PRESET);

    //get object bbox dimensions
    SoGetBoundingBoxAction *bba =
        new SoGetBoundingBoxAction(graspItGUI->getIVmgr()->getViewer()->getViewportRegion());
    bba->apply(mObject->getIVGeomRoot());
    SbVec3f bbmin,bbmax;
    bba->getBoundingBox().getBounds(bbmin,bbmax);
    delete bba;
    double scale = 0.5;
    double a = scale*(bbmax[0] - bbmin[0]);
    double b = scale*(bbmax[1] - bbmin[1]);
    double c = scale*(bbmax[2] - bbmin[2]);
    int resolution = 30;
    //this fills the pregrasps
    graspItGUI->getMainWorld()->toggleAllCollisions(false);
    boxSampling(a, b, c, resolution);
    graspItGUI->getMainWorld()->toggleAllCollisions(true);

    std::cout << "Found " << pregrasps.size() << " Pre grasp locations." << std::endl;

    plannerStarted = true;
}

void GraspGenerationPlugin::stepPlanner()
{
    std::cout << "Stepping Planner" <<std::endl;
    std::cout << "current_step: " << current_step <<std::endl;
    std::cout << "pregrasps.size(): " << pregrasps.size() <<std::endl;
    if (current_step >= pregrasps.size())
    {
        plannerFinished=true;
    }
    else{
        double sleep_time = 10;

        GraspPlanningState *gps = pregrasps.at(current_step);

        graspItGUI->getMainWorld()->toggleAllCollisions(false);
        std::cout << "About to Execute" <<std::endl;
        gps->execute(mHand);
        std::cout << "Executed" <<std::endl;
        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(sleep_time);

        mHand->approachToContact(-300);
        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(sleep_time);

        mHand->autoGrasp(true,-1, false);
        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(sleep_time);

        graspItGUI->getMainWorld()->toggleAllCollisions(true);

        mHand->approachToContact(350);
        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(sleep_time);

        mHand->autoGrasp(true, 1.0, false);
        bool is_legal;
        double new_planned_energy;
        std::cout << "Getting Energy: " << std::endl;
        mEnergyCalculator->analyzeCurrentPosture(mHand, mObject, is_legal, new_planned_energy, false );
        std::cout << "Energy: " << new_planned_energy << std::endl;

        gps->setEnergy(new_planned_energy);
        gps->saveCurrentHandState();

        std::cout << "Energy: " << gps->getEnergy() << std::endl;

        graspItGUI->getIVmgr()->getViewer()->render();
        usleep(sleep_time);
        current_step++;
    }
}

void GraspGenerationPlugin::uploadResults()
{

     std::cout << "Uploading Results " << std::endl;

    int num_grasps = pregrasps.size();
    std::cout << "Found " << num_grasps << " Grasps. " << std::endl;


    for(int i=0; i < num_grasps; i++)
    {
        std::cout << "i: " << i << std::endl;

        GraspPlanningState *gps = pregrasps.at(i);

        if (gps->getEnergy() >=0)
        {
            continue;
        }
        gps->execute(mHand);

        usleep(100000);

        double dofVals [mHand->getNumDOF()];
        mHand->getDOFVals(dofVals);

        transf hand_pose = mHand->getPalm()->getTran();

        //TODO
        //here we need to save all this to the database:
        std::cout << "Object: " << mesh_filepath.toStdString().c_str() << std::endl;
        std::cout << "Hand: " << mHand->getFilename().toStdString().c_str() << std::endl;
        std::cout << "Energy: " << gps->getEnergy() << std::endl;
        std::cout << "Pose: ";
        std::cout << hand_pose.translation().x() << "; ";
        std::cout << hand_pose.translation().y() << "; ";
        std::cout << hand_pose.translation().z() << "; ";
        std::cout << hand_pose.rotation().w << "; ";
        std::cout << hand_pose.rotation().x << "; ";
        std::cout << hand_pose.rotation().y << "; ";
        std::cout << hand_pose.rotation().z << std::endl;
        std::cout << "Dof: ";
        for(int dof_idx = 0; dof_idx < mHand->getNumDOF(); dof_idx ++)
        {
            std::cout << dofVals[dof_idx] << "; ";
        }
        std::cout << std::endl;

    }

    assert(false);
}

void GraspGenerationPlugin::boxSampling(double a, double b, double c, double res)
{
    double scale = 1;
    sampleFace( vec3(0, 1,0), vec3(-1,0,0), vec3(0,0,1) , a, c, vec3(0,-b*scale,0), res);
    sampleFace( vec3(0,-1,0), vec3( 1,0,0), vec3(0,0,1) , a, c, vec3(0, b*scale,0), res);

    sampleFace( vec3(0,0, 1), vec3(0,1,0), vec3(-1,0,0) , b, a, vec3(0,0,-c*scale), res);
    sampleFace( vec3(0,0,-1), vec3(0,1,0), vec3( 1,0,0) , b, a, vec3(0,0, c*scale), res);

    sampleFace( vec3( 1,0,0), vec3(0, 1,0), vec3(0,0,1) , b, c, vec3(-a*scale,0,0), res);
    sampleFace( vec3(-1,0,0), vec3(0,-1,0), vec3(0,0,1) , b, c, vec3( a*scale,0,0), res);
}

void GraspGenerationPlugin::sampleFace(vec3 x, vec3 y, vec3 z,
                                double sz1, double sz2, vec3 tln, double res)
{
    mat3 R(x, y, z);
    int rotSamples=2;

    double m1 = (2.0*sz1 - floor(2.0*sz1 / res) * res)/2.0;
    while (m1 < 2*sz1){
        double m2 = (2.0*sz2 - floor(2.0*sz2 / res) * res)/2.0;
        while (m2 < 2*sz2) {
            vec3 myTln(tln);
            myTln = myTln + (m1 - sz1)* y;
            myTln = myTln + (m2 - sz2)* z;
            transf tr(R, myTln);
            for(int rot=0; rot < rotSamples; rot++) {
                double angle = M_PI * ((double)rot) / rotSamples;
                transf rotTran(Quaternion(angle, vec3(1,0,0)), vec3(0,0,0));
                tr = rotTran * tr;
                GraspPlanningState* seed = new GraspPlanningState(mHand);
                seed->setObject(mObject);
                seed->setRefTran(mObject->getTran(), false);
                seed->setPostureType(POSE_DOF, false);
                seed->setPositionType(SPACE_COMPLETE, false);
                seed->reset();
                seed->getPosition()->setTran(tr);
                pregrasps.push_back(seed);
            }
            m2+=res;
        }
        m1 += res;
    }
}



