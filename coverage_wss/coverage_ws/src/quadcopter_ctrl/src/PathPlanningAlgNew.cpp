//	Copyright (c) 2014, Francesco Wanderlingh. 			        //
//	All rights reserved.						                //
//	License: BSD (http://opensource.org/licenses/BSD-3-Clause)  //

/*
 * PathPlanningAlg.cpp
 *
 *  Created on: Apr 7, 2014
 *      Author: francescow
 */


#include "geometry_msgs/PoseStamped.h"
#include "PathPlanningAlgNew.h"
#include "quadcopterRosCtrl.h"

PathPlanningAlgNew::PathPlanningAlgNew()
{
  gridWidth = 0;
  gridHeight = 0;

}

PathPlanningAlgNew::~PathPlanningAlgNew()
{
  // TODO Auto-generated destructor stub
}



void PathPlanningAlgNew::InterpNewPoint(geometry_msgs::PoseStamped* quadPos,
                                          geometry_msgs::PoseStamped* targetPos,
                                          double dSubWP[3])
{
  /** interpNewPoint() interpolates the next point in the direction of the target
   * with a distance from the quadcopter that is less than CRITICAL_DIST
   * As a first step we compute the vector d = (quadPos -> targetPos),
   * then we compute its norm to get the unit vector and then we will
   * multiply the unit vector for a given proportionality constant to
   * get the desired dSubWP movement.
   */

   double distVec[3] = { (targetPos->pose.position.x - quadPos->pose.position.x),
                        (targetPos->pose.position.y - quadPos->pose.position.y),
                        (targetPos->pose.position.z - quadPos->pose.position.z) };
   double distVecNorm = Distance(quadPos, targetPos);
   //std::cout << "PathPlanningAlg Function -> Distance = " <<  distVecNorm << " m" << std::endl;
   dSubWP[X] = (distVec[X]/distVecNorm)*dSubWP[X];
   dSubWP[Y] = (distVec[Y]/distVecNorm)*dSubWP[Y];
   dSubWP[Z] = (distVec[Z]/distVecNorm)*dSubWP[Z];

}

double PathPlanningAlgNew::Distance(geometry_msgs::PoseStamped* quadPos,
                                   geometry_msgs::PoseStamped* targetPos){

  double dist = sqrt (pow(quadPos->pose.position.x - targetPos->pose.position.x, 2.0)
                      + pow(quadPos->pose.position.y - targetPos->pose.position.y, 2.0)
                      + pow(quadPos->pose.position.z - targetPos->pose.position.z, 2.0)
                      );
  return dist;
}


int PathPlanningAlgNew::LoadParams(std::string controlMode,
                        double &scale, double &ofs_x, double &ofs_y,
                        double &wpStep, double &critDist, double &threshold){
 
  if(controlMode == "sim"){
    scale = 0.5; 
    wpStep = 0.1;
    //ofs_x = 10.7;
    //ofs_y = 9.43;
    ofs_x = -0.5*scale;
    ofs_y = -0.5*scale;
    critDist = 0.65;
    threshold = 0.3;

    return 0;
  }else if(controlMode == "asctec"){
    scale = 5.0;
    wpStep = 5.0;
    ofs_x = ofs_y = 0;
    critDist = 100;
    threshold = 1.0;
    return 0;  
  }else return -1;
}

