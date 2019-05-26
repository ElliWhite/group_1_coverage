/*
 * LRTAstar.h
 *
 *  Created on: May 9, 2014
 *      Author: francescow
 */

#ifndef LRTASTAR_H_
#define LRTASTAR_H_

#include <vector>
#include <set>
#include <fstream>
#include "graphStructs.h"

using std::vector;


class LRTAstar
{
  int STARTNODE;
  int gridSizeX;
  int gridSizeY;
  //int currentNode;
  //int nextNode;
  //vector<graphNode> graphNodes;
  vector< vector <int> > graphAdjMat; // adjacency matrix
  vector <int> AStarCount;
  vector<int> access_vec;  // access_vec[i] = 1 if occupied, 0 otherwise; It could be "bool" but I left "int" for future map developments
  vector<int> unvisited;
  int unvisitedCount;
  int minVisit;

  int numFreeNodes;
  vector<int> finalPath;

  void createEdgeMat();
  void loadMatrixFile(std::ifstream &access_mat);
  void createGraph(std::ifstream & INFILE);
  void loadGraphFile(std::ifstream &graph_mat);
  void loadPosVecFile(std::ifstream &Pos_vec);

public:
  vector<graphNode> graphNodes;
  int currentNode;
  int nextNode;
  
public: 
    
  LRTAstar();
  LRTAstar(std::ifstream & INFILE);
  virtual ~LRTAstar(); 
  void init_acc(std::ifstream & graph_mat, int startingNode, int minVis);
  void init_graph_pos(std::ifstream &graph_mat, std::ifstream &Pos_vec, int startingNode, int minVis);

  void incrCount(int currIndex, int nextIndex, bool nextType, bool printDebug = false);
  bool findNext();
  // find the next node by considering the input engaged nodes as obstacles 
  bool findNext(const std::set<int>& setEngagedNodes);
  
  float getCurrentCoord(char coordinate);
  float getNextCoord(char coordinate);
  float getCoord(char coordinate, int node);
  
  // compute the set of neighbours of a given input node 
  void getNeighboursSet(const int node, std::set<int>& setNeighbours);
    
  bool getCurrType();
  bool getNextType();
  bool isCompleted(); 

  int getCurrentIndex() const
  {
    return currentNode;
  }

  const vector<int>& getFinalPath() const
  {
    return finalPath;
  }

  int getNumFreeNodes() const
  {
    return numFreeNodes;
  }

  int getNextIndex() const
  {
    return nextNode;
  }
  
  // update current node (this is useful when we arrive in the node we planned to reach)
  void updateCurrNode()
  {
      currentNode = nextNode;
  }
  
  void printCoverage(); 
    
};

#endif /* LRTASTAR_H_ */
