#include "MyImprovedEST.h"

#include "MPLibrary/MPLibrary.h"

#include <iostream>
#include <string>
#include <utility>
#include <vector>


/*----------------------------- Construction ---------------------*/

MyImprovedEST::
MyImprovedEST() {
  this->SetName("MyImprovedEST");
}


MyImprovedEST::
MyImprovedEST(XMLNode& _node)
  : MPStrategyMethod(_node) {
    this->SetName("MyImprovedEST");
    m_vcLabel = _node.Read("vcLabel", false, "pqp_solid", "Validity Checker");
    m_dmLabel = _node.Read("dmLabel", false, "euclidean", "Distance Metric");
    m_lpLabel = _node.Read("lpLabel", false, "sl", "Local Planner");
  }


/*--------------------------- MPBaseObject Overrides ------------------*/

void
MyImprovedEST::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

/*------------------------ MPStrategyMethod Overrides ---------------------*/

void
MyImprovedEST::
Initialize() {
  // Generate the start and goal configuration
  const VID start = this->GenerateStart("UniformRandomFree");
  VID goal = this->GenerateGoals("UniformRandomFree").front();

  // Add the start and goal to the respective trees
  m_startTree.push_back(start);
  m_goalTree.push_back(goal);
}


void
MyImprovedEST::
Run() {
  do {
    GrowTree(m_startTree, true);
    GrowTree(m_goalTree, false);
    TryMerge();
  } while(!this->EvaluateMap());
}


/*------------------------- Add helper functions -----------------*/
void
MyImprovedEST::
GrowTree(vector<VID>& _tree, bool isStart) {

  // access pointers
  auto r = this->GetRoadmap();
  auto env = this->GetEnvironment();
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  auto lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);
  auto robot = this->GetTask()->GetRobot();
  LPOutput lpOutput;
  Cfg collisionCfg(robot);

  // choose a random VID in the tree and get its corresponding Cfg q
  auto randomIndex = rand() % _tree.size();
  auto randomVID = _tree[randomIndex];
  Cfg q = r->GetVertex(randomVID);

  // choose a random direction dir and a random distance d between 0 and
  // some maximum distance maxDist
  double maxDist = 3.0;
  Cfg dir(robot);
  dir.GetRandomRay(maxDist, dm);

  // set the likelihood by which the random direction is scaled by the
  // direction vector to the start/goal configuration in the following stage
  double bias = 0.2;

  if ((double)rand()/RAND_MAX < bias){
    if (isStart == true){
      //if start tree: scale random dir to goal config
      Cfg goalVector = r->GetVertex(m_goalTree[0])-q;
      dir = goalVector;
      dm->ScaleCfg(maxDist, dir);
    }
    else{
      //if goal tree: scale random dir to start config
      Cfg startVector = r->GetVertex(m_startTree[0])-q;
      dir = startVector;
      dm->ScaleCfg(maxDist, dir);
    }
  }

  // create new node by moving in dir's direction
  Cfg n = q + dir;

  // Try to connect n to q using a local planner
  if(lp->IsConnected(q, n, collisionCfg, &lpOutput, env->GetPositionRes(), env->GetOrientationRes())) {
    // add the new configuration to the roadmap and store its ID to the tree 
    VID nID = r->AddVertex(n);
    _tree.push_back(nID);
    // c. Add to the roadmap the edge connecting q to n
    r->AddEdge(randomVID, nID, lpOutput.m_edge);
  }
}

void
MyImprovedEST:: 
TryMerge() {
  // access pointers
  auto r = this->GetRoadmap();
  auto env = this->GetEnvironment();
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  auto lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);
  auto robot = this->GetTask()->GetRobot();
  LPOutput lpOutput;
  Cfg collisionCfg(robot);

  // choose a random set of VIDs from each tree and store them in local
  // variables startSet and goalSet
  size_t k = 5;
  vector<VID> startSet;
  vector<VID> goalSet;
  for(size_t i = 0; i < k; i++) {
    size_t randStartIndex = rand() % m_startTree.size();
    size_t randGoalIndex = rand() % m_goalTree.size();
    startSet.push_back(m_startTree[randStartIndex]);
    goalSet.push_back(m_goalTree[randGoalIndex]);
  }

  // for each VID in startSet, get the corresponding Cfg and try to connect 
  // it to each Cfg corresponnding to the VIDs in goalSet using a local planner
  for(auto startVID : startSet) {
    Cfg startCfg = r->GetVertex(startVID);
    for(auto goalVID : goalSet) {
      Cfg goalCfg = r->GetVertex(goalVID);
      if(lp->IsConnected(startCfg, goalCfg, collisionCfg, &lpOutput, env->GetPositionRes(), env->GetOrientationRes())) {
        r->AddEdge(startVID, goalVID, lpOutput.m_edge);
        break;
      }
    }
  }
}