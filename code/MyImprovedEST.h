#ifndef MY_IMPROVEDEST_H_
#define MY_IMPROVEDEST_H_

#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPStrategyMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Utilities/XMLNode.h"

//////////////////////////////////////////////////////////////////
/// MyImprovedEST algorithm
///
/// @ingroup MotionPlanningStrategies
/// /////////////////////////////////////////////////////////////

class MyImprovedEST : public MPStrategyMethod {

  public:

    typedef typename MPBaseObject::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID           VID;

    MyImprovedEST();
    MyImprovedEST(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    virtual void Initialize();
    virtual void Run();
    

  // Add code here to declare protected helper functions and private global variables
  

  protected:
    void GrowTree(vector<VID>& _tree, bool isStart);
    void TryMerge();

  private:
    string m_vcLabel;
    string m_dmLabel;
    string m_lpLabel;
  
    //VID's of trees at start and goal configuration
    vector<VID> m_startTree;
    vector<VID> m_goalTree;

};

#endif