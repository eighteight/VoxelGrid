
#include "c4d.h"
#include "c4d_symbols.h"
#include "c4d_tools.h"
#include "ge_dynamicarray.h"
#include "voxelgrid.h"
#include <vector>
#include <string>
#include <iostream>
#include "Voxelifyer.h"
#include <boost/make_shared.hpp>
#include "VGrid.h"
// unique ID obtained from www.plugincafe.com
#define ID_VOXELGRID 1031351

class VoxelGrid : public ObjectData
{
private:

    Matrix parentMatrix;
    void DoRecursion(BaseObject *op, BaseObject *child, GeDynamicArray<Vector> &points, Matrix ml);
    vector<vector<float> > objectPointsToPoints(GeDynamicArray<Vector>  objectPoints);
    
public:
    BaseObject* GetVirtualObjects(BaseObject *op, HierarchyHelp *hh);
    virtual Bool Init(GeListNode *node);
    static NodeData *Alloc(void) { return gNew VoxelGrid; }
    Bool Message(GeListNode *node, LONG type, void *data);
    ~VoxelGrid();

};

Bool VoxelGrid::Init(GeListNode *node)
{

    GePrint("VoxelGrid by http://twitter.com/eight_io for Cinema 4D r14");
    
    return TRUE;
}

VoxelGrid::~VoxelGrid(){
    
}

Bool VoxelGrid::Message(GeListNode *node, LONG type, void *data)
{
	return TRUE;
}

void VoxelGrid::DoRecursion(BaseObject *op, BaseObject *child, GeDynamicArray<Vector> &points, Matrix ml) {
	BaseObject *tp;
	if (child){
		tp = child->GetDeformCache();
		ml = ml * child->GetMl();
		if (tp){
			DoRecursion(op,tp,points,ml);
		}
		else{
			tp = child->GetCache(NULL);
			if (tp){
				DoRecursion(op,tp,points,ml);
			}
			else{
				if (!child->GetBit(BIT_CONTROLOBJECT)){
					if (child->IsInstanceOf(Opoint)){
						PointObject * pChild = ToPoint(child);
						LONG pcnt = pChild->GetPointCount();
						const Vector *childVerts = pChild->GetPointR();
						for(LONG i=0; i < pcnt; i++){
							points.Push(childVerts[i] * ml * parentMatrix);
						}
					}
				}
			}
		}
		for (tp = child->GetDown(); tp; tp=tp->GetNext()){
			DoRecursion(op,tp,points,ml);
		}
	}
}

vector<vector<float> > VoxelGrid::objectPointsToPoints(GeDynamicArray<Vector>  objectPoints){
    vector<vector<float> > points(objectPoints.GetCount());
    for (LONG i = 0; i < objectPoints.GetCount(); i++) {
        vector<float> p(3);
        p[0] = objectPoints[i].x;
        p[1] = objectPoints[i].y;
        p[2] = objectPoints[i].z;
        points[i] = p;
    }
    return points;
}


BaseObject *VoxelGrid::GetVirtualObjects(BaseObject *op, HierarchyHelp *hh)
{
    BaseContainer *data = op->GetDataInstance();
    
    LONG gridSize = data->GetLong(GRID_SIZE, 1);
    BaseDocument* doc = (BaseDocument*)op->GetDocument();
    LONG crntFrame = doc->GetTime().GetFrame(doc->GetFps());
    LONG trck = 0;
    BaseObject* chld = NULL;

    BaseObject* clone = NULL;
    for (chld=op->GetDownLast(); chld; chld=chld->GetPred()) {
        if (trck == crntFrame){
            clone = (BaseObject*)chld->GetClone(COPYFLAGS_NO_HIERARCHY|COPYFLAGS_NO_ANIMATION|COPYFLAGS_NO_BITS,NULL);
        }
        trck++;
    }
    
    if (!clone) return NULL;

    GeDynamicArray<Vector> objectPoints;
	StatusSetBar(0);
    StatusSetText("Collecting Points");
    vector<vector<float> > points;
    std::vector<VGrid> grids;
    
    BaseObject* ret = BaseObject::Alloc(Onull);
    
    parentMatrix = op->GetMl();
    
    Vector bb = clone->GetRad();
    Vector gridStep(bb.x/gridSize, bb.y/gridSize, bb.z/gridSize);
    Matrix ml;
    GePrint(clone->GetTypeName());

    DoRecursion(op,ToPoint(clone),objectPoints, ml);
    if (objectPoints.GetCount() == 0) return NULL;
    points = objectPointsToPoints(objectPoints);
    
    LONG thre = data->GetReal(THRESHOLD, 1.0);
    Real multi = data->GetReal(MULTIPLIER, 1.0);
    vector<float> centro(3);
    boost::shared_ptr<Voxelifier> vox ( new Voxelifier());
    VGrid grid = vox->voxelify(points,gridSize, centro, thre, multi );
    StatusSetText("Voxel gridding");
    for (int i = 0; i < grid.points.size(); i++){
        if (grid.indices[i] == -1) continue;
        Vector pos(grid.points[i][0],grid.points[i][1],grid.points[i][2]);
        BaseObject* cube = BaseObject::Alloc(Ocube);
        cube->SetRelPos(pos);
        cube->SetRelScale(Vector(1.0/gridSize, 1.0/gridSize, 1.0/gridSize));
        cube->InsertUnder(ret);
    }
    StatusClear();
    vox.reset();
    return ret;
}


Bool Registervoxelify(void)
{
	return RegisterObjectPlugin(ID_VOXELGRID ,GeLoadString(IDS_VOXELIFY),OBJECT_GENERATOR|OBJECT_INPUT,VoxelGrid::Alloc,"voxelgrid",AutoBitmap("tsp.tif"),0);
}
