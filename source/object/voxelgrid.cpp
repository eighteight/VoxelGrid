
#include "c4d.h"
#include "c4d_symbols.h"
#include "c4d_tools.h"
#include "lib_splinehelp.h"
#include "ge_dynamicarray.h"
#include "voxelify.h"
#include <vector>
#include <string>
#include <iostream>
#include "Voxelifyer.h"
#include "VGrid.h"
// unique ID obtained from www.plugincafe.com
#define ID_VOXELGRID 1031351

typedef std::pair<SplineObject*,Real> SplinePair;
bool comparator ( const SplinePair& l, const SplinePair& r){
    return l.second > r.second;
}

class VoxelGrid : public ObjectData
{
private:
    Real maxSeg, minSeg;
    Matrix parentMatrix;
    void DoRecursion(BaseObject *op, BaseObject *child, GeDynamicArray<Vector> &points, Matrix ml);
    vector<vector<float> > objectPointsToPoints(GeDynamicArray<Vector>  objectPoints);
    Voxelifier vox;
    
public:
    BaseObject* GetVirtualObjects(BaseObject *op, HierarchyHelp *hh);
    virtual Bool Init(GeListNode *node);
    static NodeData *Alloc(void) { return gNew VoxelGrid; }
};

Bool VoxelGrid::Init(GeListNode *node)
{
	BaseObject		*op   = (BaseObject*)node;
	BaseContainer *data = op->GetDataInstance();

    data->SetLong(SPLINEOBJECT_INTERPOLATION,SPLINEOBJECT_INTERPOLATION_ADAPTIVE);
    GePrint("VoxelGrid by http://twitter.com/eight_io for Cinema 4D r14");
    
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
    
    BaseDocument *doc = op->GetDocument();
    BaseObject* obj = (BaseObject*)data->GetLink(CTT_OBJECT_LINK,doc,Obase);
    if (!obj) return NULL;
    LONG startObject = data->GetLong(START_FRAME);
    LONG endObject = data->GetLong(END_FRAME);
    
    if (startObject >=endObject) return NULL;
    
    maxSeg = data->GetReal(CTTSPOBJECT_MAXSEG,30.);
    minSeg = data->GetReal(CTTSPOBJECT_MINSEG);
    
    LONG delta = data->GetLong(OBJECT_SKIP,1);
    delta = delta < 1 ? 1 : delta;
    
    GeDynamicArray<BaseObject*> children;
    GeDynamicArray<GeDynamicArray<Vector> > splineAtPoint;
    
    BaseObject* chld = NULL;
    LONG trck = 0;
    for (chld=obj->GetDownLast(); chld; chld=chld->GetPred()) {
        if (trck >= startObject && trck<= endObject && trck % delta == 0){
            children.Push((BaseObject*)chld->GetClone(COPYFLAGS_NO_HIERARCHY|COPYFLAGS_NO_ANIMATION|COPYFLAGS_NO_BITS,NULL));
        }
        trck++;
    }
    
    if (children.GetCount() < 1) {
        return NULL;
    }
    
    LONG longestPercent = data->GetLong(TAKE_LONGEST, 1);
    longestPercent = longestPercent > 100 ? 100: longestPercent;
    
    GeDynamicArray<GeDynamicArray<Vector> > objectPoints(children.GetCount());
	StatusSetBar(0);
    StatusSetText("Collecting Points");
    vector<vector<float> > points;
    std::vector<VGrid> grids;
    
    BaseObject* ret = BaseObject::Alloc(Onull);

    parentMatrix = op->GetMl();
    for (int k= 0; k < 1; k++){
        Vector bb = children[k]->GetRad();
        Matrix ml;
        DoRecursion(op,children[k],objectPoints[k], ml);
        points = objectPointsToPoints(objectPoints[k]);
        GePrint(children[k]->GetName());
        VGrid grid = vox.voxelify(points,bb.x/(float)gridSize,bb.y/(float)gridSize,bb.z/(float)gridSize);
        
        for (int i = 0; i < grid.points.size(); i++){
            if (grid.indices[i] == -1) continue;
            Vector pos(grid.points[i][0],grid.points[i][1],grid.points[i][2]);
            BaseObject* cube = BaseObject::Alloc(Ocube);
            cube->SetRelPos(pos*parentMatrix);
            cube->InsertUnder(ret);
        }
    }
    

    
    for (int k=0; k<children.GetCount(); k++){
        if (children[k]){
            BaseObject::Free(children[k]);
        }
    }
    
    return ret;
}


Bool Registervoxelify(void)
{
	return RegisterObjectPlugin(ID_VOXELGRID ,GeLoadString(IDS_VOXELIFY),OBJECT_GENERATOR|OBJECT_INPUT|OBJECT_ISSPLINE|OBJECT_CALL_ADDEXECUTION,VoxelGrid::Alloc,"voxelify",AutoBitmap("tsp.tif"),0);
}
