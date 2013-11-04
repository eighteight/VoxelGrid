//
//  Voxelifier.h
//  voxelify
//
//  Created by Gusev, Vladimir on 10/22/13.
//
//

#ifndef __voxelify__Voxelifier__
#define __voxelify__Voxelifier__
#include "VGrid.h"

class Voxelifier{
public:
    Voxelifier(){};
    ~Voxelifier(){};
    
    VGrid voxelify(const std::vector<std::vector<float> >& points, const float xLeaf, const float yLeaf, const float zLeaf);


};
#endif /* defined(__voxelify__Voxelifier__) */
