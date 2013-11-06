CONTAINER voxelify
{
	NAME voxelify;
	INCLUDE Obase;

	GROUP ID_OBJECTPROPERTIES
	{
        LONG GRID_SIZE {MIN 1; MAX 10000; CUSTOMGUI LONGSLIDER;}
		REAL RADIUS { UNIT METER;	MIN 1.0; CUSTOMGUI REALSLIDER; STEP 1.0; MINSLIDER 1.; MAXSLIDER 100.0;}
	}
}
