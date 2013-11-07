CONTAINER voxelify
{
	NAME voxelify;
	INCLUDE Obase;

	GROUP ID_OBJECTPROPERTIES
	{
        LONG GRID_SIZE {MIN 1; MAX 10000; CUSTOMGUI LONGSLIDER;}
        LONG THRESHOLD {MIN 1; MAX 10000; CUSTOMGUI LONGSLIDER;}
        REAL MULTIPLIER {MIN 1.0; CUSTOMGUI REALSLIDER; STEP 0.1; MINSLIDER 1.; MAXSLIDER 10.0;}
	}
}
