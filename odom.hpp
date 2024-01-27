#ifndef _ODOM_
#define _ODOM_

#define degperinch 28.6482787
#define degperdegrot 2.752189

//odom task
void odomTracker(void* ignore);

void odomSetCoords(double x, double y, double bearing);

extern double odomGlobalX, odomGlobalY, globalDeltaY, globalDeltaX, localX, localY, odomRadius, odomPrevPosLeft, odomPrevPosRight, odomDeltaPosLeft, odomDeltaPosRight, odomPrevAngle, odomDeltaAngle;
#endif