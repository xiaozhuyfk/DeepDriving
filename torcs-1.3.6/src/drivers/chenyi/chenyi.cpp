/***************************************************************************

    file                 : chenyi.cpp
    created              : 2014年 05月 20日 星期二 20:15:18 EDT
    copyright            : (C) 2002 Chenyi Chen

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 

/* 
 * Module entry point  
 */ 
extern "C" int 
chenyi(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("chenyi");		/* name of the module (short) */
    modInfo->desc    = strdup("");	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
} 


/* Compute gear. */
const float SHIFT = 0.85;         /* [-] (% of rpmredline) */
const float SHIFT_MARGIN = 4.0;  /* [m/s] */

int getGear(tCarElt *car)
{
	if (car->_gear <= 0)
		return 1;
	float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
	float omega = car->_enginerpmRedLine/gr_up;
	float wr = car->_wheelRadius(2);

	if (omega*wr*SHIFT < car->_speed_x) {
		return car->_gear + 1;
	} else {
		float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
		omega = car->_enginerpmRedLine/gr_down;
		if (car->_gear > 1 && omega*wr*SHIFT > car->_speed_x + SHIFT_MARGIN) {
			return car->_gear - 1;
		}
	}
	return car->_gear;
}


/* check if the car is stuck */
const float MAX_UNSTUCK_SPEED = 5.0;   /* [m/s] */
const float MIN_UNSTUCK_DIST = 3.0;    /* [m] */
const float MAX_UNSTUCK_ANGLE = 20.0/180.0*PI;
const int MAX_UNSTUCK_COUNT = 250;
static int stuck = 0;

bool isStuck(tCarElt* car)
{
    float angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle);

    if (fabs(angle) > MAX_UNSTUCK_ANGLE &&
        car->_speed_x < MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
        if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*angle < 0.0) {
            return true;
        } else {
            stuck++;
            return false;
        }
    } else {
        stuck = 0;
        return false;
    }
}


/* Drive during race. */
extern double* psteerCmd;
extern double* paccelCmd;
extern double* pbrakeCmd;

extern double* pspeed;
extern double* ptoMiddle;
extern double* pangle;

extern double* pfast;

extern double* pdist_L;
extern double* pdist_R;

extern double* ptoMarking_L;
extern double* ptoMarking_M;
extern double* ptoMarking_R;

extern double* pdist_LL;
extern double* pdist_MM;
extern double* pdist_RR;

extern double* ptoMarking_LL;
extern double* ptoMarking_ML;
extern double* ptoMarking_MR;
extern double* ptoMarking_RR;

// Compute the length to the start of the segment.
float getDistToSegStart(tCarElt *ocar)
{
    if (ocar->_trkPos.seg->type == TR_STR) {
        return ocar->_trkPos.toStart;
    } else {
        return ocar->_trkPos.toStart*ocar->_trkPos.seg->radius;
    }
}


static void drive(int index, tCarElt* car, tSituation *s) 
{ 
    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    if (isStuck(car)) {
        float angle = -RtTrackSideTgAngleL(&(car->_trkPos)) + car->_yaw;
        NORM_PI_PI(angle); // put the angle back in the range from -PI to PI

        car->ctrl.steer = angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.9; // 90% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } 
    else {
/////////////////////////////////////// for label learning
///////// drive on the road with 3 lanes

        car->ctrl.gear = getGear(car);

        car->ctrl.steer = *psteerCmd;
        car->ctrl.accelCmd = *paccelCmd;
        car->ctrl.brakeCmd = *pbrakeCmd;


        const float lane_width=4.0;
        const float half_lane_width=lane_width/2;
        const float max_dist_range=60.0;
        const float deactivated_dist_range=75.0;

        const float max_toMarking_LR=7.0;
        const float max_toMarking_M=3.5;
        const float max_toMarking_LLRR=9.5;
        const float max_toMarking_MM=5.5;

        float min_dist[3]={99999,99999,99999};
        float distance;
        tCarElt* min_car[3]={0,0,0};

        float fast=0.0;

        float dist_L=deactivated_dist_range;
        float dist_R=deactivated_dist_range;

        float toMarking_L=-max_toMarking_LR;
        float toMarking_M=max_toMarking_M;
        float toMarking_R=max_toMarking_LR;

        float dist_LL=deactivated_dist_range;
        float dist_MM=deactivated_dist_range;
        float dist_RR=deactivated_dist_range;

        float toMarking_LL=-max_toMarking_LLRR;
        float toMarking_ML=-max_toMarking_MM;
        float toMarking_MR=max_toMarking_MM;
        float toMarking_RR=max_toMarking_LLRR;

	for (int i = 0; i < s->_ncars; i++) {

            if (s->cars[i] != car) {
	        distance = s->cars[i]->_trkPos.seg->lgfromstart + getDistToSegStart(s->cars[i]) - car->_distFromStartLine;
    	        if (distance > curTrack->length/2.0f) {
    	            distance -= curTrack->length;
	        } else if (distance < -curTrack->length/2.0f) {
	            distance += curTrack->length;
	        }

                if (distance>0) {
                    if (s->cars[i]->_trkPos.toMiddle>half_lane_width && distance<min_dist[0]) {  // on left lane
                        min_dist[0]=distance;
                        min_car[0]=s->cars[i];
                    } else if (s->cars[i]->_trkPos.toMiddle<-half_lane_width && distance<min_dist[2]) {  // on right lane
                        min_dist[2]=distance;
                        min_car[2]=s->cars[i];
                    } else if (s->cars[i]->_trkPos.toMiddle>=-half_lane_width && s->cars[i]->_trkPos.toMiddle<=half_lane_width && distance<min_dist[1]) {  // on central lane
                        min_dist[1]=distance;
                        min_car[1]=s->cars[i];
                    }
                }
            }
	}

        if (min_dist[0]>max_dist_range) min_dist[0]=max_dist_range;
        if (min_dist[1]>max_dist_range) min_dist[1]=max_dist_range;
        if (min_dist[2]>max_dist_range) min_dist[2]=max_dist_range;

        if (car->_trkPos.toMiddle<=-(half_lane_width+0.5) && car->_trkPos.toMiddle>=-(lane_width+half_lane_width-0.5)) { // on the right lane
           toMarking_LL=car->_trkPos.toMiddle-half_lane_width;
           toMarking_ML=car->_trkPos.toMiddle+half_lane_width;
           toMarking_MR=car->_trkPos.toMiddle+lane_width+half_lane_width;
           toMarking_RR=max_toMarking_LLRR;
           if (min_car[1]!=0) dist_LL=min_dist[1];
           else dist_LL=max_dist_range;
           if (min_car[2]!=0) dist_MM=min_dist[2];
           else dist_MM=max_dist_range;
           // dist_RR=deactivated_dist_range;
        } else if (car->_trkPos.toMiddle<=(lane_width+half_lane_width-0.5) && car->_trkPos.toMiddle>=(half_lane_width+0.5)) { // on the left lane
           toMarking_LL=-max_toMarking_LLRR;
           toMarking_ML=car->_trkPos.toMiddle-lane_width-half_lane_width;
           toMarking_MR=car->_trkPos.toMiddle-half_lane_width;
           toMarking_RR=car->_trkPos.toMiddle+half_lane_width;
           // dist_LL=deactivated_dist_range;
           if (min_car[0]!=0) dist_MM=min_dist[0];
           else dist_MM=max_dist_range;
           if (min_car[1]!=0) dist_RR=min_dist[1]; 
           else dist_RR=max_dist_range;
        } else if (car->_trkPos.toMiddle<=(half_lane_width-0.5) && car->_trkPos.toMiddle>=-(half_lane_width-0.5)) { // on the central lane
           toMarking_LL=car->_trkPos.toMiddle-lane_width-half_lane_width;
           toMarking_ML=car->_trkPos.toMiddle-half_lane_width;
           toMarking_MR=car->_trkPos.toMiddle+half_lane_width;
           toMarking_RR=car->_trkPos.toMiddle+lane_width+half_lane_width;
           if (min_car[0]!=0) dist_LL=min_dist[0];
           else dist_LL=max_dist_range;
           if (min_car[1]!=0) dist_MM=min_dist[1]; 
           else dist_MM=max_dist_range;
           if (min_car[2]!=0) dist_RR=min_dist[2];   
           else dist_RR=max_dist_range;        
        }

        if (car->_trkPos.toMiddle<=(half_lane_width+1) && car->_trkPos.toMiddle>=(half_lane_width-1)) { // on the center L marking
           toMarking_L=car->_trkPos.toMiddle-lane_width-half_lane_width;
           toMarking_M=car->_trkPos.toMiddle-half_lane_width;
           toMarking_R=car->_trkPos.toMiddle+half_lane_width;
           if (min_car[0]!=0) dist_L=min_dist[0];
           else dist_L=max_dist_range;
           if (min_car[1]!=0) dist_R=min_dist[1];
           else dist_R=max_dist_range;
        } else if (car->_trkPos.toMiddle<=-(half_lane_width-1) && car->_trkPos.toMiddle>=-(half_lane_width+1)) { // on the center R marking
           toMarking_L=car->_trkPos.toMiddle-half_lane_width;
           toMarking_M=car->_trkPos.toMiddle+half_lane_width;
           toMarking_R=car->_trkPos.toMiddle+lane_width+half_lane_width;
           if (min_car[1]!=0) dist_L=min_dist[1];
           else dist_L=max_dist_range;
           if (min_car[2]!=0) dist_R=min_dist[2];
           else dist_R=max_dist_range;
        } else if (car->_trkPos.toMiddle<=-(lane_width+half_lane_width-1) && car->_trkPos.toMiddle>=-(lane_width+half_lane_width+2)) { // on the right marking
           toMarking_L=car->_trkPos.toMiddle+half_lane_width;
           toMarking_M=car->_trkPos.toMiddle+lane_width+half_lane_width;
           toMarking_R=max_toMarking_LR;
           if (min_car[2]!=0) dist_L=min_dist[2];
           else dist_L=max_dist_range;
           //dist_R=deactivated_dist_range;
        } else if (car->_trkPos.toMiddle<=(lane_width+half_lane_width+2) && car->_trkPos.toMiddle>=(lane_width+half_lane_width-1)) { // on the left marking
           toMarking_L=-max_toMarking_LR;
           toMarking_M=car->_trkPos.toMiddle-lane_width-half_lane_width;
           toMarking_R=car->_trkPos.toMiddle-half_lane_width;
           //dist_L=deactivated_dist_range;
           if (min_car[0]!=0) dist_R=min_dist[0];
           else dist_R=max_dist_range;
        }

        if (car->_trkPos.seg->type == TR_STR) {
           float str_length=car->_trkPos.seg->length - car->_trkPos.toStart;
           tTrackSeg *segptr = car->_trkPos.seg->next;
           while (segptr->type == TR_STR) {
               str_length=str_length+segptr->length;
               segptr = segptr->next;
           }

           if (str_length>60) fast=1.0;
        }

        float angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
        NORM_PI_PI(angle);
        *pangle = angle;
        *ptoMiddle = car->_trkPos.toMiddle;
        *pspeed = car->_speed_x;

        *pfast = fast;

        *pdist_L = dist_L;
        *pdist_R = dist_R;

        *ptoMarking_L = toMarking_L;
        *ptoMarking_M = toMarking_M;
        *ptoMarking_R = toMarking_R;

        *pdist_LL = dist_LL;
        *pdist_MM = dist_MM;
        *pdist_RR = dist_RR;

        *ptoMarking_LL = toMarking_LL;
        *ptoMarking_ML = toMarking_ML;
        *ptoMarking_MR = toMarking_MR;
        *ptoMarking_RR = toMarking_RR;
/////////////////////////////////////// for label learning
    }    
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}

