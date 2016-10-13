/***************************************************************************

    file                 : main.cpp
    created              : Sat Mar 18 23:54:30 CET 2000
    copyright            : (C) 2000 by Eric Espie
    email                : torcs@free.fr
    version              : $Id: main.cpp,v 1.14.2.3 2012/06/01 01:59:42 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <stdlib.h>

#include <GL/glut.h>

#include <tgfclient.h>
#include <client.h>

#include "linuxspec.h"
#include <raceinit.h>

/////////////////////////////////// by Chenyi
#include <sys/shm.h> 
#define image_width 640
#define image_height 480
/////////////////////////////////// by Chenyi

extern bool bKeepModules;

static void
init_args(int argc, char **argv, const char **raceconfig)
{
	int i;
	char *buf;

	i = 1;

	while(i < argc) {
		if(strncmp(argv[i], "-l", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetLocalDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-L", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetLibDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-D", 2) == 0) {
			i++;

			if(i < argc) {
				buf = (char *)malloc(strlen(argv[i]) + 2);
				sprintf(buf, "%s/", argv[i]);
				SetDataDir(buf);
				free(buf);
				i++;
			}
		} else if(strncmp(argv[i], "-s", 2) == 0) {
			i++;
			SetSingleTextureMode();
		} else if(strncmp(argv[i], "-k", 2) == 0) {
			i++;
			// Keep modules in memory (for valgrind)
			printf("Unloading modules disabled, just intended for valgrind runs.\n");
			bKeepModules = true;
#ifndef FREEGLUT
		} else if(strncmp(argv[i], "-m", 2) == 0) {
			i++;
			GfuiMouseSetHWPresent(); /* allow the hardware cursor */
#endif
		} else if(strncmp(argv[i], "-r", 2) == 0) {
			i++;
			*raceconfig = "";

			if(i < argc) {
				*raceconfig = argv[i];
				i++;
			}

			if((strlen(*raceconfig) == 0) || (strstr(*raceconfig, ".xml") == 0)) {
				printf("Please specify a race configuration xml when using -r\n");
				exit(1);
			}
		} else {
			i++;		/* ignore bad args */
		}
	}

#ifdef FREEGLUT
	GfuiMouseSetHWPresent(); /* allow the hardware cursor (freeglut pb ?) */
#endif
}

/*
 * Function
 *	main
 *
 * Description
 *	LINUX entry point of TORCS
 *
 * Parameters
 *
 *
 * Return
 *
 *
 * Remarks
 *
 */

//////////////////////////////////////////// by Chenyi
struct shared_use_st  
{  
    int written;
    uint8_t data[image_width*image_height*3]; 
    int control;
    int pause;
    double fast;

    double dist_L;
    double dist_R;

    double toMarking_L;
    double toMarking_M;
    double toMarking_R;

    double dist_LL;
    double dist_MM;
    double dist_RR;

    double toMarking_LL;
    double toMarking_ML;
    double toMarking_MR;
    double toMarking_RR;

    double toMiddle;
    double angle;
    double speed;

    double steerCmd;
    double accelCmd;
    double brakeCmd;
}; 

int* pwritten = NULL;
uint8_t* pdata = NULL;
int* pcontrol = NULL;
int* ppause = NULL;

double* psteerCmd_ghost = NULL;
double* paccelCmd_ghost = NULL;
double* pbrakeCmd_ghost = NULL;

double* pspeed_ghost = NULL;
double* ptoMiddle_ghost = NULL;
double* pangle_ghost = NULL;

double* pfast_ghost = NULL;

double* pdist_L_ghost = NULL;
double* pdist_R_ghost = NULL;

double* ptoMarking_L_ghost = NULL;
double* ptoMarking_M_ghost = NULL;
double* ptoMarking_R_ghost = NULL;

double* pdist_LL_ghost = NULL;
double* pdist_MM_ghost = NULL;
double* pdist_RR_ghost = NULL;

double* ptoMarking_LL_ghost = NULL;
double* ptoMarking_ML_ghost = NULL;
double* ptoMarking_MR_ghost = NULL;
double* ptoMarking_RR_ghost = NULL;

void *shm = NULL;
//////////////////////////////////////////// by Chenyi

int
main(int argc, char *argv[])
{

//////////////////////////////////////////// by Chenyi 
    struct shared_use_st *shared = NULL;
    int shmid;  
    // establish memory sharing 
    shmid = shmget((key_t)4567, sizeof(struct shared_use_st), 0666|IPC_CREAT);  
    if(shmid == -1)  
    {  
        fprintf(stderr, "shmget failed\n");  
        exit(EXIT_FAILURE);  
    }  
  
    shm = shmat(shmid, 0, 0);  
    if(shm == (void*)-1)  
    {  
        fprintf(stderr, "shmat failed\n");  
        exit(EXIT_FAILURE);  
    }  
    printf("\n********** Memory sharing started, attached at %X **********\n \n", shm);  
    // set up shared memory 
    shared = (struct shared_use_st*)shm;  
    shared->written = 0;
    shared->control = 0;
    shared->pause = 0;
    shared->fast = 0.0;

    shared->dist_L = 0.0;
    shared->dist_R = 0.0;

    shared->toMarking_L = 0.0;
    shared->toMarking_M = 0.0;
    shared->toMarking_R = 0.0;

    shared->dist_LL = 0.0;
    shared->dist_MM = 0.0;
    shared->dist_RR = 0.0;

    shared->toMarking_LL = 0.0;
    shared->toMarking_ML = 0.0;
    shared->toMarking_MR = 0.0;
    shared->toMarking_RR = 0.0;

    shared->toMiddle = 0.0;
    shared->angle = 0.0;
    shared->speed = 0.0;

    shared->steerCmd = 0.0;
    shared->accelCmd = 0.0;
    shared->brakeCmd = 0.0;

    pwritten=&shared->written;
    pdata=shared->data;
    pcontrol=&shared->control;
    ppause=&shared->pause;

    psteerCmd_ghost=&shared->steerCmd;
    paccelCmd_ghost=&shared->accelCmd;
    pbrakeCmd_ghost=&shared->brakeCmd;

    pspeed_ghost=&shared->speed;
    ptoMiddle_ghost=&shared->toMiddle;
    pangle_ghost=&shared->angle;

    pfast_ghost=&shared->fast;

    pdist_L_ghost=&shared->dist_L;
    pdist_R_ghost=&shared->dist_R;

    ptoMarking_L_ghost=&shared->toMarking_L;
    ptoMarking_M_ghost=&shared->toMarking_M;
    ptoMarking_R_ghost=&shared->toMarking_R;

    pdist_LL_ghost=&shared->dist_LL;
    pdist_MM_ghost=&shared->dist_MM;
    pdist_RR_ghost=&shared->dist_RR;

    ptoMarking_LL_ghost=&shared->toMarking_LL;
    ptoMarking_ML_ghost=&shared->toMarking_ML;
    ptoMarking_MR_ghost=&shared->toMarking_MR;
    ptoMarking_RR_ghost=&shared->toMarking_RR;
/////////////////////////////////////////// by Chenyi

	const char *raceconfig = "";

	init_args(argc, argv, &raceconfig);
	LinuxSpecInit();			/* init specific linux functions */

	if(strlen(raceconfig) == 0) {
		GfScrInit(argc, argv);	/* init screen */
		TorcsEntry();			/* launch TORCS */
		glutMainLoop();			/* event loop of glut */
	} else {
		// Run race from console, no Window, no OpenGL/OpenAL etc.
		// Thought for blind scripted AI training
		ReRunRaceOnConsole(raceconfig);
	}

	return 0;					/* just for the compiler, never reached */
}

