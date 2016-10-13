/***************************************************************************

    file                 : main.cpp
    created              : Sat Sep  2 10:40:47 CEST 2000
    copyright            : (C) 2000 by Patrice & Eric Espie
    email                : torcs@free.fr
    version              : $Id: main.cpp,v 1.9.2.2 2012/02/16 14:11:47 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef WIN32
#include <windows.h>
#include <stdlib.h>
#endif
#include <GL/glut.h>
#include <tgfclient.h>
#include <client.h>
#include <portability.h>
#include <raceinit.h>

#include "windowsspec.h"

static void
init_args(int argc, char **argv, const char** raceconfig)
{
	int i;
	i = 1;
	while (i < argc) {
		if ((strncmp(argv[i], "-s", 2) == 0) || (strncmp(argv[i], "/s", 2) == 0)) {
			i++;
			SetSingleTextureMode ();
		} else if ((strncmp(argv[i], "-r", 2) == 0) || (strncmp(argv[i], "/r", 2) == 0)) {
			i++;
			*raceconfig = "";
			if (i < argc) {
				*raceconfig = argv[i];
				i++;
			}

			if ((strlen(*raceconfig) == 0) || (strstr(*raceconfig, ".xml") == 0)) {
				printf("Please specify a race configuration xml when using -r\n");
				exit(1);
			}
		} else {
			i++;		// Ignore unknown arguments
		}
	}

	static const int BUFSIZE = 1024;
	char buf[BUFSIZE];
	strncpy(buf, argv[0], BUFSIZE);
	buf[BUFSIZE-1] = '\0';	// Guarantee zero termination for next operation.
	char *end = strrchr(buf, '\\');

	// Did we find the last '\' and do we get a complete path?
	if (end != NULL && buf[1] == ':') {
		end++;
		*(end) = '\0';
		// replace '\' with '/'
		for (i = 0; i < BUFSIZE && buf[i] != '\0'; i++) {
			if (buf[i] == '\\') {
				buf[i] = '/';
			}
		}

		// TODO: Let localdir point to users "home" directory (I think on NT successors this exists,
		// perhaps HOMEDRIVE, HOMEPATH).
		SetLocalDir(buf);
		SetDataDir(buf);
		SetLibDir("");
	} else {
		if (_fullpath(buf, argv[0], BUFSIZE) != NULL &&
			(strcmp(argv[0], "wtorcs") == 0 ||
			 strcmp(argv[0], "wtorcs.exe") == 0)
		   )
		{
			end = strrchr(buf, '\\');
			end++;
			*(end) = '\0';
			// replace '\' with '/'
			for (i = 0; i < BUFSIZE && buf[i] != '\0'; i++) {
				if (buf[i] == '\\') {
					buf[i] = '/';
				}
			}
			SetLocalDir(buf);
			SetDataDir(buf);
			SetLibDir("");
		} else {
			printf("Run wtorcs.exe either from the GUI or from the directory which contains wtorcs.exe\n");
			exit(1);
		}
	}
}

/*
 * Function
 *	main
 *
 * Description
 *	Win32 entry point of TORCS
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
int
main(int argc, char *argv[])
{
	const char* raceconfig = "";

	init_args(argc, argv, &raceconfig);
	WindowsSpecInit();			/* init specific windows functions */

	if (strlen(raceconfig) == 0) {
		GfScrInit(argc, argv);	/* init screen */
		TorcsEntry();			/* launch TORCS */
		glutMainLoop();			/* event loop of glut */
	} else {
		// Run race from console, no Window, no OpenGL/OpenAL etc.
		// Thought for blind scripted AI training
		ReRunRaceOnConsole(raceconfig);
	}

	return 0;			/* just for the compiler, never reached */
}

