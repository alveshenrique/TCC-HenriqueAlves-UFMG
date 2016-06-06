/******************************************************************************
 *
 * Copyright (C) 2014 Microchip Technology Inc. and its
 *                    subsidiaries ("Microchip").
 *
 * All rights reserved.
 *
 * You are permitted to use the Aurea software, GestIC API, and other
 * accompanying software with Microchip products.  Refer to the license
 * agreement accompanying this software, if any, for additional info regarding
 * your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
 * MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP, SMSC, OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH
 * OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY FOR ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT OR CONSEQUENTIAL DAMAGES, OR OTHER SIMILAR COSTS.
 *
 ******************************************************************************/
#include "console.h"

/* MATLAB INTERFACE*/

#include <Engine.h>

#pragma comment ( lib, "libmat.lib")
#pragma comment ( lib, "libmx.lib")
#pragma comment ( lib, "libmex.lib")
#pragma comment ( lib, "libeng.lib")

void init_data(data_t *data)
{
    /* The demo should be running */
    data->running = 1;

    /* Render menu on startup. */
    data->menu_current = -1;

    /* Automatic calibration enabled by default */
    data->auto_calib = 1;

    /* No gestures so far */
    data->last_gesture = 0;

    /* No gestic_t instance allocated yet */
    data->gestic = NULL;
}

int main()
{
    data_t data;


	/* MATLAB INTERFACE */
	Engine *m_pEngine;
	mxArray *dX = mxCreateDoubleMatrix(1, 1, mxREAL), *dY = mxCreateDoubleMatrix(1, 1, mxREAL), *dZ = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray *dLastGesture = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray *dGesture = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray *dAirWheel = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray *dTouch = mxCreateDoubleMatrix(1, 1, mxREAL);
	mxArray *dFlagTouch = mxCreateDoubleMatrix(1, 1, mxREAL);


	double *pX = mxGetPr(dX), *pY = mxGetPr(dY), *pZ = mxGetPr(dZ);
	double *pLastGesture = mxGetPr(dLastGesture);
	double *pGesture = mxGetPr(dGesture);
	double *pAirWheel = mxGetPr(dAirWheel);
	double *pTouch = mxGetPr(dTouch);
	double *pFlagTouch = mxGetPr(dFlagTouch);


	/* Engine Init. */
	m_pEngine = engOpen("null");

	/* Initial values loaded to the variables measured by the MGC3130 */
	*pX = 1;
	*pY = 2;
	*pZ = 3;
	*pLastGesture = (double) 0;
	*pGesture = (double)5;
	*pAirWheel = (double)-1;
	*pTouch = (double)5;
	*pFlagTouch = (double)6;


	/* Loading those values at MATLAB matrix */
	engPutVariable(m_pEngine, "dX", dX);
	engPutVariable(m_pEngine, "dY", dY);
	engPutVariable(m_pEngine, "dZ", dZ);
	engPutVariable(m_pEngine, "dLastGesture", dLastGesture);
	engPutVariable(m_pEngine, "dGesture", dGesture);
	engPutVariable(m_pEngine, "dTouch", dTouch);
	engPutVariable(m_pEngine, "dFlagTouch", dFlagTouch);


	/* Add the path to the MATLAB script that will be used. */
	engEvalString(m_pEngine, "addpath(genpath( 'C:\\Users\\Henrique\\OneDrive\\Documents\\UFMG\\TCC\\MATLAB'))");
	engEvalString(m_pEngine, "addpath(genpath( 'C:\\Users\\Henrique\\OneDrive\\Documents\\UFMG\TCC\\External Libraries\\dq_robotics\\ax18smartrobotarm\\AX18Smart'))");
	engEvalString(m_pEngine, "addpath(genpath( 'C:\\Users\\Henrique\\OneDrive\\Documents\\UFMG\TCC\\External Libraries\\ICREATE'))");

	/* Demo code. */

    printf("\n\tGestIC %s - Console Demo\n\n", gestic_version_str());

    /* Initialize data and the device */
    init_data(&data);
    init_device(&data);

    /* Update device and menu until quit */
    while(data.running)
    {
		int i = 0;

		update_device(&data);
        update_menu(&data);
		
		/* Get the data */
		*pX = (double) data.gestic_pos->x;
		*pY = (double) data.gestic_pos->y;
		*pZ = (double) data.gestic_pos->z;
		*pLastGesture = (double)data.last_gesture;
		if (*pLastGesture != 0)
		{
			*pGesture = *pLastGesture;
		}
			
		*pAirWheel = (double) data.gestic_air_wheel->counter;
		*pFlagTouch = (double)0;
		for (i = 0; i < 5; ++i) {
			if (data.gestic_touch->flags & (1 << i))
			{
				*pTouch = (double)i;
				*pFlagTouch = (double)1;
			}

				
		}

		

		

		/* Load the data to MATLAB the matrix we will use in our MATLAB script */
		engPutVariable(m_pEngine, "dX", dX);
		engPutVariable(m_pEngine, "dY", dY);
		engPutVariable(m_pEngine, "dZ", dZ);
		engPutVariable(m_pEngine, "dLastGesture", dLastGesture);
		engPutVariable(m_pEngine, "dGesture", dGesture);
		engPutVariable(m_pEngine, "dAirWheel", dAirWheel);
		engPutVariable(m_pEngine, "dTouch", dTouch);
		engPutVariable(m_pEngine, "dFlagTouch", dFlagTouch);

        Sleep(10);

    }

    /* Do the cleanup work */
    free_device(&data);

	/* Close the engine. */
	engClose(m_pEngine);

    return 0;
}
