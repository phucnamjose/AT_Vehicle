/*
 * trajectory.c
 *
 *  Created on: Nov 11, 2020
 *      Author: Dang Nam
 */


#include "trajectory.h"
#include "math.h"

uint8_t	trajecInitLSPB	(Trajectory_t *lspb,
						double total_s,
						ModeInit_t modeinit,
						double v_design,
						double a_design) {
	double v_lim, q0, q1, v0, v1, ta, td, tf;
	uint32_t no_sample;
	int8_t	dir;

	q0 = 0;
	q1 = total_s;
	v0 = 0;
	v1 = 0;

	if ( q0 <= q1) {
		dir = 1;
	} else {
		dir = -1;
		q1 = -q1;
	}

	if (TRAJEC_INIT_QVT == modeinit) {
		double v_lower, v_upper, tc_upper, a_upper;
		tf = lspb->Tf;
		// Avoid division by 0
		if (tf > 0.001) {
			v_lower 	= (q1 - q0) / tf;
			v_upper 	= 2*(q1 - q0) / tf;

			if ( v_design < v_lower) {
				return FALSE;
			} else {
				if ( v_upper <= v_design) {
					v_design = v_upper;
				}
				// Avoid division by 0
				if (v_design > 0.0000001) {
					tc_upper	= tf - (q1 - q0)/v_design;
				} else {
					tc_upper = tf / 2;
				}
				a_upper	= v_design/tc_upper;
				if ( a_upper > a_design) {
					return FALSE;
				} else {
					a_design = a_upper;
				}
			}
		} else {
		v_design = 0;
		a_design = 0;
		}
	}

	// Check condition trapezoidal ---> triangle
	// Avoid division by 0
	if (a_design > 0.0000001 && v_design > 0.0000001) {
		if ((fabs(q1 - q0)*a_design) <= (v_design*v_design - (v0*v0 + v1*v1)/2)) {
			v_lim 	= sqrt(fabs(q1 - q0)*a_design + (v0*v0 + v1*v1)/2);
			ta		= (v_lim - v0)/a_design;
			td		= (v_lim - v1)/a_design;
			tf		= ta + td;
		} else {
			v_lim	= v_design;
			ta		= (v_lim - v0)/a_design;
			td		= (v_lim - v1)/a_design;
			tf		= fabs(q0 - q1)/v_lim + v_lim/(2*a_design)*(1 - v0/v_lim)*(1 - v0/v_lim)
						+ v_lim/(2*a_design)*(1 - v1/v_lim)*(1 - v1/v_lim);
		}
	} else {
		v_lim	= 0;
		ta = tf/2;
		td = tf/2;
	}

	no_sample = ceilf(tf / 0.01); // ceiling
	// Init lspb params
	lspb->dir= dir;
	lspb->s0 = q0;
	lspb->s1 = q1;
	lspb->Ta = ta;
	lspb->Td = td;
	lspb->Tf = tf;
	lspb->a_design = a_design;
	lspb->v_design = v_design;
	lspb->v_lim = v_lim;
	lspb->v0 = v0;
	lspb->v1 = v1;
	lspb->num_of_sampling = no_sample;
	lspb->total_s = lspb->s1 - lspb->s0;

	return TRUE;
}

uint8_t	trajecFlowLSPB	(Trajectory_t *lspb, double time) {
	double tf, td, ta;

	tf = lspb->Tf;
	td = lspb->Td;
	ta = lspb->Ta;

	// Accelerate
	if ( 0.0f <= time && time <= ta) {
		lspb->a_current		=	lspb->a_design;
		lspb->v_current		=	lspb->v0 + lspb->a_design*time;
		lspb->s_current		=	lspb->s0 + lspb->v0*time + 0.5*lspb->a_design*time*time;
	// Constant velocity
	} else if (ta <= time && time <= (tf - td)) {
		lspb->a_current		=	0;
		lspb->v_current		=	lspb->v_lim;
		lspb->s_current		=	lspb->s0 + lspb->v0*ta/2 + lspb->v_lim*(time - ta/2);
	// Decelerate
	} else if ((tf - td) <= time && time <= tf) {
		lspb->a_current		=	-lspb->a_design;
		lspb->v_current		=	lspb->v1 + lspb->a_design*(tf - time);
		lspb->s_current		=	lspb->s1 - lspb->v1*(tf - time)
								- (lspb->v_lim - lspb->v1)*(tf - time)*(tf - time)/(2*td);
	} else {
		lspb->a_current 	=	0;
		lspb->v_current		=	0;
		lspb->s_current		=	lspb->total_s;
	}

	return TRUE;
}
