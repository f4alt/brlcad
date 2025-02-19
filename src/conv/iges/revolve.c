/*                       R E V O L V E . C
 * BRL-CAD
 *
 * Copyright (c) 1990-2022 United States Government as represented by
 * the U.S. Army Research Laboratory.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 2.1 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this file; see the file named COPYING for more
 * information.
 */
/** @file iges/revolve.c
 *
 * SOLID OF REVOLUTION
 *
 */

#include "./iges_struct.h"
#include "./iges_extern.h"
#include "vmath.h"

void Addsub();


struct subtracts
{
    struct bu_vls name;
    int index;
    struct subtracts *next;
};


struct trclist
{
    point_t base, top;
    fastf_t r1, r2;
    int op; /* 0 => union, 1=> subtract */
    int index;
    struct bu_vls name;
    struct subtracts *subtr;
    struct trclist *next, *prev;
};


int
revolve(int entityno)
{
    struct wmember head;			/* For region */
    int sol_num;		/* IGES solid type number */
    point_t pt;			/* Point on axis of revolution */
    vect_t adir;			/* Direction of axis of revolution */
    int curve;			/* Pointer to directory entry for curve */
    fastf_t fract;			/* Fraction of circle for rotation (0 < fract <= 1.0) */
    vect_t v1;			/* Vector from "pt" to any point along curve */
    fastf_t h;			/* height of "TRC" */
    int npts;			/* Number of points used to approximate curve */
    struct ptlist *curv_pts, *ptr;		/* Pointer to a linked list of npts points along curve */
    int ntrcs;			/* number of "TRC" solids used */
    vect_t tmp;			/* temporary storage for a vector */
    struct trclist *trcs, *trcptr, *trcptr_tmp, *ptr2;	/* Pointers to linked list of TRC`s */
    fastf_t r2;			/* TRC radius */
    fastf_t hmax, hmin;		/* Max and Min distances along axis of rotation */
    fastf_t rmax;			/* Max radius */
    int cutop = Intersect;	/* Operator for cutting solid */
    struct bu_vls cutname;	/* Name for cutting solid */
    struct subtracts *subp;
    int i;

    BU_LIST_INIT(&head.l);

    /* Default values */
    VSET(adir, 0.0, 0.0, 1.0);
    VSET(pt, 0.0, 0.0, 0.0);
    fract = 1.0;

    /* Acquire data */

    if (dir[entityno]->param <= pstart) {
	bu_log("Illegal parameter pointer for entity D%07d (%s)\n" ,
	       dir[entityno]->direct, dir[entityno]->name);
	return 0;
    }
    Readrec(dir[entityno]->param);
    Readint(&sol_num, "");

    /* Read pointer to directory entry for curve to be extruded */

    Readint(&curve, "");

    /* Convert this to a "dir" index */

    curve = (curve-1)/2;

    Readflt(&fract, "");
    Readflt(&pt[X], "");
    Readflt(&pt[Y], "");
    Readflt(&pt[Z], "");
    Readflt(&adir[X], "");
    Readflt(&adir[Y], "");
    Readflt(&adir[Z], "");

    /* just to be safe */
    VUNITIZE(adir);

    if (fract <= 0.0 || fract > 1.0) {
	bu_log("Illegal parameters for entity D%07d (%s)\n" ,
	       dir[entityno]->direct, dir[entityno]->name);
	return 0;
    }

    dir[entityno]->referenced = 1;

    /* Get the curve in the form of a series of straight line segments */

    npts = Getcurve(curve, &curv_pts);
    if (npts == 0) {
	bu_log("Could not get points along curve for revolving\n");
	bu_log("Illegal parameters for entity D%07d (%s)\n" ,
	       dir[entityno]->direct, dir[entityno]->name);
	return 0;
    }

/* Construct a linked list of TRC's */
    ntrcs = 0;
    trcs = NULL;
    ptr = curv_pts;

    /* Calculate radius at start of curve */
    VSUB2(v1, ptr->pt, pt);
    VCROSS(tmp, v1, adir);
    r2 = MAGNITUDE(tmp);
    V_MAX(r2, TOL);

    rmax = r2;
    hmax = VDOT(v1, adir);
    hmin = hmax;

    trcptr = NULL;
    while (ptr->next != NULL) {
	struct trclist *prev;
	fastf_t h1;

	if (trcs == NULL) {
	    BU_ALLOC(trcs, struct trclist);
	    trcptr = trcs;
	    prev = NULL;
	} else if (bu_vls_cstr(&trcptr->name)[0] != '\0') {
	    BU_ALLOC(trcptr->next, struct trclist);
	    prev = trcptr;
	    trcptr = trcptr->next;
	} else prev = NULL;
	trcptr->next = NULL;
	trcptr->prev = prev;
	trcptr->op = 0;
	trcptr->subtr = NULL;
	bu_vls_addr(&trcptr->name)[0] = '\0';

	/* Calculate base point of TRC */
	VSUB2(v1, ptr->pt, pt);
	VJOIN1(trcptr->base, pt, VDOT(v1, adir), adir);

	/* Height along axis of rotation */
	h1 = VDOT(v1, adir);
	V_MIN(hmin, h1);
	V_MAX(hmax, h1);

	/* Radius at base is top radius from previous TRC */
	trcptr->r1 = r2;

	/* Calculate new top radius */
	VSUB2(v1, ptr->next->pt, pt);
	VCROSS(tmp, v1, adir);
	trcptr->r2 = MAGNITUDE(tmp);
	V_MAX(trcptr->r2, TOL);

	r2 = trcptr->r2;
	V_MIN(rmax, r2);

	/* Calculate height of TRC */
	VSUB2(v1, ptr->next->pt, pt);
	VJOIN1(trcptr->top, pt, VDOT(v1, adir), adir);
	VSUB2(v1, trcptr->top, trcptr->base);
	h = MAGNITUDE(v1);
	/* If height is zero, don't make a TRC */
	if (NEAR_ZERO(h, TOL)) {
	    ptr = ptr->next;
	    continue;
	}

	/* Make a name for the TRC */
	bu_vls_sprintf(&trcptr->name, "rev.%d.%d", entityno, ntrcs); /* Format for creating TRC names */

	/* Make the TRC */
	if (mk_trc_top(fdout, bu_vls_cstr(&trcptr->name), trcptr->base,
		       trcptr->top, trcptr->r1, trcptr->r2) < 0) {
	    bu_log("Unable to write TRC for entity D%07d (%s)\n" ,
		   dir[entityno]->direct, dir[entityno]->name);
	    return 0;
	}

	/* Count 'em */
	ntrcs++;
	ptr = ptr->next;
    }

    /* Eliminate last struct if not used */
    if (trcptr && bu_vls_cstr(&trcptr->name)[0] == '\0') {
      if (trcptr->prev) trcptr->prev->next = NULL;
      bu_free((char *)trcptr, "Revolve: trcptr");
    }

    if (dir[entityno]->form == 1) {
	/* curve closed on itself */
	trcptr = trcs;
	while (trcptr != NULL) {
	    fastf_t hb1, ht1, hb2, ht2; /* distance from "pt" to bottom and top of TRC's */
	    fastf_t rtmp;	/* interpolated radii for TRC */
	    fastf_t tmpp;	/* temp storage */

	    /* Calculate distances to top and base */
	    VSUB2(tmp, trcptr->base, pt);
	    hb1 = MAGNITUDE(tmp);
	    VSUB2(tmp, trcptr->top, pt);
	    ht1 = MAGNITUDE(tmp);
	    /* Make sure distance to base is smaller */
	    if (ht1 < hb1) {
		tmpp = ht1;
		ht1 = hb1;
		hb1 = tmpp;
	    }

	    /* Check every TRC against this one */
	    ptr2 = trcs;
	    while (ptr2 != NULL) {
		if (ptr2 == trcptr) /* but not itself */
		    ptr2 = ptr2->next;
		else {
		    /* Calculate heights */
		    VSUB2(tmp, ptr2->base, pt);
		    hb2 = MAGNITUDE(tmp);
		    VSUB2(tmp, ptr2->top, pt);
		    ht2 = MAGNITUDE(tmp);
		    /* and order them */
		    if (ht2 < hb2) {
			tmpp = ht2;
			ht2 = hb2;
			hb2 = tmpp;
		    }
		    if (hb2 < ht1 && hb2 > hb1) {
			/* These TRC's overlap */
			/* Calculate radius at hb2 */
			rtmp = trcptr->r1 + (trcptr->r2 - trcptr->r1)*(hb2-hb1)/(ht1-hb1);
			if (rtmp > ptr2->r1) {
			    /* ptr2 must be an inside solid, so subtract it */
			    Addsub(trcptr, ptr2);
			    ptr2->op = 1;
			} else if (rtmp < ptr2->r1) {
			    /* trcptr must be an inside solid */
			    Addsub(ptr2, trcptr);
			    trcptr->op = 1;
			}
		    } else if (ht2 < ht1 && ht2 > hb1) {
			/* These TRC's overlap */
			/* Calculate radius at ht2 */
			rtmp = trcptr->r1 + (trcptr->r2 - trcptr->r1)*(ht2-hb1)/(ht1-hb1);
			if (rtmp > ptr2->r2) {
			    /* ptr2 must be an inside solid, so subtract it */
			    Addsub(trcptr, ptr2);
			    ptr2->op = 1;
			} else if (rtmp < ptr2->r1) {
			    /* trcptr must be an inside solid */
			    Addsub(ptr2, trcptr);
			    trcptr->op = 1;
			}
		    }
		    ptr2 = ptr2->next;
		}
	    }
	    trcptr = trcptr->next;
	}
    }

    if (fract < 1.0) {
	/* Must calculate a cutting solid */
	vect_t pdir = VINIT_ZERO;
	vect_t enddir = VINIT_ZERO;
	vect_t startdir = VINIT_ZERO;
	fastf_t len = 0.0;
	fastf_t theta = 0.0;
	point_t pts[8];

	/* Calculate direction from axis to curve */
	len = 0.0;
	ptr = curv_pts;
	while (ptr && ZERO(len)) {
	    VSUB2(pdir, ptr->pt, pt);
	    VJOIN1(startdir, pdir, -VDOT(pdir, adir), adir);
	    len = MAGNITUDE(startdir);
	    ptr = ptr->next;
	}
	VUNITIZE(startdir);

	/* Calculate direction towards solid from axis */
	VCROSS(pdir, adir, startdir);
	VUNITIZE(pdir);

	if (fract < 0.5) {
	    theta = M_2PI*fract;
	    cutop = Intersect;
	} else if (fract > 0.5) {
	    theta = (-M_2PI*(1.0-fract));
	    cutop = Subtract;
	} else {
	    /* FIXME: fract == 0.5, a dangerous comparison (roundoff) */
	    theta = M_PI;
	    cutop = Intersect;
	    /* Construct vertices for cutting solid */
	    VJOIN2(pts[0], pt, hmin, adir, rmax, startdir);
	    VJOIN1(pts[1], pts[0], (-2.0*rmax), startdir);
	    VJOIN1(pts[2], pts[1], rmax, pdir);
	    VJOIN1(pts[3], pts[0], rmax, pdir);
	    for (i = 0; i < 4; i++) {
		VJOIN1(pts[i+4], pts[i], (hmax-hmin), adir);
	    }
	}
	if (!ZERO(fract - 0.5)) {
	    /* Calculate direction to end of revolve */
	    VSCALE(enddir, startdir, cos(theta));
	    VJOIN1(enddir, enddir, sin(theta), pdir);
	    VUNITIZE(enddir);

	    /* Calculate required length of a side */
	    len = rmax/cos(theta/4.0);

	    /* Construct vertices for cutting solid */
	    /* Point at bottom center of revolution */
	    VJOIN1(pts[0], pt, hmin, adir);
	    /* Point at bottom on curve */
	    VJOIN1(pts[1], pts[0], len, startdir);
	    /* Point at bottom at end of revolution */
	    VJOIN1(pts[3], pts[0], len, enddir);
	    /* Calculate direction to pts[2] */
	    VADD2(enddir, enddir, startdir);
	    VUNITIZE(enddir);
	    /* Calculate pts[2] */
	    VJOIN1(pts[2], pts[0], len, enddir);

	    /* Calculate top vertices */
	    for (i = 0; i < 4; i++) {
		VJOIN1(pts[i+4], pts[i], (hmax-hmin), adir);
	    }
	}

	/* Make the BRL-CAD solid */
	if (mk_arb8(fdout, bu_vls_cstr(&cutname), &pts[0][X]) < 0) {
	    bu_log("Unable to write ARB8 for entity D%07d (%s)\n" ,
		   dir[entityno]->direct, dir[entityno]->name);
	    return 0;
	}
    }

    /* Build region */
    trcptr = trcs;
    while (trcptr != NULL) {
	/* Union together all the TRC's that are not subtracts */
	if (trcptr->op != 1) {
	    (void)mk_addmember(bu_vls_cstr(&trcptr->name), &head.l, NULL, operators[Union]);

	    if (fract < 1.0) {
		/* include cutting solid */
		(void)mk_addmember(bu_vls_cstr(&cutname), &head.l, NULL, operators[cutop]);
	    }

	    subp = trcptr->subtr;
	    /* Subtract the inside TRC's */
	    while (subp != NULL) {
		(void)mk_addmember(bu_vls_cstr(&subp->name), &head.l, NULL, operators[Subtract]);
		subp = subp->next;
	    }
	}
	trcptr = trcptr->next;
    }

    /* Make the object */
    if (mk_lcomb(fdout, dir[entityno]->name, &head, 0 ,
		 (char *)0, (char *)0, (unsigned char *)0, 0) < 0) {
	bu_log("Unable to make combination for entity D%07d (%s)\n" ,
	       dir[entityno]->direct, dir[entityno]->name);
	return 0;
    }


    /* Free the TRC structures */
    trcptr = trcs;
    while (trcptr != NULL) {
	trcptr_tmp = trcptr->next;
	bu_vls_free(&trcptr->name);
	bu_free((char *)trcptr, "Revolve: trcptr");
	trcptr = trcptr_tmp;
    }
    return 1;
}


/* Routine to add a name to the list of subtractions */
void
Addsub(struct trclist *trc, struct trclist *ptr)
{
    struct subtracts *subp;

    if (trc->subtr == NULL) {
	BU_ALLOC(trc->subtr, struct subtracts);
	subp = trc->subtr;
    } else {
	subp = trc->subtr;
	while (subp->next != NULL)
	    subp = subp->next;
	BU_ALLOC(subp->next, struct subtracts);
	subp = subp->next;
    }

    subp->next = NULL;
    subp->name = ptr->name; /* struct copy */
    subp->index = ptr->index;
}


/*
 * Local Variables:
 * mode: C
 * tab-width: 8
 * indent-tabs-mode: t
 * c-file-style: "stroustrup"
 * End:
 * ex: shiftwidth=4 tabstop=8
 */
