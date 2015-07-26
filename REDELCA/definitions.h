 /* Copyright (c) 2015, Computer Science Department (CSD), University of Crete, and 
 * Signal Processing Lab (SPL), Institute of Computer Science (ICS), FORTH, Greece.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of CSD, SPL/ICS-FORTH nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 /**
  * \file
 
 *The header file for calculating a Delaunay triangulation. To use in conjuction with redelca-main.c
 * 
 * \author
 * Author: Phivos Phivou, Computer Science Department, University of Crete.
 *
 * email: fivou@csd.uoc.gr, apanouso@ics.forth.gr
 *
 *
 * This code has been writted for contiki-2.7 distribution.
*/


#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define DEBUG 1

#include <stdio.h>
#include <stdlib.h>

typedef struct _Triangle Triangle;
typedef struct _Point Point;

struct _Triangle{
	Triangle * next;
	Point *points[3];
};

struct _Point{
	float x, y, rssi;
	short id;
	unsigned char p;
	Point * next, ** neighbors;
};

#define MAXPOINTS 30

#define NEXT(_x) ((_x+1)%3)
#define PREV(_x) ((_x+2)%3)

#define P0 T->points[0]
#define P1 T->points[1]
#define P2 T->points[2]

#define A0 A->points[0]
#define A1 A->points[1]
#define A2 A->points[2]

#define NEWT (Triangle*) malloc(sizeof(Triangle));
#define NEWP (Point*) malloc(sizeof(Point));

/**************Angle orientation*/
#define CCW(_a, _b, _c) (_b->x - _a->x) * (_c->y - _a->y) - (_b->y - _a->y) * (_c->x - _a->x)
/*Verticality between two points*/
#define VERTICAL(_a, _b) (_a->x == _b->x)
/******************Point on edge*/
#define ON_EDGE(_a, _b, _p) (_p->x <= (_a->x > _b->x ? _a->x : _b->x) && _p->x >= (_a->x < _b->x ? _a->x : _b->x) && _p->y <= (_a->y > _b->y ? _a->y : _b->y) && _p->y >= (_a->y < _b->y ? _a->y : _b->y))
/**************Point in triangle*/
#define IN_TRIA(_t, _p) (_t->points[0] == _p || _t->points[1] == _p || _t->points[2] == _p)
/**Distance^2 between two points*/
#define DISQR(_p, _c) ((_c->y - _p->y) * (_c->y - _p->y) + (_c->x - _p->x) * (_c->x - _p->x))
/***********Absolute of a number*/
#define ABS(_a) (_a < 0 ? _a - _a * 2 : _a)
/*********Find adjacent triangle*/
#define ADJACENT(_T, _a, _b, _t) for(_t=tHead==_T?_T->next:tHead; _t && !(IN_TRIA(_t, _a) && IN_TRIA(_t, _b)); _t=(_t->next == _T ? _T->next:_t->next));

/**********Functions definitions*/
short int	legalizeEdge	(Point * p, Triangle * T);
short int	pointInTriangle	(Point * p, Triangle * T);
short int	delaunay	();
short int	swap		(Triangle * a, Triangle * b);
void		addPoint	(Point * p);
Point *		calcCenter	(Triangle * T);

/******************Lists heads*/
Point * pHead;
Triangle * tHead;


/*REDELCA specific declarations*/
short int mem;
void	addREDELCApoint(Point * p);
float	str2float(char * str);

#endif

/*DELAUNAY Module return codes
Code	Meaning
-100	Less than three points for triangulation - Triangulation cannot be achieved
-200	A point lies outside any triangle
-300	Point lies on an external edge
-400	The point lies on an edge of a triangle which does not contain edges' points (VERY BAD CODE - NORMALY IT WILL NEVER OCCOUR!)
-500	Legalization of the triangles -when point is on an edge- failed
	-510 Fail of T (triangle that the point is on one of its edges)
	-520 Fail of A (adjdacent triangle of T which shares the edge that the point lies on)
	-530 Fail of B (newly created triangle)
	-540 Fail of C (---------//-----------)

*/
