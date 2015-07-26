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
 * The c functions for calculating a Delaunay triangulation. To use in conjuction with redelca-main.c
 *
 * \author
 * Phivos Phivou, Computer Science Department, University of Crete.
 *
 * email: fivou@csd.uoc.gr, apanouso@ics.forth.gr
 *
 * How to cite this code: 
 * P. Phivou, A. Panousopoulou, and P. Tsakalides, ``On Realizing Distributed Topology Control in Low-power IoT Platforms'', submitted for puiblication at IEEE WF-IoT, 2015.
 *
 * 
 * This code has been writted for contiki-2.7 distribution.
*/



#include "definitions.h"
#include <string.h>
short int legalizeEdge(Point * p, Triangle * T){
	short int ut, ua;
	Triangle * A;
	Point * center;

	for(ut=0; ut<3 && T->points[ut] != p; ++ut);
	if(ut==3) return -1;

	ADJACENT(T, T->points[NEXT(ut)], T->points[PREV(ut)], A);

	if(!A) return 1;

	for(ua=0; ua<3 && IN_TRIA(T, A->points[ua]); ua++);

	center = calcCenter(T);
	if(!center) return -2;

	if(DISQR(A->points[ua], center) >= DISQR(center, P0)){
		free(center);
		return 1;
	}
	free(center);

	if((ua=swap(T, A))<0) return -3;

	if((ua=legalizeEdge(p, T))<0) return -ua;
	if((ua=legalizeEdge(p, A))<0) return -ua;

	return 1;
}

short int pointInTriangle (Point * p, Triangle * T){
	float a, b, c;
	a = ((P1->y - P2->y) * (p->x-P2->x) + (P2->x - P1->x) * (p->y - P2->y)) /
			((P1->y - P2->y) * (P0->x - P2->x) + (P2->x - P1->x) * (P0->y - P2->y));
	b = ((P2->y - P0->y) * (p->x-P2->x) + (P0->x - P2->x) * (p->y - P2->y)) /
			((P1->y - P2->y) * (P0->x - P2->x) + (P2->x - P1->x) * (P0->y - P2->y));
	c = 1.0 - a - b;
	if 	(CCW(P0, p, P1) == 0 && ON_EDGE(P0, P1, p))
	{return 0;}
	else if (CCW(P1, p, P2) == 0 && ON_EDGE(P1, P2, p))
	{return 1;}
	else if (CCW(P2, p, P0) == 0 && ON_EDGE(P2, P0, p))
	{return 2;}
	else if (a > 0 && b > 0 && c > 0)
	{return -1;}
	else
	{return -2;}

}
short int delaunay(){
	Point * point, * p;
	Triangle * T, * A, * B, * C;
	int i, e, m;
	float a, b;
	short int len;

	for(e=0, p=pHead; p; ++e, p=p->next){
		if(ABS(p->x) > ABS(p->y)){
			i = ABS(p->x) > i ? ABS(p->x) : i;
		}
		else{
			i = ABS(p->y) > i ? ABS(p->y) : i;
		}
	}
	len = e;
	if(e==2) return -100;

	T = tHead = NEWT;
	T->next = NULL;

	P0 = NEWP;
	P0->id = -1;
	P0->x = i * 3 + 1;
	P0->y = 0;
	P0->next = pHead;
	pHead = P0;

	P1 = NEWP;
	P1->id = -2;
	P1->x = 0;
	P1->y = i * 3 + 1;
	P1->next = pHead;
	pHead = P1;

	P2 = NEWP;
	P2->id = -3;
	P2->x = P2->y = -(3 * i) - 1;
	P2->next = pHead;
	pHead = P2;

	for(point = pHead->next->next->next; point; point = point->next){
		for(T=tHead; T; T = T->next){
			e = pointInTriangle(point, T);
			switch(e){
				case -2: continue;
				case -1: goto pit;
				default: goto poe;
			}
		}
		return -200;
		poe:
		ADJACENT(T, T->points[e], T->points[NEXT(e)], A)
		if(!A) return -300;

		for(m=0; m<3 && A->points[m] != T->points[e]; ++m);
		if(m==3) return -400;

		B = NEWT;
		B->next = tHead;
		tHead = B;
		C = NEWT;
		C->next = tHead;
		tHead = C;

		B->points[0] = C->points[0] = T->points[e];
		B->points[1] = T->points[PREV(e)];
		C->points[1] = A->points[NEXT(m)] == T->points[NEXT(e)] ? A->points[PREV(m)] : A->points[NEXT(m)];
		T->points[e] = A->points[m] = B->points[2] = C->points[2] = point;

		if((i=legalizeEdge(point, T))<0) return i-510;
		if((i=legalizeEdge(point, A))<0) return i-520;
		if((i=legalizeEdge(point, B))<0) return i-530;
		if((i=legalizeEdge(point, C))<0) return i-540;

		continue;

		pit:

		A = NEWT;
		B = NEWT;
		A->next=tHead;
		tHead = A;
		B->next = tHead;
		tHead=B;

		A0 = P1;
		A1 = B->points[0] = P2;
		B->points[1] = P0;
		P2 = A2 = B->points[2] = point;

		if((i=legalizeEdge(point, T))<0) return i-610;
		if((i=legalizeEdge(point, A))<0) return i-620;
		if((i=legalizeEdge(point, B))<0) return i-630;


	}

	/*Calculate peak memory allocation*/
	mem=0;
	for(T=tHead, i=0; T; T=T->next, ++i);
	for(point=pHead, e=0; point; point=point->next, ++e);
	mem = i*sizeof(Triangle) + e*sizeof(Point);

	/*Patch*/
	for(T=tHead; T; T = T->next){
		/*Find # of external points*/
		for(i=0, e=0; i<3; e+=(T->points[i]->id < 0 ? 1 : 0), ++i);

		/*If # of external points is not equal to 1 continue*/
		if(e != 1) continue;

		check:

		/*Find external point*/
		for(e=0; e<3 && T->points[e]->id > -1; e++);

		/*Next mutual case*/
		/*Find the adjacent triangle*/
		ADJACENT(T, T->points[e], T->points[NEXT(e)], A)
		if(!A) return -710;

		/*Find the # of external points in the adjacent triangle*/
		for(i=0, m=0; i<3; m+=(A->points[i]->id < 0 ? 1 : 0), ++i);

		if(m==1){
			/*Find adjacent triangles external point*/
			for(m=0; m<3 && A->points[m]->id > -1; m++);
			/*Check whether the convex hull criteria apply*/
			a = CCW(T->points[PREV(e)], T->points[e],	A->points[T->points[NEXT(e)] == A->points[NEXT(m)] ? PREV(m) : NEXT(m)]);
			b = CCW(T->points[PREV(e)], T->points[NEXT(e)], A->points[T->points[NEXT(e)] == A->points[NEXT(m)] ? PREV(m) : NEXT(m)]);
			a*=b;
			if(a<0){
				if((i=swap(T, A))<0) return -711;
				T = P0->id<0 || P1->id<0 || P2->id<0 ? T : A;
				goto check;
			}
		}

		/*Find external point*/
		for(e=0; e<3 && T->points[e]->id > -1; e++);

		/*Prev mutual case*/
		/*Find the adjacent triangle*/
		ADJACENT(T, T->points[e], T->points[PREV(e)], A)
		if(!A) return -720;

		/*Find the external point in the adjacent triangle*/
		for(i=0, m=0; i<3; m+=(A->points[i]->id < 0 ? 1 : 0), ++i);

		if(m==1){
			/*Find adjacent triangles external point*/
			for(m=0; m<3 && A->points[m]->id > -1; m++);
			/*Check whether the convex hull criteria apply*/
			a = CCW(T->points[NEXT(e)], T->points[e],       A->points[T->points[NEXT(e)] == A->points[NEXT(m)] ? NEXT(m) : PREV(m)]);
			b = CCW(T->points[NEXT(e)], T->points[PREV(e)], A->points[T->points[NEXT(e)] == A->points[NEXT(m)] ? NEXT(m) : PREV(m)]);
			a*=b;
			if(a<0){
				if((i=swap(T, A))<0)return -721;;
				T = P0->id<0 || P1->id<0 || P2->id<0 ? T : A;
				goto check;
			}
		}

	}

	for(T=tHead, A=NULL; T; A=T, T=(A==NULL?T:T->next)){
		for(i=0, e=0; i<3; e+=(T->points[i]->id<0?1:0), ++i);
		if(e==0) continue;
		if(!A){
			A=T;
			T=T->next;
			free(A);
			A=NULL;
			tHead=T;
		} else {
			A->next = T->next;
			free(T);
			T=A;
		}
	}

	for(i=0; i<3; ++i){
		point=pHead;
		pHead=pHead->next;
		free(point);
	}

	//return 1;
	return len;
}

void addPoint (Point * p){
	p->next = pHead;
	pHead = p;
}

void addREDELCApoint(Point * P){
	Point * t, * p;
	if(!pHead){
		pHead = P;
		P->next = NULL;
		return;
	}
	for(t=pHead, p=pHead->next; p && p->rssi < P->rssi; t=p, p=p->next);

	P->next=p;
	t->next=P;

	if(++mem < MAXPOINTS)
		return;

	mem--;
	p=P;
	while(p->next){
		t=p;
		p=p->next;
	}
	t->next=NULL;
	free(p);
}

Point * calcCenter (Triangle * T){
	float m21,m32, m13;
	Point * center;

	m21 = (P1->y - P0->y) / (P1->x - P0->x);
	m32 = (P2->y - P1->y) / (P2->x - P1->x);
	m13 = (P0->y - P2->y) / (P0->x - P2->x);

	if(CCW(P0, P1, P2) == 0){
		return NULL;
	}
	center = (Point*) malloc(sizeof(Point));
	if(!center) return NULL;

	if(!VERTICAL(P0, P1) && !VERTICAL(P1, P2) && m21!=m32){//p2 common point
		center->x = (m21 * m32 * (P2->y - P0->y) + m21 * (P1->x + P2->x) - m32 * (P0->x + P1->x)) / (2 * (m21- m32));
		center->y = (-(1/(m21?m21:m32))) * (center->x - ((m21?P0->x:P2->x) + P1->x)/2) + ((m21?P0->y:P2->y) + P1->y) / 2;
	}
	else if(!VERTICAL(P0, P2) && !VERTICAL(P1, P2) && m32!=m13){//p3 common point
		center->x = (m32 * m13 * (P0->y - P1->y) + m32 * (P2->x + P0->x) - m13 * (P1->x + P2->x)) / (2 * (m32- m13));
		center->y = (-(1/(m32?m32:m13))) * (center->x - ((m32?P1->x:P0->x) + P2->x)/2) + ((m32?P1->y:P0->y) + P2->y) / 2;
	}
	else{
		center->x = (m13 * m21 * (P1->y - P2->y) + m13 * (P0->x + P1->x) - m21 * (P2->x + P0->x)) / (2 * (m13- m21));
		center->y = (-(1/(m13?m13:m21))) * (center->x - ((m13?P2->x:P1->x) + P0->x)/2) + ((m13?P2->y:P1->y) + P0->y) / 2;
	}
	return center;
}

short int swap(Triangle * a, Triangle * b){
	int ua, ub;
	for(ua=0; ua<3 && IN_TRIA(b, a->points[ua]); ++ua);
	for(ub=0; ub<3 && IN_TRIA(a, b->points[ub]); ++ub);
	if(ua==3 || ub==3) return -1;
	a->points[NEXT(ua)] = b->points[ub];
	b->points[b->points[NEXT(ub)] == a->points[PREV(ua)] ? NEXT(ub) : PREV(ub)] = a->points[ua];
	return 1;
}

float str2float(char * str)
{
	float r, num;
	char * s = strdup(str);
	char * a =(char *) strtok(s, "."), b[3];
	num = atoi(a);
	r = num;
	a = (char *) strtok(NULL, ".");
	b[0] = a[0];
	b[1] = a[1];
	b[2] = a[2];
	num = atoi(b);
	r += (r<0 ? - (num / 1000) : (num / 1000));
	return r;
}
