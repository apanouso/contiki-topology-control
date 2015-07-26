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
 * 3. Neither the name of the CSD, SPL/ICS-FORTH nor the names of its
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
 * The main process for topology control calculation based on the REDELCA scheme
 *
 * \author
 * Phivos Phivou, Computer Science Department, University of Crete.
 *
 * email: fivou@csd.uoc.gr, apanouso@ics.forth.gr
 *
 *
 * This code has been writted for contiki-2.7 distribution.
*/




#include <stdio.h>
#include "contiki.h"
#include "cc2420.h"
#include "net/rime.h"
#include "net/packetbuf.h"
#include "dev/serial-line.h"
#include "random.h"
#include "definitions.h" //ALL DELAUNAY-related CALCULATIONS.

#define TIMEFRAME 30

//reference to the transmission power for cooja UDGM, using 15m as transmission range.
static const
float POWER [8] = {
    {2.107}  ,
    {11.472} ,
    {28.330} ,
    {52.680} ,
    {84.520} ,
    {123.850},
    {170.680},
    {225.000}
};

static Point * point;
static char veri[256];
static char buff [5];
static uint8_t numofneighs;
/*---------------------------------------------------------------------------*/
PROCESS(b_thread, "Broadcast thread");
PROCESS(r_thread, "REDELCA thread");
AUTOSTART_PROCESSES(&b_thread, &r_thread);
/*---------------------------------------------------------------------------*/
static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  char * msg = strdup((char *)packetbuf_dataptr()), * x, * y;
 
  if(msg[0] == 'N'){
    printf("NEIGHBOR %d\n", from->u8[0]);
    return;
  }
  numofneighs++;
  point = (Point *) malloc(sizeof(Point));
  x = strtok(msg, "#");
  y = strtok(NULL, "#");
  point->id = from->u8[0];
  point->x = str2float(x);
  point->y = str2float(y);
  free(x);
  free(y);
  free(msg);
  //consider points for delaunay graph calculation.
  point->rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI) - 45;
  addREDELCApoint(point);
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;

static process_event_t redelca_start_event;
static process_event_t redelca_finish_event;

/*MAIN THREAD------------------------------------------------------------------------------*/
PROCESS_THREAD(b_thread, ev, data)
{
  static struct etimer send_timer, redelca_timer;
  
  static uint32_t start_energy_cpu, start_energy_rx, start_energy_tx;
  static uint32_t end_energy_cpu, end_energy_rx, end_energy_tx;
  static char * msg, * my_x, * my_y, * position;
  
  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
  PROCESS_BEGIN();
  broadcast_open(&broadcast, 129, &broadcast_call);
  /*Wait for the position from Cooja script*/
  PROCESS_YIELD_UNTIL(ev == serial_line_event_message);
  /*Process position string and add to points*/
  msg = strdup((char*) data);
  position = strdup((char*) data);
  my_x = strtok(msg, "#");
  my_y = strtok(NULL, "#");

  point = (Point*) malloc(sizeof(Point));
  point->id = rimeaddr_node_addr.u8[0];
  point->x = str2float(my_x);
  point->y = str2float(my_y);
  addREDELCApoint(point);
  free(my_x);
  free(my_y);
  free(msg);
  /*Set timers redelca timer to 120s and broadcast timer*/
  etimer_set(&redelca_timer,  120*CLOCK_SECOND);
  etimer_set(&send_timer,     ((10 + rimeaddr_node_addr.u8[0] % TIMEFRAME) * CLOCK_SECOND));
 
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
  /*Prepare broadcast buffer and broadcast message*/
  numofneighs = 0;
  
  packetbuf_clear();
  packetbuf_clear_hdr();
  packetbuf_copyfrom(position, strlen(position));
  broadcast_send(&broadcast);
  free(position);
  position = 0;
  
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&redelca_timer));
  
  /*Switch control to REDELCA thread*/
  start_energy_rx = energest_type_time(ENERGEST_TYPE_LISTEN);
  start_energy_tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  start_energy_cpu = energest_type_time(ENERGEST_TYPE_CPU);
  process_post(&r_thread, redelca_start_event, NULL);
  /*Wait for the control to switch from the REDELCA thread*/
  PROCESS_YIELD_UNTIL(ev == redelca_finish_event);
  end_energy_cpu = energest_type_time(ENERGEST_TYPE_CPU);
  /*Set timers redelca timer to 60s and broadcast timer*/
  etimer_set(&redelca_timer,  7680);
  etimer_set(&send_timer,     ((10 + rimeaddr_node_addr.u8[0] % TIMEFRAME) * CLOCK_SECOND));
  //etimer_set(&send_timer,     ((random_rand() % TIMEFRAME) * CLOCK_SECOND));
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
  /*Prepare broadcast buffer and broadcast message*/
  packetbuf_clear();
  position = strdup("N\0");
  packetbuf_attr_clear();
  packetbuf_copyfrom(position, strlen(position));
  broadcast_send(&broadcast);
  free(position);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&redelca_timer));
  
  //calculate the timings per component (to use for power consumption).
  end_energy_rx = energest_type_time(ENERGEST_TYPE_LISTEN);
  end_energy_tx = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  
  printf("ENERGY_CPU %lu\n", end_energy_cpu-start_energy_cpu);
  printf("ENERGY_TX %lu\n", end_energy_tx-start_energy_tx);
  printf("ENERGY_RX %lu\n", end_energy_rx-start_energy_rx);
  printf("DONE\n");
  PROCESS_END();
}

/*REDELCA THREAD---------------------------------------------------------------------------*/
PROCESS_THREAD(r_thread, ev, data)
{
  static struct etimer et;
  static Triangle * T;
  static Point * p1;
  static short i, j, k, maxn, l;
  static float a, b, dis;
  static uint16_t maxp;
  PROCESS_BEGIN();
  /*Wait until the control switches from the main thread*/
  PROCESS_YIELD_UNTIL(ev == redelca_start_event);
  
  printf("DELAUNAY\n");
  /*Triangulize received points*/
  i = delaunay();
  i = (i== -100) ? 2:i;
   
  printf("INITIALIZE %d\n", i-1);

  for(maxn=0, T=tHead; T; maxn+=IN_TRIA(T, pHead)?1:0, T=T->next)
        T->points[0]->neighbors = T->points[1]->neighbors = T->points[2]->neighbors = 0;
  maxn++;
  pHead->neighbors = (Point**) malloc(maxn * sizeof(Point*));

  for(j=0, T=tHead; T; T=tHead){
    /*Find pHead in triangle*/
    for(i=0; i<3 && T->points[i] != pHead; ++i);
    /*If pHead not in triangle just free the triangle*/
    if(i==3 || T->points[0]->id<0 || T->points[1]->id<0 || T->points[2]->id<0) goto freeTriangle;
    /*Find NEXT in pHeads' neighbors*/
    if(T->points[NEXT(i)]->neighbors){
      /*NEXT is a neighbor, add PREV as its neighbor*/
      T->points[NEXT(i)]->neighbors[1] = T->points[PREV(i)];
    }
    else {
      /*NEXT is not a neighbor, remove from points list, add to neighbors list, initialize its neighbors list*/
      for(p1=pHead; p1 && p1->next!=T->points[NEXT(i)]; p1=p1->next);
      p1->next = p1->next->next;
      pHead->neighbors[j++] = T->points[NEXT(i)];
      T->points[NEXT(i)]->neighbors = (Point **) malloc(2 * sizeof(Point*));
      T->points[NEXT(i)]->neighbors[0] = T->points[PREV(i)];
      T->points[NEXT(i)]->neighbors[1] = 0;
    }
    /*Find PREV in pHeads' neighbors*/
    if(T->points[PREV(i)]->neighbors){
      /*NEXT is a neighbor, add PREV as its neighbor*/
      T->points[PREV(i)]->neighbors[1] = T->points[NEXT(i)];
    }
    else {
      /*NEXT is not a neighbor, remove from points list, add to neighbors list, initialize its neighbors list*/
      for(p1=pHead; p1 && p1->next!=T->points[PREV(i)]; p1=p1->next);
      p1->next = p1->next->next;
      pHead->neighbors[j++] = T->points[PREV(i)];
      T->points[PREV(i)]->neighbors = (Point **) malloc(2 * sizeof(Point*));
      T->points[PREV(i)]->neighbors[0] = T->points[NEXT(i)];
      T->points[PREV(i)]->neighbors[1] = 0;
    }
    /*Free the triangle from memory*/
    freeTriangle:
    tHead=tHead->next;
    free(T);
  }
  /*Free unused points*/
  for(p1=pHead->next; p1; p1=pHead->next){
    pHead->next=p1->next;
    free(p1);
  }
  pHead->next = 0;
  pHead->p = 0;
  /*Adjust neighbors number*/
  maxn=j;
  
  printf("REDELCA %d\n", maxn);
  maxp = 31;
    if (maxn >0){
  maxp = 0;

  for(i=0; i<maxn; ++i){
	  

    /*Initialize neighbors values*/
    for(k=0; k<maxn; ++k){
      dis = DISQR(pHead, pHead->neighbors[k]);
      a = POWER[7] - dis;
      j = 7;
      for (l=0;l<7;l++){
		  b = POWER[l]-dis;
		 j = (b >0 && a > b) ? l:j;
		 a = (b >0 && a > b) ? b:a;  
		  }
   
      pHead->neighbors[k]->p = 3 + j * 4;
      pHead->neighbors[k]->next = 0;
    }
    /*Find least power expensive neighbor*/
    for(j=0, k=1; k<maxn && pHead->neighbors[j] != pHead->neighbors[i]; j = (pHead->neighbors[j]->p > pHead->neighbors[k]->p ? k : j), k++);

    p1 = pHead->neighbors[j]->p == pHead->neighbors[i]->p ? pHead->neighbors[i] : pHead->neighbors[j];
    p1->next = pHead;

    /*Until i-th neighbor is not found keep searching*/
    while(p1->id != pHead->neighbors[i]->id){
      /*If neighbor 0 of p1 exists and hasnt yet been visited estimate transmition power else set power to 500 (just a big value)*/
      if(p1->neighbors[0] && !p1->neighbors[0]->next){
        dis = DISQR(p1, p1->neighbors[0]);
    
        a = POWER[7] - dis;
      j = 7;
      for (l=0;l<7;l++){
		  b = POWER[l]-dis;
		 j = (b >0 && a > b) ? l:j;
		 a = (b >0 && a > b) ? b:a;  
		  }
        j = 3 + j * 4;
      } else {j=500;}
      /*If neighbor 1 of p1 exists and hasnt yet been visited estimate transmition power else set power to 500 (just a big value)*/
      if(p1->neighbors[1] && !p1->neighbors[1]->next){
        dis = DISQR(p1, p1->neighbors[1]);
      
        a = POWER[7] - dis;
      k = 7;
      for (l=0;l<7;l++){
		  b = POWER[l]-dis;
		 k = (b >0 && a > b) ? l:k;
		 a = (b >0 && a > b) ? b:a;  
		  }
        k = 3 + k * 4;
      } else {k=500;}
      
      /*Well this is the case where the shortest path leads to nowhere!*/
      if(j==500 && k==500 && !pHead->next){
		 //  printf("NEIGHBOR: **%d\n", pHead->neighbors[i]->id);
        break;
      }
      /*This is a case where the path somewhere has taken a wrong turn, this is the correcting action*/
      if(j==500 && k==500) {
        p1 = pHead->next;
        pHead->next = 0;
        dis = DISQR(p1, p1->neighbors[0]);
       
        a = POWER[7] - dis;
      j = 7;
      for (l=0;l<7;l++){
		  b = POWER[l]-dis;
		 j = (b >0 && a > b) ? l:j;
		 a = (b >0 && a > b) ? b:a;  
		  }
        p1->neighbors[0]->p = 3 + j * 4;
        p1->neighbors[0]->next = p1;
        p1 = p1->neighbors[0];
        continue;
      }
      /*There is a chance the proper neighbor isnt selected, this is done to continue from here*/
      if(j == k){pHead->next = p1;}
      /*Set future p1's next to current p1 power needed to reach p1*/
      p1->neighbors[(j<k ? 0 : 1)]->next = p1;
      p1->neighbors[(j<k ? 0 : 1)]->p = j<k ? j : k;

      /*Continue to the least power expensive neighbor*/
      p1 = p1->neighbors[(j<k ? 0 : 1)];
    }
    pHead->next = 0;
    if(j==500 && k==500){
      //printf("neighbor not found in shortest path!\n");
      //  printf("NEIGHBOR: ****%d\n", pHead->neighbors[i]->id);
    }
    else {
      j=0;
      while(p1){
       j += p1->p;
       p1 = p1->next;
      }
    }
    dis = DISQR(pHead, pHead->neighbors[i]);
 
    a = POWER[7] - dis;
      k = 7;
      for (l=0;l<7;l++){
		  b = POWER[l]-dis;
		 k = (b >0 && a > b) ? l:k;
		 a = (b >0 && a > b) ? b:a;  
		  }
    
    k = 3 + k * 4;
  
    /*Select the optimal transimtion power for i-th neighbor*/
    k = k<j?k:j;
    maxp = maxp>k ? maxp : k;
  }
	if (maxp >0){
		cc2420_set_txpower(maxp);
	}
	else {maxp = 31;} 
  }
  printf("POWER %d\n", maxp);
  printf("MEMORY %d\n", mem);
  
  process_post(&b_thread, redelca_finish_event, NULL);
  PROCESS_END();
}
