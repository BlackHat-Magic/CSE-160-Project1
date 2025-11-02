/*
 * ANDES Lab - University of California, Merced
 * This class provides the basic functions of a network node.
 *
 * @author UCM ANDES Lab
 * @date   2013/09/03
 *
 */
#include <string.h>
#include <AM.h>
#include <Timer.h>
#include "includes/command.h"
#include "includes/packet.h"
#include "includes/CommandMsg.h"
#include "includes/sendInfo.h"
#include "includes/channels.h"
// #include "dataStructures/interfaces/Hashmap.nc"

module Node{
   uses interface Boot;

   uses interface SplitControl as AMControl;
   uses interface Receive;

   uses interface SimpleSend as Sender;

   uses interface CommandHandler;

   // new interfaces
   uses interface AMPacket;
   uses interface Timer<TMilli> as periodicTimer;

   // key: src    value: last seq
   uses interface Hashmap<uint16_t> as SeqMap;
}

implementation{
   pack sendPackage;
   uint16_t nextSeq = 1;

   enum { MAX_NEIGHBORS = 32, ND_MISS_THRESHOLD = 5 };
   typedef struct neighbor_entry {
      uint16_t addr;   // neighbor node id
      uint8_t  misses; // consecutive ND periods with no reply
   } neighbor_entry_t;
   neighbor_entry_t neighbors[MAX_NEIGHBORS];

   enum { MAX_NODES = 32, MAX_DEGREE = 8, INF_COST = 0x3fff };
   uint16_t lsaMySeq = 1;                  // my LSA seq
   uint16_t lsaLastSeq[MAX_NODES + 1];     // last LSA seq seen per origin
   uint8_t  lsCount[MAX_NODES + 1];
   uint16_t lsNbr[MAX_NODES + 1][MAX_DEGREE];
   uint8_t  lsCost[MAX_NODES + 1][MAX_DEGREE];
   uint16_t routeNext[MAX_NODES + 1];

   bool hasEdge(uint16_t u, uint16_t v, uint8_t *costOut) {
      uint8_t i, j;
      // find v in u’s list
      for (i = 0; i < lsCount[u]; i++) {
         if (lsNbr[u][i] == v) {
            // find u in v’s list (symmetric requirement)
            for (j = 0; j < lsCount[v]; j++) {
               if (lsNbr[v][j] == u) {
                  if (costOut) *costOut = lsCost[u][i]; // choose u’s advertised cost
                  return TRUE;
               }
            }
         }
      }
      return FALSE;
   }

   void recomputeRoutes() {
      uint16_t my = TOS_NODE_ID;
      uint16_t dist[MAX_NODES + 1];
      uint16_t firstHop[MAX_NODES + 1];
      bool     vis[MAX_NODES + 1];
      uint16_t i, u, v, best, bestd;
      uint8_t  c;

      for (i = 0; i <= MAX_NODES; i++) {
         dist[i] = INF_COST; firstHop[i] = 0; vis[i] = FALSE;
      }
      dist[my] = 0; firstHop[my] = my;

   // Simple O(N^2 + E) Dijkstra using LSDB neighbors
      for (;;) {
         best = 0; bestd = INF_COST;
         for (i = 1; i <= MAX_NODES; i++) {
            if (!vis[i] && dist[i] < bestd) { bestd = dist[i]; best = i; }
         }
         if (best == 0 || bestd == INF_COST) break;
         u = best; vis[u] = TRUE;

         // relax neighbors of u
         for (i = 0; i < lsCount[u]; i++) {
            v = lsNbr[u][i];
            if (!hasEdge(u, v, &c)) continue; // only symmetric links
            if (dist[u] + c < dist[v]) {
            dist[v] = dist[u] + c;
            firstHop[v] = (u == my) ? v : firstHop[u];
            }
         }
      }

      for (i = 1; i <= MAX_NODES; i++) {
         routeNext[i] = firstHop[i];
         routeCost[i] = dist[i];
      }

      dbg(ROUTING_CHANNEL, "RT: recomputed (me=%u)\n", my);
   }

   void sendLSA() {
      pack p;
      uint8_t i, count = 0, maxEntries = (PACKET_MAX_PAYLOAD_SIZE - 3) / 3;
      nx_uint8_t *pl = p.payload;

      p.src = TOS_NODE_ID;
      p.dest = AM_BROADCAST_ADDR;
      p.seq = nextSeq++;              // keep global monotonic seq
      p.TTL = MAX_TTL;
      p.protocol = PROTOCOL_LINKSTATE;

      // header: seq (2 bytes), count (1 byte)
      pl[0] = (lsaMySeq >> 8) & 0xff;
      pl[1] = (lsaMySeq     ) & 0xff;
      pl[2] = 0; // fill later
      for (i = 0; i < MAX_NEIGHBORS && count < maxEntries; i++) {
         if (neighbors[i].addr == 0) continue;
         pl[3 + 3*count + 0] = (neighbors[i].addr >> 8) & 0xff;
         pl[3 + 3*count + 1] = (neighbors[i].addr     ) & 0xff;
         pl[3 + 3*count + 2] = 1;  // cost=1 (or derive from your ND stats)
         count++;
      }
      pl[2] = count;
      lsaMySeq++;

      // Also update our own LSDB entry locally
      {
         uint8_t k = 0; uint16_t j;
         lsCount[TOS_NODE_ID] = 0;
         for (j = 0; j < MAX_NEIGHBORS && k < MAX_DEGREE; j++) {
            if (neighbors[j].addr == 0) continue;
            lsNbr[TOS_NODE_ID][k]  = neighbors[j].addr;
            lsCost[TOS_NODE_ID][k] = 1;
            k++;
         }
         lsCount[TOS_NODE_ID] = k;
      }

      dbg(ROUTING_CHANNEL, "LSA: send seq=%u entries=%u\n", lsaMySeq - 1, count);
      call Sender.send(p, AM_BROADCAST_ADDR);
   }

   void applyLSA(uint16_t origin, nx_uint8_t *pl, uint8_t len)
   {
      uint16_t seq = ((uint16_t)pl[0] << 8) | pl[1];
      uint8_t  cnt = pl[2];
      uint8_t  i, maxCnt = (PACKET_MAX_PAYLOAD_SIZE - 3) / 3;
      if (cnt > maxCnt) cnt = maxCnt;

      if (seq <= lsaLastSeq[origin]) {
         dbg(ROUTING_CHANNEL, "LSA: old from %u (got %u <= have %u)\n",
             origin, seq, lsaLastSeq[origin]);
         return;
      }
      lsaLastSeq[origin] = seq;

      lsCount[origin] = 0;
      for (i = 0; i < cnt && i < MAX_DEGREE; i++) {
         uint16_t nb = ((uint16_t)pl[3 + 3*i] << 8) | pl[3 + 3*i + 1];
         uint8_t  c  = pl[3 + 3*i + 2];
         lsNbr[origin][i]  = nb;
         lsCost[origin][i] = c;
      }
      lsCount[origin] = (cnt > MAX_DEGREE) ? MAX_DEGREE : cnt;

      dbg(ROUTING_CHANNEL, "LSA: rx from %u seq=%u entries=%u\n", origin, seq, lsCount[origin]);
      recomputeRoutes();
   }

   // Neighbor helpers
   int16_t findNeighbor(uint16_t a){
      int16_t i;
      for (i = 0; i < MAX_NEIGHBORS; i++){
         if (neighbors[i].addr == a) return i;
      }
      return -1;
   }
   int16_t allocNeighborSlot(){
      int16_t i;
      for (i = 0; i < MAX_NEIGHBORS; i++){
         if (neighbors[i].addr == 0) return i;
      }
      return -1;
   }
   void noteNeighborHeard(uint16_t a){
      int16_t idx = findNeighbor(a);
      if (idx < 0){
         idx = allocNeighborSlot();
         if (idx >= 0){
            neighbors[idx].addr = a;
            neighbors[idx].misses = 0;
            dbg(NEIGHBOR_CHANNEL, "ND: add neighbor %u\n", a);
            sendLSA ();
            recomputeRoutes ();
         }else{
            dbg(NEIGHBOR_CHANNEL, "ND: neighbor table full, cannot add %u\n", a);
         }
      }else{
         neighbors[idx].misses = 0; // reset miss counter on any reply
      }
   }

   // Prototypes
   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t Protocol, uint16_t seq, uint8_t *payload, uint8_t length);

   event void Boot.booted(){
      call AMControl.start();

      call periodicTimer.startPeriodic(10000 + (TOS_NODE_ID * 137) % 2000);

      dbg(GENERAL_CHANNEL, "Booted\n");
   }

   event void AMControl.startDone(error_t err){
      if(err == SUCCESS){
         dbg(GENERAL_CHANNEL, "Radio On\n");
      }else{
         //Retry until successful
         call AMControl.start();
      }
   }

   event void periodicTimer.fired () {
      pack p;
      int16_t i;

      for (i = 0; i < MAX_NEIGHBORS; i++) {
         if (neighbors[i].addr == 0) continue;
         if (neighbors[i].misses < 255) neighbors[i].misses++;
         if (neighbors[i].misses >= ND_MISS_THRESHOLD) {
            dbg(NEIGHBOR_CHANNEL, "ND: drop neighbor %u (misses=%u)\n", neighbors[i].addr, neighbors[i].misses);
            neighbors[i].addr = 0;
            neighbors[i].misses = 0;
         }
      }

      p.src = TOS_NODE_ID;
      p.dest = AM_BROADCAST_ADDR;
      p.seq = nextSeq++;
      p.TTL = 1;
      p.protocol = PROTOCOL_PING;
      memset (p.payload, 0, PACKET_MAX_PAYLOAD_SIZE);

      dbg(
         NEIGHBOR_CHANNEL,
         "ND: node %u sending probe seq=%u\n",
         TOS_NODE_ID,
         p.seq
      );
      call Sender.send(p, AM_BROADCAST_ADDR);
   }

   event void AMControl.stopDone(error_t err){}

   event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
      uint16_t src, seq, last, dest, ttl;
      pack* p;
      pack forward_packet;
      error_t e;

      dbg(GENERAL_CHANNEL, "Packet Received\n");
      if (len != sizeof (pack)) {
         dbg(GENERAL_CHANNEL, "Unknown Packet Type %d\n", len);
         return msg;
      };
      p = (pack*) payload;
      
      src = p->src;
      seq = p->seq;
      dest = p->dest;
      ttl = p->TTL;

      if (p->protocol == PROTOCOL_LINKSTATE) {
         // Use your duplicate filter; if this is the first time, SeqMap will be updated below.
         // Parse/apply LSA
         applyLSA(src, p->payload, PACKET_MAX_PAYLOAD_SIZE);
         if (ttl > 1) {
            pack fwd = *p; fwd.TTL = ttl - 1;
            dbg(FLOODING_CHANNEL, "LSA FWD from %u ttl=%u\n", src, fwd.TTL);
            call Sender.send(fwd, AM_BROADCAST_ADDR);
         }
         return msg;
      }

      if (p->protocol == PROTOCOL_PING && dest == AM_BROADCAST_ADDR) {
         if (src != TOS_NODE_ID) {
            pack reply;

            dbg(NEIGHBOR_CHANNEL, "ND: node %u received ND probe from %u seq=%u - replying\n", TOS_NODE_ID, src, seq);

            reply.src = TOS_NODE_ID;
            reply.dest = src;
            reply.seq = nextSeq++;
            reply.TTL = MAX_TTL;
            reply.protocol = PROTOCOL_PINGREPLY;
            memcpy(reply.payload, p->payload, PACKET_MAX_PAYLOAD_SIZE);
            ((char*)reply.payload)[PACKET_MAX_PAYLOAD_SIZE - 1] = '\0';

            call SeqMap.insert(reply.src, reply.seq);

            e = call Sender.send(reply, AM_BROADCAST_ADDR);
            if (e != SUCCESS) {
               dbg(GENERAL_CHANNEL, "Sender.send (ND reply) returned %d\n", e);
            }
         }
         return msg;
      }

      if (!call SeqMap.contains(src)) {
         call SeqMap.insert(src, seq);
         dbg(FLOODING_CHANNEL, "NEW from %u seq=%u (not seen before)\n", src, seq);
      } else {
         last = call SeqMap.get(src);
         if (seq <= last) {
            dbg(FLOODING_CHANNEL, "DUP from %u seq=%u (last %u); drop\n", src, seq, last);
            return msg;
         } else {
            call SeqMap.insert(src, seq);
            dbg(FLOODING_CHANNEL, "NEW from %u seq=%u (was %u)\n", src, seq, last);
         }
      }

      // is for me? 🥺👉👈
      if (dest == TOS_NODE_ID) {
         // Safe %s printing
         ((char*)p->payload)[PACKET_MAX_PAYLOAD_SIZE - 1] = '\0';
         if (p->protocol == PROTOCOL_PING){
            pack reply2;
            dbg(FLOODING_CHANNEL, "PING to me (%u) from %u seq=%u payload=%s\n", TOS_NODE_ID, src, seq, p->payload);
            // Send a ping reply back to the origin (prefer unicast via routing)
            reply2.src = TOS_NODE_ID;
            reply2.dest = src;
            reply2.seq = nextSeq++;
            reply2.TTL = MAX_TTL;
            reply2.protocol = PROTOCOL_PINGREPLY;
            memcpy(reply2.payload, p->payload, PACKET_MAX_PAYLOAD_SIZE);
            ((char*)reply2.payload)[PACKET_MAX_PAYLOAD_SIZE - 1] = '\0';
            if (src <= MAX_NODES && routeNext[src] != 0 && routeCost[src] < INF_COST) {
               uint16_t nh2 = routeNext[src];
               dbg(ROUTING_CHANNEL, "ROUTE REPLY to %u via %u\n", src, nh2);
               e = call Sender.send(reply2, nh2);
            } else {
               // fall back to flood if no route yet
               e = call Sender.send(reply2, AM_BROADCAST_ADDR);
            }
            if (e != SUCCESS) {
               dbg(GENERAL_CHANNEL, "Sender.send (PING reply) returned %d\n", e);
            }
         } else if (p->protocol == PROTOCOL_PINGREPLY){
            dbg(FLOODING_CHANNEL, "PINGREPLY to me (%u) from %u seq=%u payload=%s\n", TOS_NODE_ID, src, seq, p->payload);
            // Treat any ping-reply addressed to me as ND evidence
            noteNeighborHeard(src);
            dbg(NEIGHBOR_CHANNEL, "ND: reply heard from %u\n", src);
         } else {
            dbg(FLOODING_CHANNEL, "DEL to %u from %u seq=%u proto=%u payload=%s\n", TOS_NODE_ID, src, seq, p->protocol, p->payload);
         }
         return msg;
      }

      if (dest != AM_BROADCAST_ADDR) {
         if (ttl <= 1) {
            dbg(ROUTING_CHANNEL, "DROP TTL0 src=%u dst=%u seq=%u\n", src, dest, seq);
            return msg;
         }
         // route lookup
         if (dest <= MAX_NODES && routeNext[dest] != 0 && routeCost[dest] < INF_COST) {
            uint16_t nh = routeNext[dest];
            pack fwd = *p; fwd.TTL = ttl - 1;
            dbg(ROUTING_CHANNEL, "ROUTE FWD src=%u dst=%u via=%u seq=%u ttl=%u\n",
               src, dest, nh, seq, fwd.TTL);
            call Sender.send(fwd, nh);   // unicast to next-hop
         } else {
            dbg(ROUTING_CHANNEL, "NO ROUTE src=%u dst=%u seq=%u — drop\n", src, dest, seq);
         }
         return msg;
      }

      forward_packet = *p;
      forward_packet.TTL = ttl - 1;
      dbg(
         FLOODING_CHANNEL, "FWD src=%u, dst=%u, seq=%u, ttl=%u\n",
         src, dest, seq, forward_packet.TTL
      );
      e = call Sender.send(forward_packet, AM_BROADCAST_ADDR);
      if (e != SUCCESS) {
         dbg(GENERAL_CHANNEL, "Sender.send returned %d\n", e);
      }

      return msg;
   }


   event void CommandHandler.ping(uint16_t destination, uint8_t *payload){
      dbg(GENERAL_CHANNEL, "PING EVENT \n");
      makePack(&sendPackage, TOS_NODE_ID, destination, MAX_TTL, PROTOCOL_PING, nextSeq++, payload, PACKET_MAX_PAYLOAD_SIZE);
      // Prefer routed unicast if we have a route; otherwise flood to bootstrap
      if (destination <= MAX_NODES && routeNext[destination] != 0 && routeCost[destination] < INF_COST) {
         uint16_t nh = routeNext[destination];
         dbg(ROUTING_CHANNEL, "ROUTE ORIG dst=%u via=%u\n", destination, nh);
         call Sender.send(sendPackage, nh);
      } else {
         dbg(FLOODING_CHANNEL, "ORIG flood dst=%u (no route yet)\n", destination);
         call Sender.send(sendPackage, AM_BROADCAST_ADDR);
      }
   }

   event void CommandHandler.printNeighbors(){
      int16_t i;
      dbg(NEIGHBOR_CHANNEL, "Neighbor dump for node %u:\n", TOS_NODE_ID);
      for (i = 0; i < MAX_NEIGHBORS; i++){
         if (neighbors[i].addr != 0){
            dbg(NEIGHBOR_CHANNEL, "  neighbor=%u misses=%u\n", neighbors[i].addr, neighbors[i].misses);
         }
         // continue;
      }
   }

   event void CommandHandler.printRouteTable(){
      uint16_t d;
      dbg(ROUTING_CHANNEL, "Route table (me=%u):\n", TOS_NODE_ID);
      for (d = 1; d <= MAX_NODES; d++){
         if (routeNext[d] != 0 && routeCost[d] < INF_COST) {
            dbg(ROUTING_CHANNEL, "  dest=%u next=%u cost=%u\n", d, routeNext[d], routeCost[d]);
         }
      }
      // Optional: dump LSDB
      for (d = 1; d <= MAX_NODES; d++){
         uint8_t i;
         if (lsCount[d] == 0) continue;
         dbg(ROUTING_CHANNEL, "  LSA[%u] seq=%u:", d, lsaLastSeq[d]);
         for (i = 0; i < lsCount[d]; i++){
            dbg(ROUTING_CHANNEL, " (%u,c%u)", lsNbr[d][i], lsCost[d][i]);
         }
         dbg(ROUTING_CHANNEL, "\n");
      }
   }

   event void CommandHandler.printLinkState(){}

   event void CommandHandler.printDistanceVector(){}

   event void CommandHandler.setTestServer(){}

   event void CommandHandler.setTestClient(){}

   event void CommandHandler.setAppServer(){}

   event void CommandHandler.setAppClient(){}

   void makePack(pack *Package, uint16_t src, uint16_t dest, uint16_t TTL, uint16_t protocol, uint16_t seq, uint8_t* payload, uint8_t length){
      Package->src = src;
      Package->dest = dest;
      Package->TTL = TTL;
      Package->seq = seq;
      Package->protocol = protocol;
      memcpy(Package->payload, payload, length);
   }
}
