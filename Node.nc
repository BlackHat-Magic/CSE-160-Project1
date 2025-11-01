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
         dbg(
            FLOODING_CHANNEL, "DEL to %u from %u seq=%u proto=%u, payload=%s\n",
            TOS_NODE_ID, src, seq, p->protocol, p->payload
         );
         return msg;
      }

      if (ttl <= 1) {
         dbg(FLOODING_CHANNEL, "TTL expired drop src=%u seq=%u\n", src, seq);
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
      call Sender.send(sendPackage, AM_BROADCAST_ADDR);
   }

   event void CommandHandler.printNeighbors(){}

   event void CommandHandler.printRouteTable(){}

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
