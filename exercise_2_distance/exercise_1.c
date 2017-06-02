/*
 * Copyright (c) 2015, TU Graz, Austria.
 * All rights reserved.
 *
 * \file
 * File provided for the Location-Aware Computing Labor course.
 *
 * \author
 * Carlo Alberto Boano <cboano@tugraz.at>
 *
 * \student submission
 * Darjan Salaj 1031384
*/


#include "contiki.h"
#include <stdio.h>	     // For printf
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include "dev/light-sensor.h"
#include "dev/sht11/sht11-sensor.h"
#include "net/rime/rime.h"
#include "net/rime/collect.h"
#include "net/netstack.h"
#include "cc2420.h"
#include "pt.h"
#include "energest.h"


// Global variables
float x=2.000f;
static uint32_t energy_start;
static uint16_t packet_counter = 0;
#define ANCHOR_RIME_BROADCAST_CHANNEL 150
#define ANCHOR_PHY_CHANNEL 20
// Change this and project-conf.h to switch between nodes


int pa_to_dbm(int pa_level) {
    if(pa_level == 31)
        return 0;
    if(pa_level == 27)
        return -1;
    if(pa_level == 23)
        return -3;
    if(pa_level == 19)
        return -5;
    if(pa_level == 15)
        return -7;
    if(pa_level == 11)
        return -10;
    if(pa_level == 7)
        return -15;
    if(pa_level == 3)
        return -25;
    if(pa_level == 2)
        return -45;
    if(pa_level == 0)
        return -55;
}

float floor(float x){
  if(x>=0.0f)
    return (float) ((int)x);
  else
    return(float)((int)x-1);
}

float distance(float Pt, float RSSI, float n, float d0) {
    return powf(10.0f, ((Pt - 46.0f - RSSI)/(10.0f * n))) * d0;

}

struct ex2_packet {
    uint32_t sequence_number;
    uint8_t tx_power;
};
struct ex2_packet ex2_packet;

// Main process
PROCESS(exercise_1, "Exercise 1");
AUTOSTART_PROCESSES(&exercise_1);


static struct broadcast_conn broadcast;
static uint32_t power = 0;
static int32_t min_dbm = 0;
static uint8_t focus = 1;
const int NODE_ADDR = 108;


// BROADCAST RECEIVE
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
    //printf("broadcast message received from %d.%d\n", from->u8[0], from->u8[1]);
    if (focus == 0 || from->u8[0] == NODE_ADDR){
        if (packet_counter < 30) {
            packet_counter++;
            struct ex2_packet msg;
            memcpy(&msg, packetbuf_dataptr(), sizeof(msg));
            int dbm = pa_to_dbm(msg.tx_power);
            printf("Packet from %d.%d; Sequence number = %ld; Power = %d; RSSI = %d\n", from->u8[0], from->u8[1], msg.sequence_number, dbm, cc2420_last_rssi);
            float d = distance((float)dbm, (float)cc2420_last_rssi, 6.0f, 2.0f);
            printf("Distance = %ld.%03d m\n", (long) d, (unsigned) ((d - floor(d))*1000));
            if (dbm < min_dbm)
                min_dbm = dbm;
        } else {
            printf("Smallest power = %d dBm\n", min_dbm);
            packet_counter = 0;
        }
    }
}

// MAIN THREAD
PROCESS_THREAD(exercise_1, ev, data){
    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
    PROCESS_BEGIN();
    cc2420_set_channel(ANCHOR_PHY_CHANNEL);

    static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

    energy_start = energest_type_time(ENERGEST_TYPE_LISTEN);
    broadcast_open(&broadcast, ANCHOR_RIME_BROADCAST_CHANNEL, &broadcast_call);

    PROCESS_END();
}
