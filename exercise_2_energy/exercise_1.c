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
static uint32_t energy_end;
static uint32_t cpu_start;
static uint32_t cpu_end;
static uint32_t lpm_start;
static uint32_t lpm_end;
static uint16_t packet_counter = 0;
#define ANCHOR_RIME_BROADCAST_CHANNEL 150
#define ANCHOR_PHY_CHANNEL 20
#define NODE_1_ADDR 134
#define NODE_1_PHY 14
#define NODE_2_ADDR 131
#define NODE_2_PHY 11
// Change this and project-conf.h to switch between nodes
const int NODE_ADDR = NODE_2_ADDR;
const int NODE_PHY = NODE_2_PHY;


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

struct ex2_packet {
    uint32_t sequence_number;
    uint8_t tx_power;
};
struct ex2_packet ex2_packet;

// Main process
PROCESS(exercise_1, "Exercise 1");
AUTOSTART_PROCESSES(&exercise_1);


static struct broadcast_conn broadcast;
static uint8_t done = 0;


// BROADCAST RECEIVE
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
    // if (done == 0)
    //     printf("broadcast message received from %d.%d\n", from->u8[0], from->u8[1]);
    if (from->u8[0] == NODE_ADDR) {
        if (packet_counter < 10) {
            packet_counter++;
            struct ex2_packet msg;
            memcpy(&msg, packetbuf_dataptr(), sizeof(msg));
            int dbm = pa_to_dbm(msg.tx_power);
            // printf("Packet from %d; Sequence number = %ld; Power = %d\n", NODE_ADDR, msg.sequence_number, dbm);
            if (packet_counter == 10) {
                energy_end = energest_type_time(ENERGEST_TYPE_LISTEN);
                cpu_end = energest_type_time(ENERGEST_TYPE_CPU);
                lpm_end = energest_type_time(ENERGEST_TYPE_LPM);
            }
        } else if (done == 0) {
            done = 1;
            uint32_t energy_total_ticks = energy_end - energy_start;
            uint32_t cpu_total_ticks = cpu_end - cpu_start;
            uint32_t lpm_total_ticks = lpm_end - lpm_start;
            printf("Radio ticks = %ld.%03d \n", (long) energy_total_ticks, (unsigned) ((energy_total_ticks - floor(energy_total_ticks))*1000));
            printf("CPU ticks = %ld.%03d \n", (long) cpu_total_ticks, (unsigned) ((cpu_total_ticks - floor(cpu_total_ticks))*1000));
            printf("LPM ticks = %ld.%03d \n", (long) lpm_total_ticks, (unsigned) ((lpm_total_ticks - floor(lpm_total_ticks))*1000));
            float energy_total_time = (float)energy_total_ticks / (float)RTIMER_SECOND;
            float cpu_total_time = (float)cpu_total_ticks / (float)RTIMER_SECOND;
            float lpm_total_time = (float)lpm_total_ticks / (float)RTIMER_SECOND;
            printf("Radio time = %ld.%03d s\n", (long) energy_total_time, (unsigned) ((energy_total_time - floor(energy_total_time))*1000));
            printf("CPU time = %ld.%03d s\n", (long) cpu_total_time, (unsigned) ((cpu_total_time - floor(cpu_total_time))*1000));
            printf("LPM time = %ld.%03d s\n", (long) lpm_total_time, (unsigned) ((lpm_total_time - floor(lpm_total_time))*1000));
            float energy_total = energy_total_time * 20.0f * 3.3f; // listen current is 20mA and typical voltage 3.3V
            float cpu_total = cpu_total_time * 1.8f * 3.3f;
            float lpm_total = lpm_total_time * 0.0545f * 3.3f;
            printf("Radio energy = %ld.%03d mJ\n", (long) energy_total, (unsigned) ((energy_total - floor(energy_total))*1000));
            printf("CPU energy = %ld.%03d mJ\n", (long) cpu_total, (unsigned) ((cpu_total - floor(cpu_total))*1000));
            printf("LPM energy = %ld.%03d mJ\n", (long) lpm_total, (unsigned) ((lpm_total - floor(lpm_total))*1000));
            float seconds = (float)(cpu_total_time + lpm_total_time);
            printf("seconds = %ld.%03d s\n", (long) seconds, (unsigned) ((seconds - floor(seconds))*1000));
            float power_total = energy_total / seconds;
            float cpu_power = cpu_total / seconds;
            float lpm_power = lpm_total / seconds;
            printf("Radio power = %ld.%03d mW\n", (long) power_total, (unsigned) ((power_total - floor(power_total))*1000));
            printf("CPU power = %ld.%03d mW\n", (long) cpu_total, (unsigned) ((cpu_total - floor(cpu_total))*1000));
            printf("LPM power = %ld.%03d mW\n", (long) lpm_total, (unsigned) ((lpm_total - floor(lpm_total))*1000));
            printf("DONE!");
        }
    } else {
        printf("WARNING: received broadcast message from unknown source!\n");
    }
}

// MAIN THREAD
PROCESS_THREAD(exercise_1, ev, data){
    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
    PROCESS_BEGIN();
    cc2420_set_txpower(11);
    cc2420_set_channel(NODE_PHY);

    static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

    energy_start = energest_type_time(ENERGEST_TYPE_LISTEN);
    cpu_start = energest_type_time(ENERGEST_TYPE_CPU);
    lpm_start = energest_type_time(ENERGEST_TYPE_LPM);
    broadcast_open(&broadcast, ANCHOR_RIME_BROADCAST_CHANNEL, &broadcast_call);

    PROCESS_END();
}
