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
static linkaddr_t addr = {143, 0}; // sink address
static uint8_t focus = 0;
const int NODE_ADDR = 108;
static uint8_t reception_counter = 0;

float x=2.000f;
#define ANCHOR_RIME_BROADCAST_CHANNEL 150
#define ANCHOR_PHY_CHANNEL 20 // for anchors
// #define ANCHOR_PHY_CHANNEL 25 // for unicast with sink
#define NO_ANCHORS_LOCALIZE 20 // number of anchor packets to localize from
static struct unicast_conn uc;
static struct broadcast_conn broadcast;
static uint32_t seq_num = 0;
static uint32_t sent_seq_num = 0;
static uint16_t par = 0;
static uint16_t tsr = 0;
static double T = 0.0f;
static double RH = 0.0f;

static struct ctimer localization_timer;
#define LOCALIZATION_TIME (CLOCK_SECOND * 20)
#define LOWER_LIMIT_TR_POWER_LOC -25  // lowest transmission power accepted packet for localization

static void read_sensors() {
    SENSORS_ACTIVATE(light_sensor);
    par = light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC);
    tsr = light_sensor.value(LIGHT_SENSOR_TOTAL_SOLAR);
    SENSORS_DEACTIVATE(light_sensor);
    SENSORS_ACTIVATE(sht11_sensor);
    uint16_t temp = sht11_sensor.value(SHT11_SENSOR_TEMP);
    uint16_t rh = sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
    T = -39.6f + 0.01f * temp;
    double RH_lin = -2.0468 + 0.0367 * rh + -1.5955e-6 * (rh*rh);
    RH = (T - 25)*(0.01 + 0.00008*rh) + RH_lin;
    SENSORS_DEACTIVATE(sht11_sensor);
}

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

struct unicast_packet {
    uint32_t sequence_number;
    int32_t x;
    int32_t y;
    float temperature;
    float humidity;
    uint16_t par;
    uint16_t tsr;
    uint8_t led_r;
    uint8_t led_g;
    uint8_t led_b;
    uint8_t reserved;
};
struct unicast_packet p;

struct anchor {
    int addr;
    uint16_t dist;
};
struct anchor anchor_data[NO_ANCHORS_LOCALIZE];

struct coords {
    int x;
    int y;
};
struct coords ll_coord;
struct coords ur_coord;
struct coords map[32] = {
    {.x = 368, .y = 0},    // node 100
    {.x = 170, .y = 0},    // node 101
    {.x = 32, .y = 149},   // node 102
    {.x = 32, .y = 300},   // node 103
    {.x = 32, .y = 459},   // node 104
    {.x = 32, .y = 609},   // node 105
    {.x = 32, .y = 754},   // node 106
    {.x = 32, .y = 865},   // node 107
    {.x = 164, .y = 1000}, // node 108
    {.x = 368, .y = 1000}, // node 109
    {.x = 465, .y = 859},  // node 110
    {.x = 465, .y = 750},  // node 111
    {.x = 465, .y = 610},  // node 112
    {.x = 465, .y = 462},  // node 113
    {.x = 465, .y = 310},  // node 114
    {.x = 465, .y = 151},  // node 115
    {.x = 537, .y = 100},  // node 116
    {.x = 630, .y = 510},  // node 117
    {.x = 630, .y = 694},  // node 118
    {.x = 47, .y = 960},   // node 119
    {.x = 16, .y = 227},   // node 120
    {.x = 16, .y = 385},   // node 121
    {.x = 16, .y = 689},   // node 122
    {.x = 16, .y = 815},   // node 123
    {.x = 578, .y = 985},  // node 124
    {.x = 610, .y = 694},  // node 125
    {.x = 610, .y = 467},  // node 126
    {.x = 408, .y = 35},   // node 127
    {.x = 910, .y = -100}, // node 128
    {.x = 88, .y = 0},     // node 129
    {.x = 42, .y = 0},     // node 130
    {.x = 22, .y = 70},    // node 131
};


static int my_x() {
    return (ur_coord.x + ll_coord.x)/2;
}
static int my_y() {
    return (ur_coord.y + ll_coord.y)/2;
}

static void localize() {
    ll_coord.x = map[anchor_data[0].addr % 100].x - anchor_data[0].dist;
    ll_coord.y = map[anchor_data[0].addr % 100].y - anchor_data[0].dist;
    ur_coord.x = map[anchor_data[0].addr % 100].x + anchor_data[0].dist;
    ur_coord.y = map[anchor_data[0].addr % 100].y + anchor_data[0].dist;
    // printf("Starting LL x=%d y=%d, UR x=%d y=%d\n", ll_coord.x, ll_coord.y, ur_coord.x, ur_coord.y);
    int i = 1;
    while (i < NO_ANCHORS_LOCALIZE){
        int ll_x = map[anchor_data[i].addr % 100].x - anchor_data[i].dist;
        int ll_y = map[anchor_data[i].addr % 100].y - anchor_data[i].dist;
        int ur_x = map[anchor_data[i].addr % 100].x + anchor_data[i].dist;
        int ur_y = map[anchor_data[i].addr % 100].y + anchor_data[i].dist;
        // printf("checking node %d LL x=%d y=%d, UR x=%d y=%d\n",anchor_data[i].addr, ll_x, ll_y, ur_x, ur_y);
        if (ll_x > ll_coord.x){
            ll_coord.x = ll_x;
            // printf("took ll x\n");
        }
        if (ll_y > ll_coord.y){
            ll_coord.y = ll_y;
            // printf("took ll y\n");
        }
        if (ur_x < ll_coord.x) {
            ll_coord.x = ur_x;
            // printf("took ur x\n");
        }
        if (ur_y < ll_coord.y) {
            ll_coord.y = ur_y;
            // printf("took ur y\n");
        }

        i++;
    }
    printf("I am at x=%d y=%d!\n", my_x(), my_y());

    read_sensors();

    p.sequence_number = seq_num;
    p.x = my_x();
    p.y = my_y();
    p.temperature = T; //T;
    p.humidity = RH; // RH;
    p.par = par; //par;
    p.tsr = tsr; //tsr;
    p.led_r = 0;
    p.led_g = 0;
    p.led_b = 0;
    p.reserved = 0;
    sent_seq_num = seq_num;

    // after reception of ACK the seq_num will be incremented
    // until then, retransmit every 0.5 seconds
    printf("Going to send to sink: %d.%d ", addr.u8[0], addr.u8[1]);
    while(sent_seq_num == seq_num){
        packetbuf_clear();
        packetbuf_copyfrom(&p, sizeof(struct unicast_packet));
        if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
          unicast_send(&uc, &addr);
        }
        // FIXME: can not use etimer here!
        // Wait for 0.5s before retransmitting
        // static struct etimer retransmit_timer;
        // etimer_set(&retransmit_timer, (CLOCK_SECOND / 2));
        // PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&retransmit_timer));
        // printf(".");
        break;
    }
    printf("\n");
}

// Main process
PROCESS(exercise_1, "Exercise 1");
AUTOSTART_PROCESSES(&exercise_1);


// UNICAST RECEIVE
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
    //printf("unicast message received from %d.%d\n", from->u8[0], from->u8[1]);
    //printf("sink signal strength = %d, link quality = %d\n", cc2420_last_rssi, cc2420_last_correlation);
    if (from->u8[0] == 143) {
        struct unicast_packet msg;
        memcpy(&msg, packetbuf_dataptr(), sizeof(msg));
        if (seq_num == msg.sequence_number && !msg.temperature && !msg.humidity
            && !msg.led_r && !msg.led_g && !msg.led_b && !msg.par && !msg.tsr
            && !msg.reserved
            ) {
            seq_num++;
        } else {
            printf("ERROR: received corrupted ACK packet!\n");
        }
    }
}
static void send_uc(struct unicast_conn *c, int status, int tx) {
}

static uint8_t check_addr(int addr) {
    int used_anchors[4] = {102, 106, 111, 114};
    int i;
    for (i = 0; i < 4; i++) {
        if (used_anchors[i] == addr)
            return 1;
    }
    return 0;
}
// BROADCAST RECEIVE
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
    //printf("broadcast message received from %d.%d\n", from->u8[0], from->u8[1]);
    //if (focus == 0 || from->u8[0] == NODE_ADDR){
    if (check_addr(from->u8[0])){
    // if (from->u8[0] >= 100 && from->u8[0] <= 131){
    //if (from->u8[0] == 104 || from->u8[0] == 108 || from->u8[0] == 117 || from->u8[0] == 115){
        struct ex2_packet msg;
        memcpy(&msg, packetbuf_dataptr(), sizeof(msg));
        int dbm = pa_to_dbm(msg.tx_power);
        //printf("Packet from %d.%d; Sequence number = %ld; Power = %d; RSSI = %d\n", from->u8[0], from->u8[1], msg.sequence_number, dbm, cc2420_last_rssi);
        if (dbm >= LOWER_LIMIT_TR_POWER_LOC) {
            float d = distance((float)dbm, (float)cc2420_last_rssi, 6.0f, 2.0f);
            //printf("Distance = %ld.%03d m\n", (long) d, (unsigned) ((d - floor(d))*1000));
            if (reception_counter < NO_ANCHORS_LOCALIZE){
                uint16_t d_cm = (uint16_t)(d * 100);
                //printf("Distance from node %d = %ld.%03d m\n", from->u8[0], (long) d, (unsigned) ((d - floor(d))*1000));
                printf("Distance from node %d = %d cm\n", from->u8[0], d_cm);
                anchor_data[reception_counter].addr = from->u8[0];
                anchor_data[reception_counter].dist = d_cm; // convert meters to centimeters
                reception_counter++;
                if (reception_counter == NO_ANCHORS_LOCALIZE) {
                    // printf("Now calculate the minmax location!\n");
                    localize();
                }
            }
        }
    }
}

static void reset_reception_counter(void *ptr);
static void reset_reception_counter(void *ptr){
    reception_counter = 0;
    ctimer_set(&localization_timer, LOCALIZATION_TIME, reset_reception_counter, NULL);
}

// MAIN THREAD
PROCESS_THREAD(exercise_1, ev, data){
    PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
    PROCESS_BEGIN();
    cc2420_set_channel(ANCHOR_PHY_CHANNEL);

    static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
    broadcast_open(&broadcast, ANCHOR_RIME_BROADCAST_CHANNEL, &broadcast_call);
    static const struct unicast_callbacks unicast_callbacks = {recv_uc, send_uc};
    unicast_open(&uc, 181, &unicast_callbacks);
    ctimer_set(&localization_timer, LOCALIZATION_TIME, reset_reception_counter, NULL);

    // wait for button event
    while(1) {
        SENSORS_ACTIVATE(button_sensor);
        PROCESS_WAIT_EVENT();
        if((ev == sensors_event && data == &button_sensor) || etimer_expired(&localization_timer)){
            reception_counter = 0;
        }
        SENSORS_DEACTIVATE(button_sensor);
    }
    PROCESS_END();
}
