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


// Main process
PROCESS(exercise_1, "Exercise 1");
AUTOSTART_PROCESSES(&exercise_1);

// Global variables
static linkaddr_t addr = {143, 0}; // sink address
static uint8_t focus = 0;
const int NODE_ADDR = 108;
static uint8_t reception_counter = 0;
// static int used_anchors[4] = {102, 106, 109, 111, 113, 115};
static int used_anchors[11] = {108, 109, 100, 101, 104, 105, 110, 111, 113, 114, 115};
#define USED_ANCHOR_NUM 11

float x=2.000f;
#define ANCHOR_RIME_BROADCAST_CHANNEL 150
#define ANCHOR_PHY_CHANNEL 20 // for anchors
#define SINK_PHY_CHANNEL 25 // for unicast with sink
#define NO_ANCHORS_LOCALIZE 30 // number of anchor packets to localize from
static struct unicast_conn uc;
static struct broadcast_conn broadcast;
static uint32_t seq_num = 0;
static uint32_t sent_seq_num = 0;
static uint16_t par = 0;
static uint16_t tsr = 0;
static double T = 0.0f;
static double RH = 0.0f;

static uint8_t forward_to_sink = 0;
static struct ctimer localization_timer;
static struct ctimer sink_timer;
static struct ctimer retransmit_timer;
#define LOCALIZATION_TIME (CLOCK_SECOND * 20)
#define LOWER_LIMIT_TR_POWER_LOC -20  // lowest transmission power accepted packet for localization

int absolute(int value) {
    if (value < 0) {
        return -value;
    }
    else {
        return value;
    }
}
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
    uint16_t dist;
    uint8_t no_dist;
};
struct coords ll_coord;
struct coords ur_coord;
struct coords map[32] = {
    {.no_dist = 0, .dist = 0, .x = 368, .y = 0},    // node 100
    {.no_dist = 0, .dist = 0, .x = 170, .y = 0},    // node 101
    {.no_dist = 0, .dist = 0, .x = 32, .y = 149},   // node 102
    {.no_dist = 0, .dist = 0, .x = 32, .y = 300},   // node 103
    {.no_dist = 0, .dist = 0, .x = 32, .y = 459},   // node 104
    {.no_dist = 0, .dist = 0, .x = 32, .y = 609},   // node 105
    {.no_dist = 0, .dist = 0, .x = 32, .y = 754},   // node 106
    {.no_dist = 0, .dist = 0, .x = 32, .y = 865},   // node 107
    {.no_dist = 0, .dist = 0, .x = 164, .y = 1000}, // node 108
    {.no_dist = 0, .dist = 0, .x = 368, .y = 1000}, // node 109
    {.no_dist = 0, .dist = 0, .x = 465, .y = 859},  // node 110
    {.no_dist = 0, .dist = 0, .x = 465, .y = 750},  // node 111
    {.no_dist = 0, .dist = 0, .x = 465, .y = 610},  // node 112
    {.no_dist = 0, .dist = 0, .x = 465, .y = 462},  // node 113
    {.no_dist = 0, .dist = 0, .x = 465, .y = 310},  // node 114
    {.no_dist = 0, .dist = 0, .x = 465, .y = 151},  // node 115
    {.no_dist = 0, .dist = 0, .x = 537, .y = 100},  // node 116
    {.no_dist = 0, .dist = 0, .x = 630, .y = 510},  // node 117
    {.no_dist = 0, .dist = 0, .x = 630, .y = 694},  // node 118
    {.no_dist = 0, .dist = 0, .x = 47, .y = 960},   // node 119
    {.no_dist = 0, .dist = 0, .x = 16, .y = 227},   // node 120
    {.no_dist = 0, .dist = 0, .x = 16, .y = 385},   // node 121
    {.no_dist = 0, .dist = 0, .x = 16, .y = 689},   // node 122
    {.no_dist = 0, .dist = 0, .x = 16, .y = 815},   // node 123
    {.no_dist = 0, .dist = 0, .x = 578, .y = 985},  // node 124
    {.no_dist = 0, .dist = 0, .x = 610, .y = 694},  // node 125
    {.no_dist = 0, .dist = 0, .x = 610, .y = 467},  // node 126
    {.no_dist = 0, .dist = 0, .x = 408, .y = 35},   // node 127
    {.no_dist = 0, .dist = 0, .x = 910, .y = -100}, // node 128
    {.no_dist = 0, .dist = 0, .x = 88, .y = 0},     // node 129
    {.no_dist = 0, .dist = 0, .x = 42, .y = 0},     // node 130
    {.no_dist = 0, .dist = 0, .x = 22, .y = 70},    // node 131
};


static int my_x() {
    return (ur_coord.x + ll_coord.x)/2;
}
static int my_y() {
    return (ur_coord.y + ll_coord.y)/2;
}
static int my_precision() {
    int x_prec = absolute(absolute(ur_coord.x) - absolute(ll_coord.x));
    int y_prec = absolute(absolute(ur_coord.y) - absolute(ll_coord.y));
    return (x_prec + y_prec)/2;
}

static void retransmit(void *ptr);
static void retransmit(void *ptr){
    if (forward_to_sink) {
        printf("Retransmit ctimer callback!\n");
        packetbuf_clear();
        packetbuf_copyfrom(&p, sizeof(struct unicast_packet));
        if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
          unicast_send(&uc, &addr);
        }
        if(sent_seq_num == seq_num){
            ctimer_set(&retransmit_timer, (CLOCK_SECOND / 2), retransmit, NULL);
        } else {
            forward_to_sink = 0;
            cc2420_set_channel(ANCHOR_PHY_CHANNEL);
            printf("changing channel back to ANCHOR\n");
        }
    }
}
static void forward_to_sink_callback(void *ptr);
static void forward_to_sink_callback(void *ptr){
    cc2420_set_channel(SINK_PHY_CHANNEL);

    printf("SINK ctimer: Start of sink unicast process\n");
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
    printf("Going to send to sink: %d.%d \n", addr.u8[0], addr.u8[1]);
    if(sent_seq_num == seq_num){
        ctimer_set(&retransmit_timer, (CLOCK_SECOND / 2), retransmit, NULL);
    }
}
static void localize() {
    printf("STARTDRAW\n"); // DRAW Start Statement.
    struct anchor unique_anchor_data[USED_ANCHOR_NUM];
    int i = 0;
    uint8_t unstable[USED_ANCHOR_NUM] = {0, 0, 0, 0};
    uint16_t prev_dist[USED_ANCHOR_NUM];
    for (i = 0; i < NO_ANCHORS_LOCALIZE; i++){
        uint8_t anchor_counter = 0;
        for (anchor_counter = 0; anchor_counter < USED_ANCHOR_NUM; anchor_counter++){
            if (anchor_data[i].addr == used_anchors[anchor_counter]) {
                map[anchor_data[i].addr % 100].no_dist++;
                map[anchor_data[i].addr % 100].dist += anchor_data[i].dist;
                printf("node %d number %d distance sum %d\n", anchor_data[i].addr, map[anchor_data[i].addr % 100].no_dist, map[anchor_data[i].addr % 100].dist);
            }
        }
    }

    ll_coord.x = -5000;
    ll_coord.y = -5000;
    ur_coord.x = 5000;
    ur_coord.y = 5000;
    int ll_x = 0;
    int ll_y = 0;
    int ur_x = 0;
    int ur_y = 0;
    uint8_t anchor_counter = 0;
    for (anchor_counter = 0; anchor_counter < USED_ANCHOR_NUM; anchor_counter++){
        int dist = map[used_anchors[anchor_counter] % 100].dist / map[used_anchors[anchor_counter] % 100].no_dist;
        printf("LOCALIZE: node %d distance %d\n", used_anchors[anchor_counter], dist);
        // ignore the far nodes because of large fluctuation in distance
        if (dist > 450) {
            continue;
        }
        ll_x = map[used_anchors[anchor_counter] % 100].x - dist;
        ll_y = map[used_anchors[anchor_counter] % 100].y - dist;
        ur_x = map[used_anchors[anchor_counter] % 100].x + dist;
        ur_y = map[used_anchors[anchor_counter] % 100].y + dist;
        printf("DRAW_SQUARE_DENSITY(%d,%d,%d,%d,#00ff00,none,1.0)\n", ll_x, ll_y, ur_x, ur_y);
        if (ll_x > ll_coord.x){
            ll_coord.x = ll_x;
        }
        if (ll_y > ll_coord.y){
            ll_coord.y = ll_y;
        }
        if (ur_x < ur_coord.x) {
            ur_coord.x = ur_x;
        }
        if (ur_y < ur_coord.y) {
            ur_coord.y = ur_y;
        }
        map[used_anchors[anchor_counter] % 100].dist = 0;
        map[used_anchors[anchor_counter] % 100].no_dist = 0;
    }
    int x = my_x();
    int y = my_y();
    printf("I am at x=%d y=%d!\n", x, y);
    printf("DRAW_SQUARE_DENSITY(%d,%d,%d,%d,#00ff00,#ff2211,0.5)\n", ll_coord.x, ll_coord.y, ur_coord.x, ur_coord.y);
    printf("DRAW_CIRCLE(%d,%d,%d,#aaaa00, #44ff44)\n", x, y, 20);
    forward_to_sink = 1;
    ctimer_set(&sink_timer, (CLOCK_SECOND/2), forward_to_sink_callback, NULL);
}


// UNICAST RECEIVE
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
    if (from->u8[0] == 143) {
        struct unicast_packet msg;
        memcpy(&msg, packetbuf_dataptr(), sizeof(msg));
        if (seq_num == msg.sequence_number && !msg.temperature && !msg.humidity
            && !msg.led_r && !msg.led_g && !msg.led_b && !msg.par && !msg.tsr
            && !msg.reserved
            ) {
            seq_num++;
            printf("SUCCESS: received correct ACK packet!\n");
        } else {
            printf("ERROR: received corrupted ACK packet!\n");
        }
    }
}
static void send_uc(struct unicast_conn *c, int status, int tx) {
}

static uint8_t check_addr(int addr) {
    int i;
    for (i = 0; i < USED_ANCHOR_NUM; i++) {
        if (used_anchors[i] == addr)
            return 1;
    }
    return 0;
}
// BROADCAST RECEIVE
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
    if (check_addr(from->u8[0])){
        struct ex2_packet msg;
        memcpy(&msg, packetbuf_dataptr(), sizeof(msg));
        int dbm = pa_to_dbm(msg.tx_power);
        //printf("Packet from %d.%d; Sequence number = %ld; Power = %d; RSSI = %d\n", from->u8[0], from->u8[1], msg.sequence_number, dbm, cc2420_last_rssi);
        if (dbm >= LOWER_LIMIT_TR_POWER_LOC) {
            float d = distance((float)dbm, (float)cc2420_last_rssi, 4.0f, 2.0f);
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

    printf("STARTDRAW\n"); // DRAW Start Statement.
    static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
    broadcast_open(&broadcast, ANCHOR_RIME_BROADCAST_CHANNEL, &broadcast_call);
    static const struct unicast_callbacks unicast_callbacks = {recv_uc, send_uc};
    unicast_open(&uc, 181, &unicast_callbacks);
    ctimer_set(&localization_timer, LOCALIZATION_TIME, reset_reception_counter, NULL);

    // wait for button event
    while(1) {
        SENSORS_ACTIVATE(button_sensor);
        PROCESS_WAIT_EVENT();
        if(ev == sensors_event && data == &button_sensor){
            printf("MAIN PRC: EVENT triggered localization\n");
            reception_counter = 0;
        }
        SENSORS_DEACTIVATE(button_sensor);
    }
    PROCESS_END();
}

