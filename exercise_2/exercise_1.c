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


// Helper function
long my_fractional_part(float x){
    if(x>=0.0f) {
        return (long) ((x - (long) x)*1000);
    }
    else{
        return (long) 0 - ((x - (long) x)*1000);
    }
}

// Global variables
float x=2.000f;
uint16_t par = 0;
uint16_t tsr = 0;
double T = 0.0f;
double RH = 0.0f;
ANCHOR_RIME_BROADCAST_CHANNEL = 150;
ANCHOR_PHY_CHANNEL = 20;


int16_t pa_to_dbm(int pa_level) {
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

// Etimer declaration
static struct etimer e_timer1;
static struct etimer retransmit_timer;

// Button callback
static void button_callback() {
    printf("BUTT PRESSED!\n");
    leds_toggle(LEDS_ALL);
}

static void print_par_tsr() {
    SENSORS_ACTIVATE(light_sensor);
    par = light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC);
    printf("PAR raw = %d\n", par);
    tsr = light_sensor.value(LIGHT_SENSOR_TOTAL_SOLAR);
    printf("TSR raw = %d\n", tsr);
    SENSORS_DEACTIVATE(light_sensor);
}

static void print_rh_temp() {
    SENSORS_ACTIVATE(sht11_sensor);
    uint16_t temp = sht11_sensor.value(SHT11_SENSOR_TEMP);
    printf("TEMP raw = %d\n", temp);
    uint16_t rh = sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
    printf("RH raw = %d\n", rh);
    // assuming 3v voltage and 14bit readout
    //
    // Temperature in Celsius (t in 14 bits resolution at 3 Volts)
    // T = -39.60 + 0.01*t
    T = -39.6f + 0.01f * temp;
    printf("TEMP Celsius % = %c%ld.%03ld\n", T<0.0f?'-':' ' , (long) abs((long)T), my_fractional_part(T));

    // Relative Humidity in percent (h in 12 bits resolution)
    // RH_lin = -4 + 0.0405*h - 2.8e-6*(h*h)
    double RH_lin = -2.0468 + 0.0367 * rh + -1.5955e-6 * (rh*rh);
    RH = (T - 25)*(0.01 + 0.00008*rh) + RH_lin;
    printf("RH % = %c%ld.%03ld\n", RH<0.0f?'-':' ' , (long) abs((long)RH), my_fractional_part(RH));

    SENSORS_DEACTIVATE(sht11_sensor);
}


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

struct ex2_packet {
    uint32_t sequence_number;
    uint8_t tx_power;
};
struct ex2_packet ex2_packet;

static void print_packet(struct unicast_packet msg) {
    printf("------ Incoming message -------\n");
    printf("temp = %c%ld.%03ld; rh = %c%ld.%03ld;\n",
            msg.temperature<0.0f?'-':' ' , (long) abs((long)msg.temperature), my_fractional_part(msg.temperature),
            msg.humidity<0.0f?'-':' ' , (long) abs((long)msg.humidity), my_fractional_part(msg.humidity));
    printf("par = %d; tsr = %d;\n", msg.par, msg.tsr);
    printf("R = %d; G = %d; B = %d;\n", msg.led_r, msg.led_g, msg.led_b);
}
// Main process
PROCESS(exercise_1, "Exercise 1");
AUTOSTART_PROCESSES(&exercise_1);


static struct unicast_conn uc;
static struct broadcast_conn broadcast;
static uint32_t seq_num = 0;
static uint32_t sent_seq_num = 0;


// UNICAST RECEIVE
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
    printf("unicast message received from %d.%d\n", from->u8[0], from->u8[1]);
    printf("sink signal strength = %d, link quality = %d\n", cc2420_last_rssi, cc2420_last_correlation);
    if (from->u8[0] == 143) {
        struct unicast_packet msg;
        memcpy(&msg, packetbuf_dataptr(), sizeof(msg));
        if (seq_num == msg.sequence_number && !msg.temperature && !msg.humidity
            && !msg.led_r && !msg.led_g && !msg.led_b && !msg.par && !msg.tsr
            && !msg.reserved
            ) {
            printf("SUCCESS: ACK packet is correct!\n");
            seq_num++;
        } else {
            printf("ERROR: received corrupted ACK packet!\n");
        }
    }
}
static void send_uc(struct unicast_conn *c, int status, int tx) {
    printf("Sending Unicast packet!\n");
}
// BROADCAST RECEIVE
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
    printf("broadcast message received from %d.%d: '%s'\n",
           from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
    if (from->u8[0] == 143) {
        struct unicast_packet msg;
        memcpy(&msg, packetbuf_dataptr(), sizeof(msg));
        print_packet(msg);
        printf("sink signal strength = %d, link quality = %d\n", cc2420_last_rssi, cc2420_last_correlation);
        leds_off(LEDS_ALL); // just to be safe
        if (msg.led_r) {
            leds_on(LEDS_RED);
        }
        if (msg.led_g) {
            leds_on(LEDS_GREEN);
        }
        if (msg.led_b) {
            leds_on(LEDS_BLUE);
        }
    } else {
        printf("WARNING: received broadcast message from unknown source!\n");
    }
}

// MAIN THREAD
PROCESS_THREAD(exercise_1, ev, data){
    PROCESS_EXITHANDLER(unicast_close(&uc);broadcast_close(&broadcast);)
    PROCESS_BEGIN();
    cc2420_set_txpower(11);
    cc2420_set_channel(ANCHOR_PHY_CHANNEL);

    leds_off(LEDS_ALL); // just to be safe

    // After 5 seconds print RSSI noise floor and blink blue led
    etimer_set(&e_timer1, (CLOCK_SECOND*5));
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&e_timer1));
    etimer_stop(&e_timer1);
    // measure and print RSSI noise floor
    int noise_floor = cc2420_rssi();
    printf("RSSI noise floor = %d\n", noise_floor);
    leds_toggle(LEDS_BLUE);
    etimer_set(&e_timer1, (CLOCK_SECOND/2));
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&e_timer1));
    etimer_stop(&e_timer1);
    leds_off(LEDS_BLUE);

    static const struct unicast_callbacks unicast_callbacks = {recv_uc, send_uc};

    unicast_open(&uc, 181, &unicast_callbacks);

    static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

    broadcast_open(&broadcast, ANCHOR_RIME_BROADCAST_CHANNEL, &broadcast_call);

    static linkaddr_t addr;
    addr.u8[0] = 143;
    addr.u8[1] = 0;

    // wait for button event
    while(1) {
        SENSORS_ACTIVATE(button_sensor);
        PROCESS_WAIT_EVENT();
        if(ev == sensors_event && data == &button_sensor){
            print_par_tsr();
            print_rh_temp();

            p.sequence_number = seq_num;
            p.x = 105;
            p.y = 342;
            p.temperature = T; //T;
            p.humidity = RH; // RH;
            p.par = par; //par;
            p.tsr = tsr; //tsr;
            p.led_r = 0;
            p.led_g = 0;
            p.led_b = 0;
            p.reserved = 0;
            printf("sending data: %d\n", p.sequence_number);
            sent_seq_num = seq_num;

            // after reception of ACK the seq_num will be incremented
            // until then, retransmit every 0.5 seconds
            while(sent_seq_num == seq_num){
                packetbuf_clear();
                packetbuf_copyfrom(&p, sizeof(struct unicast_packet));
                printf("Going to send to %d %d\n", addr.u8[0], addr.u8[1]);
                if(!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
                  unicast_send(&uc, &addr);
                }
                // Wait for 0.5s before retransmitting
                etimer_set(&retransmit_timer, (CLOCK_SECOND / 2));
                PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&retransmit_timer) || (seq_num > sent_seq_num));
                printf("Retransmitting...\n");
            }
        }
        SENSORS_DEACTIVATE(button_sensor);
    }
    PROCESS_END();
}
