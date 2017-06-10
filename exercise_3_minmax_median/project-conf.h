/*
 * Copyright (c) 2015, TU Graz, Austria.
 * All rights reserved.
 *
 * \file
 * File provided for the Location-Aware Computing Labor course.
 *
 * \author
 * Carlo Alberto Boano <cboano@tugraz.at>
*/


#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__


// Default CCA threshold
#undef CC2420_CONF_CCA_THRESH
#define CC2420_CONF_CCA_THRESH   -32

// Default PHY channel
#undef CC2420_CONF_CHANNEL
#define CC2420_CONF_CHANNEL 25

// Default networking stack (no IPv6)
#undef NETSTACK_CONF_WITH_IPV6
#define NETSTACK_CONF_WITH_IPV6 0

// Default MAC layer
#undef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC nullmac_driver //csma_driver

// Default RDC layer
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC nullrdc_driver //contikimac_driver

// Default PHY layer
#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER  framer_802154

// Using clear channel assessment when sending?
#undef WITH_SEND_CCA
#define WITH_SEND_CCA 1


#endif /* __PROJECT_CONF_H__ */
