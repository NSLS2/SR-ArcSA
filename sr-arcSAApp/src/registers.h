#ifndef REGISTERS_H
#define REGISTERS_H

#define NUM_CHANNELS           16
#define	N_WOVALS               14

#define EP_LENGTH               2
#define EP_CONTROL              3

#define CONTROL_DEBUG_mask 0x0070
#define CONTROL_DEBUG_shft      4

#define EP_1WIRE_CMD           10

#define EP_ACTIONS           0x40
#define ACTIONS_RESET		0
#define ACTIONS_UPDATE_DS1825  15

#define EP_1WIRE_RES_0       0x3a
#define CTL_DAC_RD         0x1000

#define CTL_DAC_WR_mask    0x0c00
#define CTL_DAC_WR_shft	       10

#endif // REGISTERS_H
