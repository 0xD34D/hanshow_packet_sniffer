#pragma once

#define ON  1
#define OFF 0

#ifndef MCU_CORE_8258
#define MCU_CORE_8258 1
#endif

#define RAM _attribute_data_retention_
#define BOARD_8258_DIY 10
#define BOARD BOARD_8258_DIY
#define CLOCK_SYS_CLOCK_HZ 24000000

#include "version_cfg.h"

typedef enum
{
    EV_POLL_ED_DETECT,
    EV_POLL_HCI,
    EV_POLL_IDLE,
    EV_POLL_MAX,
} ev_poll_e;
