#pragma once
#include <stdint.h>
#include "TxTypes.h"

// Example model list. Replace the names and MAC addresses with your receivers,
// then copy this block into your local secrets.h.
static const TxModel modelList[] = {
  {"Example model 1", {0x02, 0x00, 0x00, 0x00, 0x00, 0x01}},
  {"Example model 2", {0x02, 0x00, 0x00, 0x00, 0x00, 0x02}},
};
