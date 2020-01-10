#ifndef __LPS_TWR_TAG_H__
#define __LPS_TWR_TAG_H__

#include "locodeck.h"
#include "libdw1000.h"
#include "mac.h"

#define LPS_TWR_POLL    0x01  
#define LPS_TWR_ANSWER  0x02
#define LPS_TWR_FINAL   0x03
#define LPS_TWR_REPORT  0x04

#define LPS_TWR_TYPE  0
#define LPS_TWR_SEQ   1
#define LPS_TWR_ENABLE

extern uwbAlgorithm_t uwbTwrTagAlgorithm;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];
} __attribute__((packed)) lpsTwrTagReportPayload_t;

#endif // __LPS_TWR_TAG_H__