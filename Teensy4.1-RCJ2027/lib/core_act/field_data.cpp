#include <core_act.h>
#include <DMAChannel.h>

FieldData fieldData;
CameraStats cameraStats;

// Same DMA setup as camera_dma_begin() in core_act.cpp, larger ring for 18-byte packets
#define CAM2_BUF 128 // power of 2, buffer aligned to its size
static volatile uint8_t cam2Rx[CAM2_BUF] __attribute__((aligned(CAM2_BUF)));
static DMAChannel cam2Dma;

void camera_v2_begin() {
  Serial3.begin(CAM_BAUD);
  NVIC_DISABLE_IRQ(IRQ_LPUART2);
  LPUART2_CTRL &= ~(LPUART_CTRL_RIE | LPUART_CTRL_ILIE);
  LPUART2_WATER = LPUART_WATER_RXWATER(0);

  cam2Dma.begin(true);
  cam2Dma.source((volatile const unsigned char&)LPUART2_DATA);
  cam2Dma.destinationCircular(cam2Rx, CAM2_BUF);
  cam2Dma.triggerAtHardwareEvent(DMAMUX_SOURCE_LPUART2_RX);
  cam2Dma.enable();
  LPUART2_BAUD |= LPUART_BAUD_RDMAE;
}

static void decode(FieldObj& o, bool seen, const uint8_t* b, int i) {
  uint16_t a = pkt_u16(b, i);
  o.seen = seen && a < 360;
  if (o.seen) {
    o.angle = a;
    o.dist = pkt_u16(b, i + 2);
  } else {
    o.dist = PKT_NONE;
  }
}

bool read_camera_v2() {
  static uint8_t buf[CAM_LEN];
  static int n = 0;
  static uint32_t tail = 0;
  static bool first = true;
  bool got = false;

  uint32_t head = (uint32_t)cam2Dma.TCD->DADDR - (uint32_t)cam2Rx;
  while (tail != head) {
    uint8_t b = cam2Rx[tail];
    tail = (tail + 1) % CAM2_BUF;
    if (n == 0 && b != CAM_H1) continue;
    if (n == 1 && b != CAM_H2) { n = (b == CAM_H1); continue; }
    buf[n++] = b;
    if (n < CAM_LEN) continue;
    n = 0;

    if (buf[CAM_LEN - 1] != CAM_END || pkt_sum(buf, 1, 15) != buf[16]) {
      cameraStats.bad++;
      continue;
    }
    uint8_t flags = buf[2];
    decode(fieldData.ball,       flags & CAM_FLAG_BALL,  buf, 3);
    decode(fieldData.front_goal, flags & CAM_FLAG_FRONT, buf, 7);
    decode(fieldData.back_goal,  flags & CAM_FLAG_BACK,  buf, 11);

    uint8_t seq = buf[15];
    if (!first) cameraStats.lost += (uint8_t)(seq - fieldData.seq - 1);
    first = false;
    fieldData.seq = seq;
    fieldData.t_ms = millis();
    cameraStats.pkts++;
    got = true;
  }
  return got;
}

bool camera_fresh(uint32_t max_age_ms) {
  return fieldData.t_ms != 0 && millis() - fieldData.t_ms <= max_age_ms;
}
