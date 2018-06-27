#ifdef KILOBOT 
#include <avr/io.h>  // for microcontroller register defs
#endif

#include "kilombo.h"
#include "util.h"

void smooth_set_motors(uint8_t ccw, uint8_t cw) {
#ifdef KILOBOT 
  uint8_t l = 0, r = 0;
  if (ccw && !OCR2A) // we want left motor on, and it's off
    l = 0xff;
  if (cw && !OCR2B)  // we want right motor on, and it's off
    r = 0xff;
  if (l || r)        // at least one motor needs spin-up
    {
      set_motors(l, r);
      delay(15);
    }
#endif

  // spin-up is done, now we set the real value
  set_motors(ccw, cw);
}


void motion(uint8_t type)
{
  switch (type)
    {
    case STOP:
      smooth_set_motors(0, 0);
      break;
    case LEFT:
      smooth_set_motors(kilo_turn_left, 0); 
      break;
    case RIGHT:
      smooth_set_motors(0, kilo_turn_right);
      break;
    case STRAIGHT:
      smooth_set_motors(kilo_straight_left, kilo_straight_right);
      break;
    }
}


