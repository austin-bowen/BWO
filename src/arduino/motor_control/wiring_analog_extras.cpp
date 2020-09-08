/* TODO: This header. */


#include "wiring_analog_extras.h"
#include <wiring_analog.c>


#ifdef __cplusplus
extern "C" {
#endif


// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_8(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_8(Tc* TCx) {
  while (TCx->COUNT8.STATUS.bit.SYNCBUSY);
}


/* Right now, PWM output only works on the pins with
   hardware support.  These are defined in the appropriate
   pins_*.c file.  For the rest of the pins, we just return.

   This was copied from wiring_analog.c and modified to
   1) remove code not relevant to the Seeeduino XIAO, and
   2) set the PWM frequency to 20kHz.

   Args:
     value - Range is [0, 255], like analogWrite(), but the actual resolution
             is only [0, 149], so the value will be scaled down.
*/
void analogWrite20kHz(const uint32_t pin, uint32_t value)
{
  /*
    From the SAMD21 datasheet, page 642:

                  f{GCLK_TC}
      f{PWM_SS} = ----------
                  N(TOP + 1)

    Where:
    - f{PWM_SS} is the PWM frequency.
    - f{GCLK_TC} is the clock frequency (48MHz, in this case).
    - N is the prescaler value (16, in this case).
    - TOP is the max counter value (149, in this case).

    This gives f{PWM_SS} = 48MHz / (16(149 + 1)) = 20kHz.
  */

  const static int TOP = 149;

  PinDescription pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;

  if (!(attr & PIN_ATTR_PWM))
  {
    return;
  }

  value = map(constrain(value, 0, 255), 0, 255, 0, TOP);

  uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
  uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
  static bool tcEnabled[TCC_INST_NUM + TC_INST_NUM];

  if (attr & PIN_ATTR_TIMER)
  {
    pinPeripheral(pin, PIO_TIMER);
  }
  else if ((attr & PIN_ATTR_TIMER_ALT) == PIN_ATTR_TIMER_ALT)
  {
    //this is on an alt timer
    pinPeripheral(pin, PIO_TIMER_ALT);
  }
  else
  {
    return;
  }

  if (!tcEnabled[tcNum])
  {
    tcEnabled[tcNum] = true;
    uint16_t GCLK_CLKCTRL_IDs[] = {
      GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
      GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
      GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
      GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
      GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
      GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5
      GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC6
      GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC7
    };
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
    while (GCLK->STATUS.bit.SYNCBUSY == 1);

    // Set PORT
    if (tcNum >= TCC_INST_NUM)
    {
      // -- Configure TC
      Tc *TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
      // Disable TCx
      TCx->COUNT8.CTRLA.bit.ENABLE = 0;
      syncTC_8(TCx);
      // Set Timer counter Mode to 8 bits, normal PWM, with prescale set to 1/16
      TCx->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_WAVEGEN_NPWM | TC_CTRLA_PRESCALER(TC_CTRLA_PRESCALER_DIV16_Val);
      syncTC_8(TCx);
      // Set PER to a maximum counter value of TOP
      TCx->COUNT8.PER.reg = TOP;
      syncTC_8(TCx);
      // Set the initial value
      TCx->COUNT8.CC[tcChannel].reg = (uint32_t) value;
      syncTC_8(TCx);
      // Enable TCx
      TCx->COUNT8.CTRLA.bit.ENABLE = 1;
      syncTC_8(TCx);
    }
    else
    {
      // -- Configure TCC
      Tcc *TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
      // Disable TCCx
      TCCx->CTRLA.bit.ENABLE = 0;
      syncTCC(TCCx);

      TCCx->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV16_Val);
      syncTCC(TCCx);

      // Set TCCx as normal PWM
      TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
      syncTCC(TCCx);
      // Set the initial value
      TCCx->CC[tcChannel].reg = (uint32_t) value;
      syncTCC(TCCx);
      // Set PER to a maximum counter value of TOP
      TCCx->PER.reg = TOP;
      syncTCC(TCCx);
      // Enable TCCx
      TCCx->CTRLA.bit.ENABLE = 1;
      syncTCC(TCCx);
    }
  }
  else
  {
    if (tcNum >= TCC_INST_NUM)
    {
      Tc *TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
      TCx->COUNT8.CC[tcChannel].reg = (uint32_t) value;
      syncTC_8(TCx);
    }
    else
    {
      Tcc *TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
      TCCx->CTRLBSET.bit.LUPD = 1;
      syncTCC(TCCx);
      TCCx->CCB[tcChannel].reg = (uint32_t) value;
      syncTCC(TCCx);
      TCCx->CTRLBCLR.bit.LUPD = 1;
      syncTCC(TCCx);
    }
  }
}


#ifdef __cplusplus
}
#endif
