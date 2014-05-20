// brands
#define PHILIPSTV 1

#define MAXBUTTONS 30

// user configuration
#define SYSCLOCK 16000000  // main system clock (Hz)
#define PRESCALE 8       // TIMER1 prescale value (state machine clock)

// Philips TV defines
#define PHILIPSTV_PULSECLOCK 36000  // Hz

#define PHILIPSTV_T           889  // T, microseconds
#define PHILIPSTV_RETRANSGAP  128  // gap beween code retransmissions (T units)

#define PHILIPSTV_UNITLENGTH 5   // bits in unit code
#define PHILIPSTV_BUTTONLENGTH 6 // bits in button code

#define PHILIPSTV_UNIT 0x0 // unit code for Philips TV

// buttons
#define PHILIPSTV_B01 POWER
#define PHILIPSTV_C01 0x0c
#define PHILIPSTV_B02 MUTE
#define PHILIPSTV_C02 0x0d
#define PHILIPSTV_B03 DASH
#define PHILIPSTV_C03 0x0a
#define PHILIPSTV_B04 ZERO
#define PHILIPSTV_C04 0x0
#define PHILIPSTV_B05 ONE
#define PHILIPSTV_C05 0x1
#define PHILIPSTV_B06 TWO
#define PHILIPSTV_C06 0x2
#define PHILIPSTV_B07 THREE
#define PHILIPSTV_C07 0x3
#define PHILIPSTV_B08 FOUR
#define PHILIPSTV_C08 0x4
#define PHILIPSTV_B09 FIVE
#define PHILIPSTV_C09 0x5
#define PHILIPSTV_B10 SIX
#define PHILIPSTV_C10 0x6
#define PHILIPSTV_B11 SEVEN
#define PHILIPSTV_C11 0x7
#define PHILIPSTV_B12 EIGHT
#define PHILIPSTV_C12 0x8
#define PHILIPSTV_B13 NINE
#define PHILIPSTV_C13 0x9
#define PHILIPSTV_B14 VOLUP
#define PHILIPSTV_C14 0x10
#define PHILIPSTV_B15 VOLDOWN
#define PHILIPSTV_C15 0x11
#define PHILIPSTV_B16 CHANUP
#define PHILIPSTV_C16 0x20
#define PHILIPSTV_B17 CHANDOWN
#define PHILIPSTV_C17 0x21
#define PHILIPSTV_B18 BRIGHTUP
#define PHILIPSTV_C18 0x20
#define PHILIPSTV_B19 BRIGHTDOWN
#define PHILIPSTV_C19 0x21
// END Philips TV defines

// button labels
#define ZERO         0
#define ONE          1
#define TWO          2
#define THREE        3
#define FOUR         4
#define FIVE         5
#define SIX          6
#define SEVEN        7
#define EIGHT        8
#define NINE         9
#define POWER        10
#define POWEROFF     11
#define MUTE         12
#define DASH         13
#define PLAY         14
#define VOLUP        15
#define VOLDOWN      16
#define CHANUP       17
#define CHANDOWN     18
#define BRIGHTUP     19
#define BRIGHTDOWN   20
