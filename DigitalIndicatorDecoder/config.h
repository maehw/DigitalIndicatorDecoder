#define INTVAL_ODD              (156u) /* ca. 117.2 us HIGH + ca. 39 us LOW */
#define INTVAL_EVN              (117u) /* ca.  78.2 us HIGH + ca. 39 us LOW */
#define INTVAL_BYTEXING         (391u) /* ca. 351.8 us HIGH + ca. 39 us LOW */
#define INTVAL_1ST_BYTEXING     (430u) /* ca. 390.6 us HIGH + ca. 39 us LOW */
#define SAFETY_MARGIN           (25u)  /* additional offset that the signal may already have been LOW earlier */
#define MEAS_MIN_PAUSE          (80000u) /* minimum pause between two measurements in us */
#define MEAS_MM_INVALID         (INT16_MIN) /* define value that is out of the measurement range to signal error in-band */

#define PIN_PROTOCOL_READ       (2)    /* hint: pin used for interrupt to detect falling edges (not all pins can be used for interrupts on all platforms!) */
#define PIN_SAMPLING_INDICATOR  (4)
#define PIN_PROC_INDICATOR      (7)
#define PIN_NEW_MEAS_INDICATOR  (8)

#define BIT_0                   (0x01)
#define BIT_1                   (0x02)
#define BIT_2                   (0x04)
#define BIT_3                   (0x08)
#define BIT_4                   (0x10)
#define BIT_5                   (0x20)
#define BIT_6                   (0x40)
#define BIT_7                   (0x80)
#define BIT_8                   (0x100)
#define BIT_9                   (0x200)
#define BIT_10                  (0x400)
#define BIT_11                  (0x800)
#define BITS_6n5                (0x60)
#define BITS_4n3                (0x18)
#define BITS_2n1                (0x06)

#define SERIAL_BAUDRATE         (230400u)

//#define DEBUG_PRINTS
//#define DEBUG_SIGNALS

#ifdef DEBUG_PRINTS
  #define DEBUG_PRINT           Serial.print
  #define DEBUG_PRINTLN         Serial.println
#else
  #define DEBUG_PRINT           
  #define DEBUG_PRINTLN         
#endif
