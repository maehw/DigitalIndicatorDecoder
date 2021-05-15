/**
 * Digital indicator decoder
 */

#include "config.h"

const byte protocolReadPin         = PIN_PROTOCOL_READ;
const byte samplingIndicatorPin    = PIN_SAMPLING_INDICATOR;
const byte processingIndicatorPin  = PIN_PROC_INDICATOR;
const byte newMeasIndPin           = PIN_NEW_MEAS_INDICATOR;

unsigned long timeStart            = 0;
unsigned long timeFirstFallingEdge = 0;
unsigned long timeStop             = 0;
unsigned long timeStartToEdge      = 0;
unsigned long timeStartToStop      = 0;

volatile bool bNewMeasurementDetected = false;
volatile bool bSamplingIndPinState    = false;
volatile bool bNewMeasIndPinState     = false;

volatile unsigned long timeEdge[2][60] = { 0 }; /* 2D storage for sampling timestamps of falling edges */
volatile byte nFillBuffIdx = 0; /* sampling buffer index for buffer that is filled in the ISR (sampling), i.e. first dimension */
volatile byte nWriteIndex = 0; /* sampling buffer index for second dimension */
volatile byte nLastProcessIndex = 0; /* indicate position of last valid sample/timestamp */
byte nRawData[6] = { 0 }; /* the reaw measurement data consists of 6 bytes */

void setup()
{
  Serial.begin(SERIAL_BAUDRATE);

#ifdef DEBUG_SIGNALS
  pinMode(samplingIndicatorPin, OUTPUT);
  digitalWrite(samplingIndicatorPin, LOW);
  pinMode(processingIndicatorPin, OUTPUT);
  digitalWrite(processingIndicatorPin, LOW);
  pinMode(newMeasIndPin, OUTPUT);
  digitalWrite(newMeasIndPin, LOW);
#endif

  pinMode(protocolReadPin, INPUT);
  bNewMeasurementDetected = false;

  EIFR |= (1 << INTF1) | (1 << INTF0); /* make sure all pin interrupts have been cleared */
  attachInterrupt(digitalPinToInterrupt(protocolReadPin), fallingEdgeDetected, FALLING);
}

byte getNextZerobitPos(byte nCurrentZerobitPos, byte nIdx)
{
  byte nZerobitPos = nCurrentZerobitPos;
  unsigned long timeQuery = timeEdge[1-nFillBuffIdx][nIdx];
  unsigned long timePrevious = timeEdge[1-nFillBuffIdx][nIdx-1];
  unsigned long timeQryMgn = 0;

  DEBUG_PRINT("e#");
  DEBUG_PRINT(nIdx);
  DEBUG_PRINT(",q=");
  DEBUG_PRINT(timeQuery);
  DEBUG_PRINT(",p=");
  DEBUG_PRINTLN(timePrevious);

  while( nZerobitPos < 8*6 )
  {
    if( nZerobitPos & 0x1 ) /* check LSB of position: odd bit positions (1, 3, 5, 7, 9, 11, 13, 15, ...) */
    {
      /* check byte crossing (i.e. 7->8, 15->16, 23->24, 31->32, 39->40) */
      if( (nZerobitPos & 0x7) == 0x7 )
      {
        if( nZerobitPos == 7 ) /* the very first byte crossing has a higher interval duration */
        {
          timeQuery -= INTVAL_1ST_BYTEXING;
          DEBUG_PRINT(",Z");
        }
        else
        {
          timeQuery -= INTVAL_BYTEXING;
          DEBUG_PRINT(",X");
        }
      }
      else
      {
        timeQuery -= INTVAL_EVN;
        DEBUG_PRINT(",E");
      }
    }
    else /* even bit position (0, 2, 4, 6, 8, ...) */
    {
      timeQuery -= INTVAL_ODD;
      DEBUG_PRINT(",O");
    }

    timeQryMgn = timeQuery - SAFETY_MARGIN;
    DEBUG_PRINT(",tm=");
    DEBUG_PRINT(timeQryMgn);
    if( timePrevious > timeQryMgn )
    {
      /* found new zero bit position */
      return nZerobitPos+1;
    }
    nZerobitPos++;
  }
  DEBUG_PRINT(",H");
  return 0;
}

/* the measurement in millimeters consists of a 12 bit unsigned value plus an additional sign bit;
 * TODO: improve sanity-checking of neighbour bits
 */
int16_t rawToMillimeters(byte nRawData[6])
{
  int16_t nRaw = 0;

  if( nRawData[0] & BIT_0 ) /* unit must be millimeters; currently that's an assumption & requirement to make it work */
  {
    return MEAS_MM_INVALID;
  }
  if( (nRawData[5] & BIT_7) != BIT_7 ) /* MSBit must always be set */
  {
    return MEAS_MM_INVALID;
  }

  if( (nRawData[5] & BITS_6n5) == BITS_6n5 )
  {
    nRaw |= BIT_0;
  }
  if( (nRawData[5] & BITS_4n3) == BITS_4n3 )
  {
    nRaw |= BIT_1;
  }
  if( (nRawData[5] & BITS_2n1) == BITS_2n1 )
  {
    nRaw |= BIT_2;
  }
  if( ( nRawData[5] & BIT_0 ) && ( nRawData[4] & BIT_7 ) )
  {
    nRaw |= BIT_3;
  }
  if( (nRawData[4] & BITS_6n5) == BITS_6n5 )
  {
    nRaw |= BIT_4;
  }
  if( (nRawData[4] & BITS_4n3) == BITS_4n3 )
  {
    nRaw |= BIT_5;
  }
  if( (nRawData[4] & BITS_2n1) == BITS_2n1 )
  {
    nRaw |= BIT_6;
  }
  if( ( nRawData[4] & BIT_0 ) && ( nRawData[3] & BIT_7 ) )
  {
    nRaw |= BIT_7;
  }
  if( (nRawData[3] & BITS_6n5) == BITS_6n5 )
  {
    nRaw |= BIT_8;
  }
  if( (nRawData[3] & BITS_4n3) == BITS_4n3 )
  {
    nRaw |= BIT_9;
  }
  if( (nRawData[3] & BITS_2n1) == BITS_2n1 )
  {
    nRaw |= BIT_10;
  }
  if( ( nRawData[3] & BIT_0 ) && ( nRawData[2] & BIT_7 ) )
  {
    nRaw |= BIT_11;
  }

  if( (nRawData[0] & BITS_6n5) == BITS_6n5 ) /* sign bit(s) */
  {
    nRaw = -nRaw;
  }

  return nRaw;
}
    
void loop()
{
  char sHexString[20];
  unsigned long timeFirst = 0;
  unsigned long timeLast  = 0;
  unsigned long timeDiff  = 0;
  byte nAnalysisByteIdx   = 0;
  byte nAnalysisBitIdx    = 0;
  bool bDecodingErr = false;
  float fMeas = 0.0f;

  if( bNewMeasurementDetected )
  {
#ifdef DEBUG_SIGNALS
    digitalWrite(processingIndicatorPin, HIGH);
#endif

    if( nLastProcessIndex <= 2 )
    {
      Serial.println("Error: nLastProcessIndex too low.");
    }

    timeFirst = timeEdge[1-nFillBuffIdx][0]; /* timestamp of first edge in current measurement */
    timeLast = timeEdge[1-nFillBuffIdx][nLastProcessIndex];  /* timestamp of last edge in current measurement */

    DEBUG_PRINT("nEs: ");
    DEBUG_PRINT(nLastProcessIndex+1);
    DEBUG_PRINT(", 1st: ");
    DEBUG_PRINT(timeFirst);
    DEBUG_PRINT(", last: ");
    DEBUG_PRINT(timeLast);
    DEBUG_PRINT(", dur: ");
    DEBUG_PRINTLN(timeLast - timeFirst);

    /* decode bits from time differences */
    memset(&nRawData[0], 0xFF, 6); /* initialize with all ones */
    nRawData[0] = 0xFE; /* set LSBit to zero (also LSByte first order in the array) */

    byte nZerobitPos = 0;
    bDecodingErr = false;

    for( byte nIdx = nLastProcessIndex; nIdx > 0; nIdx-- )
    {
      nZerobitPos = getNextZerobitPos(nZerobitPos, nIdx);
      if( 0 == nZerobitPos )
      {
        DEBUG_PRINTLN(" ERR: Unable to find next pos of zero bit, aborting. ");
        bDecodingErr = true;
        break;
      }
      else
      {
        if( nZerobitPos >= 8*6 )
        {
          DEBUG_PRINT(" ERR: Bit pos (");
          DEBUG_PRINT(nZerobitPos);
          DEBUG_PRINTLN(") too high, aborting. ");
          bDecodingErr = true;
          break;
        }
        else
        {
          /* found valid position for zero bit: clear bit from storage */
          byte nByteIdx = nZerobitPos >> 3;
          byte nBitIdx = nZerobitPos & 0x7;
          nRawData[nByteIdx] &= ~(1 << nBitIdx);

          DEBUG_PRINT(" I: New bit pos #");
          DEBUG_PRINT(nZerobitPos);
          DEBUG_PRINT(" (by=");
          DEBUG_PRINT(nByteIdx);
          DEBUG_PRINT(",bi=");
          DEBUG_PRINT(nBitIdx);
          DEBUG_PRINTLN("). ");
        }
      }
    }
    DEBUG_PRINTLN("done.");

    int16_t nMeas = rawToMillimeters(&nRawData[0]);    

    if( bDecodingErr )
    {
#ifdef DEBUG_PRINTS
      sprintf(sHexString, "[ERR] %02X%02X%02X%02X%02X%02X", nRawData[5], nRawData[4], nRawData[3], nRawData[2], nRawData[1], nRawData[0]);
      DEBUG_PRINTLN(sHexString);
#endif
    }
    else if( MEAS_MM_INVALID == nMeas )
    {
#ifdef DEBUG_PRINTS
      sprintf(sHexString, "[INV] %02X%02X%02X%02X%02X%02X", nRawData[5], nRawData[4], nRawData[3], nRawData[2], nRawData[1], nRawData[0]);
      DEBUG_PRINTLN(sHexString);
#endif
    }
    else
    {
#ifdef DEBUG_PRINTS
      sprintf(sHexString, "[ OK] %02X%02X%02X%02X%02X%02X", nRawData[5], nRawData[4], nRawData[3], nRawData[2], nRawData[1], nRawData[0]);
      DEBUG_PRINT(sHexString);
      DEBUG_PRINT("; ");
      DEBUG_PRINT(nMeas);
      DEBUG_PRINTLN(" mm*100; ");
#endif

      /* provide productive output as number with two decimals */
      fMeas = (float)nMeas/(float)100.0f;
      Serial.print(fMeas, 2);
      Serial.println(" mm");
    }

    bNewMeasurementDetected = false;

#ifdef DEBUG_SIGNALS
    digitalWrite(processingIndicatorPin, LOW);
#endif
  }
}

void fallingEdgeDetected()
{
  /* get timestamp of falling edge which has caused the interrupt */
  unsigned long newval = micros();

#ifdef DEBUG_SIGNALS
  /* toggle sampling indicator pin */
  bSamplingIndPinState = !bSamplingIndPinState;
  digitalWrite(samplingIndicatorPin, bSamplingIndPinState ? HIGH : LOW);
#endif

  /* store timestamp */
  timeEdge[nFillBuffIdx][nWriteIndex] = newval;

  if( nWriteIndex >= 1 )
  {
    /* check time since last edge */
    if( (newval - timeEdge[nFillBuffIdx][nWriteIndex-1] ) > MEAS_MIN_PAUSE )
    {
      /* edge cannot be part of current measurement, so finish it and mark it for processing */
      bNewMeasurementDetected = true;
      nLastProcessIndex = nWriteIndex-1;

      /* switch buffers, save first sample, reset index */
      nFillBuffIdx = 1 - nFillBuffIdx;
      timeEdge[nFillBuffIdx][0] = newval;
      nWriteIndex = 0; /* it is known that this value will be incremented to 1 shortly... */

#ifdef DEBUG_SIGNALS
      /* toggle new measurement indicator pin */
      bNewMeasIndPinState = !bNewMeasIndPinState;
      digitalWrite(newMeasIndPin, bNewMeasIndPinState ? HIGH : LOW);
#endif
    }
  }
  nWriteIndex++;
}

