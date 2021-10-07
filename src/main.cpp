#include <Arduino.h>
#include <MIDIUSB.h>

#include <CircularBuffer.h>

// #define NDEBUG

#include <cassert>

#include <pure_adc.h>
#include <pure_midictrl.h>
#include <pure_midiqueue.h>
#include <pure_task.h>
#include <pure_velocitykeybed.h>

//////////////////////////////////////////////
// Debug
//////////////////////////////////////////////

#define PURE_DEBUG

void debugPrint(char *c)
{
#if defined(PURE_DEBUG)
	SerialUSB.print(c);
#endif
}

void debugPrint(int i)
{
#if defined(PURE_DEBUG)
	SerialUSB.print(i);
#endif
}

void debugPrint(uint16_t i)
{
#if defined(PURE_DEBUG)
	SerialUSB.print(i);
#endif
}

void debugPrint(double f)
{
#if defined(PURE_DEBUG)
	SerialUSB.print(f);
#endif
}

void debugPrintLn(char *c)
{
#if defined(PURE_DEBUG)
	SerialUSB.println(c);
#endif
}

void debugPrintLn(int i)
{
#if defined(PURE_DEBUG)
	SerialUSB.println(i);
#endif
}

void debugPrintLn(uint16_t i)
{
#if defined(PURE_DEBUG)
	SerialUSB.println(i);
#endif
}

void debugPrintLn(double d)
{
#if defined(PURE_DEBUG)
	SerialUSB.println(d);
#endif
}

//////////////////////////////////////////////
// MIDI and buffers
//////////////////////////////////////////////

// MIDI CC numbers

const int highestMsbCcNumber = 31; // The highest CC number that implement a 7-bit MSB CC message.
const int lowestLsbCcNumber = 32;  // The lowest CC number that implement a 7-bit LSB CC message (related to the corresponding MSB message)
const int highestCcNumber = 127;

const int ccNumModulationMSB = 1;
const int ccNumModulation = ccNumModulationMSB;
const int ccNumModulationLSB = ccNumModulationMSB + 32;

const int ccNumGeneralPurpose1MSB = 16;
const int ccNumGeneralPurpose1 = ccNumGeneralPurpose1MSB;
const int ccNumGeneralPurpose1LSB = ccNumGeneralPurpose1MSB + 32;

const int ccNumGeneralPurpose2MSB = 17;
const int ccNumGeneralPurpose2 = ccNumGeneralPurpose2MSB;
const int ccNumGeneralPurpose2LSB = ccNumGeneralPurpose2MSB + 32;

const int ccNumGeneralPurpose3MSB = 18;
const int ccNumGeneralPurpose3 = ccNumGeneralPurpose3MSB;
const int ccNumGeneralPurpose3LSB = ccNumGeneralPurpose3MSB + 32;

const int ccNumGeneralPurpose4MSB = 19;
const int ccNumGeneralPurpose4 = ccNumGeneralPurpose4MSB;
const int ccNumGeneralPurpose4LSB = ccNumGeneralPurpose4MSB + 32;

// MIDI CC handling

bool gcEnable14BitCc = false; // Global configuration ("gc") to enable 14 bit CC handling. If disabled 7 bit CC handling will be used.

// MIDI data structures and functions

union midiPacket4_t
{
	uint32_t data32bit;
	uint8_t data8bit[4];
};

CircularBuffer<uint32_t, 100> midiBuffer4;

struct adcToCtrlMap_t
{
	static const int maxNumAdcRanges = 10;						  // The highest possible number of ADC ranges that can be defined in any adcToCtrlMap_t object.
	static const int maxNumAdcRangeBorders = maxNumAdcRanges + 1; // N range values yields N+1 range border values. E.g. val0..val1..val2..val3 defines 3 ranges and requires 4 range border values.
	int numAdcRanges = 0;										  // Current number of ADC ranges
	int numAdcRangeBorders = 0;									  // Current number of ADC range borders. Should be one more than number of ADC ranges. TODO: Is this variable unnecessary as it can be deduced?
	uint16_t adcRangeBorder[maxNumAdcRangeBorders];				  // Dynamically defines the current list of ADC ranges [adcRangeBorder[0]..adcRangeBorder[1]] , [adcRangeBorder[1]..adcRangeBorder[2]] , ... , [adcRangeBorder[numAdcRangeBorders-2]..adcRangeBorder[numAdcRangeBorders-1]]
	uint16_t ctrlRangeBorderHighest = 0;						  // Dynamically defines the currently highest border of the controller ranges.
	double k[maxNumAdcRanges];									  // Slope of the linear equation that maps ADC values in the ranges defined by adcRangeBorder[] to controller values.
	uint16_t m[maxNumAdcRanges];								  // Y-intercept of the linear equation that maps ADC values in the ranges defined by adcRangeBorder[] to controller values.
};

void printCtrlMap(adcToCtrlMap_t *aTCM)
{
	debugPrintLn("vv-Dumping adcToCtrlMap:");
	debugPrint("maxNumAdcRanges: ");
	debugPrintLn(aTCM->maxNumAdcRanges);
	debugPrint("maxNumAdcRangeBorders: ");
	debugPrintLn(aTCM->maxNumAdcRangeBorders);
	debugPrint("numAdcRanges: ");
	debugPrintLn(aTCM->numAdcRanges);
	debugPrint("numAdcRangeBorders: ");
	debugPrintLn(aTCM->numAdcRangeBorders);
	// debugPrint("adcRangeBorder[]: ");
	for (int i = 0; i < aTCM->maxNumAdcRangeBorders; i++)
	{
		debugPrint("adcRangeBorder[");
		debugPrint(i);
		debugPrint("]: ");
		debugPrintLn(aTCM->adcRangeBorder[i]);
	}
	debugPrint("ctrlRangeBorderHighest: ");
	debugPrintLn(aTCM->ctrlRangeBorderHighest);
	for (int i = 0; i < aTCM->maxNumAdcRanges; i++)
	{
		debugPrint("k[");
		debugPrint(i);
		debugPrint("]: ");
		debugPrintLn(aTCM->k[i]);
	}
	for (int i = 0; i < aTCM->maxNumAdcRanges; i++)
	{
		debugPrint("m[");
		debugPrint(i);
		debugPrint("]: ");
		debugPrintLn(aTCM->m[i]);
	}
	debugPrintLn("^^-Done dumping adcToCtrlMap");
}

const int atcmIxPitchbend = 0;
const int numAdcToCtrlMapArrElements = 16; // Defines the maximum number of different ADC value to MIDI controller value mappings that can be done simultaneously.
adcToCtrlMap_t adcToCtrlMapArr[numAdcToCtrlMapArrElements];

int atcmArrIxPerCC[highestCcNumber];

struct adcCtrlPair_t
{
	uint16_t adcValue;
	uint16_t ctrlValue;
};
typedef adcCtrlPair_t borderList_t[adcToCtrlMap_t::maxNumAdcRangeBorders]; // Pointer to a list of adcCtrlPair_t elements

// setAdcToCtrlMap1(): Set map with one (1) range (and consequently two (2) borders)
//
// adcValue0 < adcValue1
//
// ctrlValue0 <= ctrlValue1
//
//  * ADC values <= adcValue0 are mapped to ctrlValue0
//  * ADC values >= adcValue1 are mapped to ctrlValue1
//  * ADC values in the range [adcValue0..adcValue1] are mapped linearly to the range [ctrlValue0..ctrlValue1]
//
// Linear mapping is done by using the well known formula (y = k*x + m). Since the full resulting mapping is a sequence of (one or more) linear segments,
// residing inside their given (sub)range, the parameters k and m must be calculated in such a way that the segments meet on each border between ranges.

void setAdcToCtrlMap1(adcToCtrlMap_t *aTCM, uint16_t adcBorder0, uint16_t ctrlBorder0, uint16_t adcBorder1, uint16_t ctrlBorder1)
{
	assert(aTCM);
	aTCM->numAdcRanges = 1;
	aTCM->numAdcRangeBorders = aTCM->numAdcRanges + 1;
	aTCM->adcRangeBorder[0] = adcBorder0;
	aTCM->m[0] = ctrlBorder0;
	aTCM->k[0] = static_cast<float>(ctrlBorder1 - ctrlBorder0) / (adcBorder1 - adcBorder0); //k = (y1 - y0) / (x1 - x0)
	aTCM->adcRangeBorder[1] = adcBorder1;
	aTCM->ctrlRangeBorderHighest = ctrlBorder1;
}

// setAdcToCtrlMap3(): Set map with three (3) ranges (and consequently four (4) borders)
//
// adcValue0 < adcValue1 < adcValue2 < adcValue3
//
// ctrlValue0 <= ctrlValue1 <= ctrlValue2 <= ctrlValue3
//
//  * ADC values <= adcValue0 are mapped to ctrlValue0
//  * ADC values >= adcValue3 are mapped to ctrlValue3
//  * ADC values in the range [adcValue0..adcValue1] are mapped linearly to the range [ctrlValue0..ctrlValue1]
//  * ADC values in the range [adcValue1..adcValue2] are mapped linearly to the range [ctrlValue1..ctrlValue2]
//  * ADC values in the range [adcValue2..adcValue3] are mapped linearly to the range [ctrlValue2..ctrlValue3]
//
// Linear mapping is done by using the well known formula (y = k*x + m). Since the full resulting mapping is a sequence of (one or more) linear segments, residing inside their given (sub)range, the parameters k and m must be calculated in such a way that the segments meet on each border between ranges.

void setAdcToCtrlMap3(adcToCtrlMap_t *aTCM, uint16_t adcBorder0, uint16_t ctrlBorder0, uint16_t adcBorder1, uint16_t ctrlBorder1, uint16_t adcBorder2, uint16_t ctrlBorder2, uint16_t adcBorder3, uint16_t ctrlBorder3)
{
	assert(aTCM);
	aTCM->numAdcRanges = 3;
	aTCM->numAdcRangeBorders = aTCM->numAdcRanges + 1;
	aTCM->adcRangeBorder[0] = adcBorder0;
	aTCM->m[0] = ctrlBorder0;
	aTCM->k[0] = static_cast<float>(ctrlBorder1 - ctrlBorder0) / (adcBorder1 - adcBorder0); //k = (y1 - y0) / (x1 - x0)
	aTCM->adcRangeBorder[1] = adcBorder1;
	aTCM->m[1] = ctrlBorder1;
	aTCM->k[1] = static_cast<float>(ctrlBorder2 - ctrlBorder1) / (adcBorder2 - adcBorder1); //k = (y2 - y1) / (x2 - x1)
	aTCM->adcRangeBorder[2] = adcBorder2;
	aTCM->m[2] = ctrlBorder2;
	aTCM->k[2] = static_cast<float>(ctrlBorder3 - ctrlBorder2) / (adcBorder3 - adcBorder2); //k = (y3 - y2) / (x3 - x2)
	aTCM->adcRangeBorder[3] = adcBorder3;
	aTCM->ctrlRangeBorderHighest = ctrlBorder3;
}

// The max case with 10 ranges (11 borders)...

void setAdcToCtrlMap10(adcToCtrlMap_t *aTCM,
					   uint16_t adcBorder0, uint16_t ctrlBorder0,
					   uint16_t adcBorder1, uint16_t ctrlBorder1,
					   uint16_t adcBorder2, uint16_t ctrlBorder2,
					   uint16_t adcBorder3, uint16_t ctrlBorder3,
					   uint16_t adcBorder4, uint16_t ctrlBorder4,
					   uint16_t adcBorder5, uint16_t ctrlBorder5,
					   uint16_t adcBorder6, uint16_t ctrlBorder6,
					   uint16_t adcBorder7, uint16_t ctrlBorder7,
					   uint16_t adcBorder8, uint16_t ctrlBorder8,
					   uint16_t adcBorder9, uint16_t ctrlBorder9,
					   uint16_t adcBorder10, uint16_t ctrlBorder10)
{
	assert(aTCM);
	aTCM->numAdcRanges = 10;
	aTCM->numAdcRangeBorders = aTCM->numAdcRanges + 1;
	// border 0 (first border)
	aTCM->adcRangeBorder[0] = adcBorder0;
	aTCM->m[0] = ctrlBorder0;
	aTCM->k[0] = static_cast<float>(ctrlBorder1 - ctrlBorder0) / (adcBorder1 - adcBorder0); //k = (y1 - y0) / (x1 - x0)
	// border 1
	aTCM->adcRangeBorder[1] = adcBorder1;
	aTCM->m[1] = ctrlBorder1;
	aTCM->k[1] = static_cast<float>(ctrlBorder2 - ctrlBorder1) / (adcBorder2 - adcBorder1); //k = (y2 - y1) / (x2 - x1)
	// border 2
	aTCM->adcRangeBorder[2] = adcBorder2;
	aTCM->m[2] = ctrlBorder2;
	aTCM->k[2] = static_cast<float>(ctrlBorder3 - ctrlBorder2) / (adcBorder3 - adcBorder2); //k = (y3 - y2) / (x3 - x2)
	// border 3
	aTCM->adcRangeBorder[3] = adcBorder3;
	aTCM->m[3] = ctrlBorder3;
	aTCM->k[3] = static_cast<float>(ctrlBorder4 - ctrlBorder3) / (adcBorder4 - adcBorder3); //k = (y4 - y3) / (x4 - x3)
	// border 4
	aTCM->adcRangeBorder[4] = adcBorder4;
	aTCM->m[4] = ctrlBorder4;
	aTCM->k[4] = static_cast<float>(ctrlBorder5 - ctrlBorder4) / (adcBorder5 - adcBorder4); //k = (y5 - y4) / (x5 - x4)
	// border 5
	aTCM->adcRangeBorder[5] = adcBorder5;
	aTCM->m[5] = ctrlBorder5;
	aTCM->k[5] = static_cast<float>(ctrlBorder6 - ctrlBorder5) / (adcBorder6 - adcBorder5); //k = (y6 - y5) / (x6 - x5)
	// border 6
	aTCM->adcRangeBorder[6] = adcBorder6;
	aTCM->m[6] = ctrlBorder6;
	aTCM->k[6] = static_cast<float>(ctrlBorder7 - ctrlBorder6) / (adcBorder7 - adcBorder6); //k = (y7 - y6) / (x7 - x6)
	// border 7
	aTCM->adcRangeBorder[7] = adcBorder7;
	aTCM->m[7] = ctrlBorder7;
	aTCM->k[7] = static_cast<float>(ctrlBorder8 - ctrlBorder7) / (adcBorder8 - adcBorder7); //k = (y8 - y7) / (x8 - x7)
	// border 8
	aTCM->adcRangeBorder[8] = adcBorder8;
	aTCM->m[8] = ctrlBorder8;
	aTCM->k[8] = static_cast<float>(ctrlBorder9 - ctrlBorder8) / (adcBorder9 - adcBorder8); //k = (y9 - y8) / (x9 - x8)
	// border 9
	aTCM->adcRangeBorder[9] = adcBorder9;
	aTCM->m[9] = ctrlBorder9;
	aTCM->k[9] = static_cast<float>(ctrlBorder10 - ctrlBorder9) / (adcBorder10 - adcBorder9); //k = (y10 - y9) / (x10 - x9)
	// last border
	aTCM->adcRangeBorder[10] = adcBorder10;
	aTCM->ctrlRangeBorderHighest = ctrlBorder10;
}

// setAdcToCtrlMap(): Set map with up to 10 ranges (-> 11 borders)
//
// int numBorders: The number of borders, which is the same as the number of elements in <borderList bL> array
//
// borderList_t bL: An array of adcCtrlPair_t struct objects, the objects containing one adcValue and one ctrlValue, comprising a border pair. Only [numBorder] pairs need to be provided.
//
// adcValue0 < adcValue1 < adcValue2 < adcValue3
//
// ctrlValue0 <= ctrlValue1 <= ctrlValue2 <= ctrlValue3
//
//  * ADC values <= bL[0].adcValue are mapped to bL[0].ctrlValue
//  * ADC values >= bL[numBorders - 1].adcValue are mapped to bL[numBorders - 1].ctrlValue
//  * ADC values in the range [bL[N].adcValue..bL[N+1].adcValue] are mapped linearly to the range [bL[N].ctrlValue..bL[N+1].ctrlValue]
//
// Linear mapping is done by using the well known formula (y = k*x + m), where y=ctrlValue, x=adcValue. Since the full resulting mapping is a sequence of (one or more) linear segments, residing inside their given (sub)range, the parameters k and m must be calculated in such a way that the segments meet on each border between ranges.
// For each range [bL[N]..bL[N+1]] this is done by the formula:
//   m = bL[N].ctrlValue;
//   k = (bL[N+1].ctrlValue-bL[N].ctrlValue) / (bL[N+1].adcValue-bL[N].adcValue);
//
void setAdcToCtrlMap(adcToCtrlMap_t *aTCM, int numBorders, borderList_t bL)
{
	debugPrintLn("Entering setAdcToCtrlMap()");
	assert(numBorders >= 2);
	assert(aTCM);
	aTCM->numAdcRanges = numBorders - 1;
	aTCM->numAdcRangeBorders = numBorders;
	for (int border = 0; border < (numBorders - 1); border++)
	{
		debugPrint("bL[");
		debugPrint(border);
		debugPrint("].adcValue: ");
		debugPrintLn(bL[border].adcValue);
		aTCM->adcRangeBorder[border] = bL[border].adcValue;
		debugPrint("aTCM->adcRangeBorder[");
		debugPrint(border);
		debugPrint("]: ");
		debugPrintLn(aTCM->adcRangeBorder[border]);
		debugPrint("bL[");
		debugPrint(border);
		debugPrint("].ctrlValue: ");
		debugPrintLn(bL[border].ctrlValue);
		aTCM->m[border] = bL[border].ctrlValue;
		debugPrint("aTCM->m[");
		debugPrint(border);
		debugPrint("]: ");
		debugPrintLn(aTCM->m[border]);
		aTCM->k[border] = static_cast<float>(bL[border + 1].ctrlValue - bL[border].ctrlValue) / (bL[border + 1].adcValue - bL[border].adcValue); //k<n> = (y<n+1> - y<n>) / (x<n+1> - x<n>)
		debugPrint("aTCM->k[");
		debugPrint(border);
		debugPrint("]: ");
		debugPrintLn(aTCM->k[border]);
	}
	aTCM->adcRangeBorder[numBorders - 1] = bL[numBorders - 1].adcValue;
	aTCM->ctrlRangeBorderHighest = bL[numBorders - 1].ctrlValue;
	debugPrintLn("Exiting setAdcToCtrlMap()");
}

// uint16_t is the return value type of choice since the greatest controller value (pitch-bend included) in MIDI 1.0 do not exceed 14 bit size.
//
// If the adcToCtrlMap struct contains too high gain values or too high offset values in relation to the ADC value, the return value might overflow (i.e.; > (2^16)-1)
// This can be avoided if the gain (k) and offset (m) values are kept low enough in relation to the ADC value.
// The worst case scenarios to consider are the resulting values for all range border value calculations:
// (adcValue * aTCM->k[ix]) + aTCM->m[ix]), where (adcValue = aTCM->adcRangeBorder[ix]) for all ix = [0..numAdcRanges]
// Please check your calculations when setting the gain, offset and border values to make sure that these worst case scenarios do not overflow.
//
uint16_t adcToCtrl(adcToCtrlMap_t *aTCM, uint16_t adcValue)
{
	if (adcValue < aTCM->adcRangeBorder[0])
		return (aTCM->m[0]);
	for (uint8_t ix = 1; ix < aTCM->numAdcRangeBorders; ix++)
	{																								  // Iterate over the number of ADC range borders, starting at 1 since we already checked ix = 0.
		if (adcValue < aTCM->adcRangeBorder[ix])													  // ADC value is between two borders; the current index and the previous one, which we should have already checked (in previous iteration or before iteration started). So a single comparison operator suffices to find each succeeding interval.
			return (((adcValue - aTCM->adcRangeBorder[ix - 1]) * aTCM->k[ix - 1]) + aTCM->m[ix - 1]); // since the range
	}
	// If no previous range was matched, then ADC value belongs to the highest range and should return the calculated value at the highest ADC border, "pegged".
	return aTCM->ctrlRangeBorderHighest;
}

size_t sendNoteOn(byte note, byte velocity, byte channel)
{
	midiPacket4_t data;
	data.data8bit[0] = 0x09;
	data.data8bit[1] = 0x90 | channel;
	data.data8bit[2] = note;
	data.data8bit[3] = velocity;
	return MidiUSB.write(data.data8bit, 4);
}

size_t sendNoteOff(byte note, byte velocity, byte channel)
{
	midiPacket4_t data;
	data.data8bit[0] = 0x08;
	data.data8bit[1] = 0x80 | channel;
	data.data8bit[2] = note;
	data.data8bit[3] = velocity;
	return MidiUSB.write(data.data8bit, 4);
}

size_t sendMidiPacket4(midiPacket4_t midiPacket4)
{
	return MidiUSB.write(midiPacket4.data8bit, 4);
}

void enqueueNoteOn(byte note, byte velocity, byte channel)
{
	midiPacket4_t data;
	data.data8bit[0] = 0x09;
	data.data8bit[1] = 0x90 | channel;
	data.data8bit[2] = note;
	data.data8bit[3] = velocity;
	midiBuffer4.push(data.data32bit);
}

void enqueueNoteOff(byte note, byte velocity, byte channel)
{
	midiPacket4_t data;
	data.data8bit[0] = 0x08;
	data.data8bit[1] = 0x80 | channel;
	data.data8bit[2] = note;
	data.data8bit[3] = velocity;
	midiBuffer4.push(data.data32bit);
}

void enqueuePitchBend(uint16_t adcVal, byte channel)
{
	static uint16_t prevCtrlVal = 0;

	midiPacket4_t data;
	uint16_t ctrlVal = adcToCtrl(&adcToCtrlMapArr[atcmIxPitchbend], adcVal);
	if (ctrlVal != prevCtrlVal)
	{
		// debugPrint("New val: ");
		// debugPrint(ctrlVal);
		// debugPrint("   Old val: ");
		// debugPrintLn(prevCtrlVal);
		data.data8bit[0] = 0x0E;
		data.data8bit[1] = 0xE0 | channel;
		data.data8bit[2] = ctrlVal & 0x7Fu;		   // filter out the 7 LSbits (="fine")
		data.data8bit[3] = (ctrlVal >> 7) & 0x7Fu; // filter out the 7 MSbits (="coarse")
		midiBuffer4.push(data.data32bit);
		prevCtrlVal = ctrlVal;
	}
}

// According to MIDI 1.0 specs and MSB/LSB CC message pairs (assuming receiver cares about these things):
//  * MSB needs not be resent if only LSB is sent. Receiver should interpret it as a "fine adjustment".
//  * If MSB is sent ("coarse adjustment"), LSB will automatically be "reset" to 0 by receiver.
//  To implement this, each CC needs to remember its latest MSB sent, to see if a new one needs to be resent.
//
// User is able to select (globally) if 7-bit or 14-bit resolution should be assumed (aka 7-bit vs 14-bit CC "mode")

void enqueueCC(uint8_t ccNum, uint16_t adcVal, byte channel)
{
	static uint8_t prevCcValMsb[highestMsbCcNumber + 1]; // Remember the previous CC MSB value. Will be 0-initialized (once) by compiler.
	midiPacket4_t data;
	data.data8bit[0] = 0x0B;
	data.data8bit[1] = 0xB0 | channel;

	uint16_t ctrlVal = adcToCtrl(&adcToCtrlMapArr[atcmArrIxPerCC[ccNum]], adcVal);
	uint8_t ccValMsb = (ctrlVal >> 7) & (0x7Fu); // Extract the 7 highest bits of the 14-bit controller value and shift it down.
	uint8_t ccValLsb = ctrlVal & 0x7Fu;			 // Extract the 7 lowest bits of the 14-bit controller value.

	if (ccNum <= highestMsbCcNumber) // If CC# is within MSB range
	{
		if (ccValMsb != prevCcValMsb[ccNum]) // if MSB value for CC (ccNum) has changed since last function call (otherwise no use in sending any new MIDI message)...
		{
			prevCcValMsb[ccNum] = ccValMsb; // Update the "previous CC MSB value"
			data.data8bit[2] = ccNum;
			data.data8bit[3] = ccValMsb;	  // The 7-bit MSB
			midiBuffer4.push(data.data32bit); // push CC MSB message (msg #1)
		}
		//  Assuming 14-bit mode is enabled; the application (almost) always need to resend the LSB message:
		//  * If MSB has changed it needs to resend LSB since the assumption by the receiver otherwise will be that LSB is reset to 0
		//    - Only if MSB has changed and the LSB is exactly = 0 there is no need to resend LSB message, but this is probably a rare case in 14-bit mode and can be ignored.
		//  * If MSB did not change it can be inferred that the LSB must have changed (since *this* function is only called if ADC value has changed)
		//  So, the conclusion is: Iff 14-bit mode is enabled, always send LSB.
		if (gcEnable14BitCc)
		{
			data.data8bit[2] = ccNum + lowestLsbCcNumber;
			data.data8bit[3] = ccValLsb;	  // The 7-bit LSB
			midiBuffer4.push(data.data32bit); // push CC LSB message (msg #2)
		}
	}
}

void enqueueSimpleCC(uint16_t adcVal, byte channel)
{
	midiPacket4_t data;
	data.data8bit[0] = 0x0B;
	data.data8bit[1] = 0xB0 | channel;

	if (true)
	{
		// prevCcValMsb[ccNum] = ccValMsb; // Update the "previous CC MSB value"
		data.data8bit[2] = 1;						// 1 = modulation
		data.data8bit[3] = (adcVal >> 5) & (0x7Fu); // Extract the 7 highest bits of the 12-bit ADC value and shift it down (5 steps).
		midiBuffer4.push(data.data32bit);			// push CC MSB message (msg #1)
	}
}

//
// toggle pins for observability
//

#define REG_PIO_PIN_52_SODR REG_PIOB_SODR
#define REG_PIO_PIN_52_CODR REG_PIOB_CODR
#define REG_PIO_PIN_53_SODR REG_PIOB_SODR
#define REG_PIO_PIN_53_CODR REG_PIOB_CODR
#define PIN_52_BITMASK ((uint32_t)1 << 21)
#define PIN_53_BITMASK ((uint32_t)1 << 14)

#define SET_PIN_52                            \
	{                                         \
		REG_PIO_PIN_52_SODR = PIN_52_BITMASK; \
	}
#define CLR_PIN_52                            \
	{                                         \
		REG_PIO_PIN_52_CODR = PIN_52_BITMASK; \
	}
#define SET_PIN_53                            \
	{                                         \
		REG_PIO_PIN_53_SODR = PIN_53_BITMASK; \
	}
#define CLR_PIN_53                            \
	{                                         \
		REG_PIO_PIN_53_CODR = PIN_53_BITMASK; \
	}

void initPinB()
{
	pinMode(52, OUTPUT);
}

void togglePinB()
{
	static bool toggle = false;
	if (toggle)
	{
		SET_PIN_52 // digitalWrite(52,HIGH);
	}
	else
	{
		CLR_PIN_52 // digitalWrite(52,LOW);
	}
	toggle = !toggle;
}

void initPinA()
{
	pinMode(53, OUTPUT);
}

void togglePinA()
{
	static bool toggle = false;
	if (toggle)
	{
		SET_PIN_53 // digitalWrite(53,HIGH);
	}
	else
	{
		CLR_PIN_53 // digitalWrite(53,LOW);
	}
	toggle = !toggle;
}

// Tries to send the oldest midi packet. Shifts it from the head of the queue if successful. Otherwise leaves queue intact (to have a new go next invocation).
void sendOldestMidiPacket()
{
	midiPacket4_t midiPacket4;
	if (!midiBuffer4.isEmpty())
	{
		midiPacket4.data32bit = midiBuffer4.first();
	}
	else
		return;
	if (MidiUSB.write(midiPacket4.data8bit, 4) > 0)
	{
		// togglePinA();
		midiBuffer4.shift();
	}
	return;
}

//
// Initialize everything
//

// #define LOG_MISSED_TICKS
// #define LOG_KEYSWITCHES

const int _tickDeltaMajor = 250; // Major tick delta in microseconds
const int _tickDeltaMinor = 50;	 // Minor tick delta in microseconds
const int _tickDeltaADC = 10000; // ADC tick delta in microseconds

// uint16_t adcValCh0, adcValCh1, adcValCh2, adcValCh3, adcValCh4, adcValCh5 = 0;
uint16_t adcValPrevCh0, adcValPrevCh1, adcValPrevCh2, adcValPrevCh3, adcValPrevCh4, adcValPrevCh5 = 0;

bool adcChange(uint16_t threshold, uint16_t adcVal, uint16_t adcValPrev)
{
	if (abs(adcVal - adcValPrev) > threshold)
		return true;
	return false;
}

//////////////////////////////////
// SwitchArray types and functions
//////////////////////////////////

// void copySwitchArray(const switchArray_t &src, switchArray_t &dest)
// {
// 	for (int con = 0; con < numConnectors; con++)
// 	{
// 		for (int row = 0; row < numRows; row++)
// 		{
// 			for (int sw = 0; sw < numSwitches; sw++)
// 			{
// 				for (int col = 0; col < numCols; col++)
// 				{
// 					dest[con][row][sw][col] = src[con][row][sw][col];
// 				}
// 			}
// 		}
// 	}
// }

// uint64_t packSwitchArray(switchArray_t src)
// {
// 	// Bit packing (bit=s<x>r<y>c<z>) will be (in groups of 8 bits): s0r0c0..s0r0c7|s0r1c0..s0r1c7|s0r2c0..s0r2c7|s0r3c0..s0r3c7|s1r0c0..s1r0c7|s1r1c0..s1r1c7|s1r2c0..s1r2c7|s1r3c0..s1r3c7 where:
// 	// s0..s1 = switch0..switch1
// 	// r0..r3 = row1..row3
// 	// c0..c7 = column0..column7
// 	uint64_t keySwitchArrayPacked64 = 0; // The entire keySwitchArray packed into 64 bits (numRows*numSwitches*numCols = 64)
// 	int con = 0;						 // TODO! con is not just = 0; Do pack all connectors!
// 	for (int sw = 0; sw < numSwitches; sw++)
// 	{
// 		for (int row = 0; row < numRows; row++)
// 		{
// 			for (int col = 0; col < numCols; col++)
// 			{
// 				keySwitchArrayPacked64 <<= 1;
// 				keySwitchArrayPacked64 |= src[con][row][sw][col];
// 			}
// 		}
// 	}
// 	return keySwitchArrayPacked64;
// }

// void dumpKeySwitches(switchArray_t keySwitchArray)
// {
// 	unsigned char columnSwitchBits = 0;
// 	int con = 0; // TODO iterate over all connectors as well!
// 	for (int row = 0; row < numRows; row++)
// 	{
// 		for (int mkbk = 0; mkbk < numSwitches; mkbk++)
// 		{
// 			for (int col = 0; col < numCols; col++)
// 			{
// 				int keySwitch = keySwitchArray[con][row][mkbk][col];
// 				columnSwitchBits <<= 1;
// 				columnSwitchBits |= keySwitch;
// 			}
// 			SerialUSB.print(columnSwitchBits, HEX);
// 		}
// 		SerialUSB.print(" ");
// 	}
// 	SerialUSB.print(" - ");
// }

////////////////////////////////////
// Various helper functions
////////////////////////////////////

void mynoteon(int note, int vel, int ch)
{
	while (sendNoteOn(note, vel, ch) == 0)
	{
		delayMicroseconds(10);
	};
}

void mynoteoff(int note, int vel, int ch)
{
	while (sendNoteOff(note, vel, ch) == 0)
	{
		delayMicroseconds(10);
	};
}

void deactivateRowPin(int rowPin)
{
#ifdef DISABLE_HIZ
	digitalWrite(rowPin, HIGH);
#else
	digitalWrite(rowPin, HIGH);
	pinMode(rowPin, INPUT); // set row pin to Hi-Z
#endif
}

void activateRowPin(int rowPin)
{
#ifndef DISABLE_HIZ
	pinMode(rowPin, OUTPUT); // prepare pin for write
#endif
	digitalWrite(rowPin, LOW); // set pin to LOW
}

// void midiLogSwitchChange(int con, int row, int mkbk, int col, int keySwitch, int prevKeySwitch)
// {
// 	int a = addressArray[con][row][col];
// 	if (keySwitch != prevKeySwitch)
// 	{
// 		if (keySwitch == LOW)
// 		{ // key switch is pressed
// 			if (mkbk == MK)
// 			{
// 				mynoteon(a, 127, 10);
// 			}
// 			else
// 			{ // mkbk = BK
// 				mynoteon(a, 127, 11);
// 			}
// 		}
// 		else
// 		{ // keySwitch == HIGH; key switch is released
// 			if (mkbk == MK)
// 			{
// 				mynoteoff(a, 127, 10);
// 			}
// 			else
// 			{ // mkbk = BK
// 				mynoteoff(a, 127, 11);
// 			}
// 		}
// 	}
// }

///////////////////
// Log entry stuff
///////////////////

#if defined(LOG_KEYSWITCHES) | defined(LOG_MISSED_TICKS)

enum LOG_ENTRY_TYPES
{
	TickOverrunType,
	SwitchStateType
};

struct logEntry_t
{
	uint32_t time;
	unsigned char entryType;
	uint64_t value;
};

const int logArraySize = 2000;
logEntry_t logArray[logArraySize];

int logArrayIx = 0;

void resetLog()
{
	logArrayIx = 0;
}

void dumpLog()
{
	SerialUSB.println("");
	for (int ix = 0; ix < logArrayIx; ix++)
	{
		SerialUSB.print("Time:");
		SerialUSB.print(logArray[ix].time);
		SerialUSB.print(" ");
		switch (logArray[ix].entryType)
		{
		case TickOverrunType:
			SerialUSB.print("Tick_misses:");
			SerialUSB.print((uint32_t)(0xFFFFFFFF & (logArray[ix].value)));
			break;
		case SwitchStateType:
			SerialUSB.print("Switch_state:");
			SerialUSB.print((uint32_t)(0xFFFFFFFF & (logArray[ix].value >> 32)), HEX); // Upper bytes
			SerialUSB.print(":");
			SerialUSB.print((uint32_t)(0xFFFFFFFF & (logArray[ix].value)), HEX); // Lower bytes
			break;
		default:
			SerialUSB.print("Unknown log entry type!");
			break;
		}
		SerialUSB.println("");
	}
	SerialUSB.println("");
	SerialUSB.println("Dumping of log completed.");
}

void addEntryToLog(int time, int entryType, uint64_t value)
{
	if (logArrayIx < logArraySize)
	{
		logArray[logArrayIx].time = time;
		logArray[logArrayIx].entryType = entryType;
		logArray[logArrayIx].value = value;
		logArrayIx++;
		if (logArrayIx == logArraySize)
			dumpLog();
	}
}

#endif

#ifdef LOG_MISSED_TICKS

void logMissedTicks(int timeStamp, int ticks)
{
	addEntryToLog(timeStamp, TickOverrunType, ticks);
}

#endif

#ifdef LOG_KEYSWITCHES

void logKeySwitches(int timeStamp, switchArray_t sa)
{
	addEntryToLog(timeStamp, SwitchStateType, packSwitchArray(sa));
}

#endif

namespace keybed
{
	// initialize the keybed scanning variables
	const int numConnectors = 2;				// number of switch matrix connectors on keybed
	const int numRows = 4;						// number of key switch matrix rows per connector, disregarding the number of switches per key
	const int numSwitches = 2;					// Number of key switches per key (2 switches => 1 make, 1 break)
	const int numCols = 8;						// number of key switch matrix columns
	const int MK = 0, BK = 1;					// Make or Break row index numering
	const int RELEASED = 0, PRESSED = 1;		// Key states
	const int switchMuteTimerStartValueMK = 20; // * Major tick delta
	const int switchMuteTimerStartValueBK = 20; // * Major tick delta

#define ROW_PORT_INITIAL_BIT_PATTERN (((uint32_t)1) << 1)
#define ROW_PORT_BIT_MASK (((uint32_t)0b11111111) << 1)
#define ROW_PORT_DISABLE_OUTPUT_DATA_REG REG_PIOD_SODR // Disabling is done by setting port pin to HIGH (*Set* Output Data Register; SODR)
#define ROW_PORT_ENABLE_OUTPUT_DATA_REG REG_PIOD_CODR  // Enabling is done by clearing port pin to LOW (*Clear* Output Data Register; CODR)

	// int _rowPin; // remember the row Pin from previous lap in the current scan loop (or, if current lap is the first; from the last lap in the previous scan loop)
	static uint32_t _rowPortBitPattern; // Remember the row port bit pattern from previous lap in the current scan loop (or, if current lap is the first; from the last lap in the previous scan loop)

	struct rowMkbk_t
	{
		int row;
		int mkbk;
	};

	rowMkbk_t rowMkbkLoopSeq[(numCols * numSwitches) + 1] = { // Sequence of <Row,Mkbk> tuples representing the sequence of a full iteration of one connector (where Mkbk is the innermost loop). The sequence is "wrapped around" itself with the first element also added to the end of the sequence. This makes the sequence one element longer than what would be "normal".
		{0, MK},
		{0, BK},
		{1, MK},
		{1, BK},
		{2, MK},
		{2, BK},
		{3, MK},
		{3, BK},
		{0, MK}};

	// int rowPinList[numRows][numSwitches] = {{37,35},{33,31},{29,27},{25,23}}; // Pairs of {MK,BK} switches sorted in increasing switch matrix "row" order (row 0, row 1, ...). These pins are the "write" pins that will be used for output, one by one forced to LOW during scan (not more than one pin is allowed to be LOW at any given moment in time).
	// int colPinList[numConnectors][numCols] = {{22,24,26,28,30,32,34,36},{38,40,42,44,46,48,50,52}}; // These are the "read" pins that will be used for input (potentially with internal pullup resistors enabled), During scan reading "LOW" will mean a pressed key switch.

	int rowPinList[numRows][numSwitches] = {{26, 27}, {28, 14}, {15, 29}, {11, 12}};							   // Pairs of {MK,BK} switches sorted in increasing switch matrix "row" order (row 0, row 1, ...). These pins are the "write" pins that will be used for output, one by one forced to LOW during scan (not more than one pin is allowed to be LOW at any given moment in time).
	int colPinList[numConnectors][numCols] = {{33, 34, 35, 36, 37, 38, 39, 40}, {51, 50, 49, 48, 47, 46, 45, 44}}; // These are the "read" pins that will be used for input (potentially with internal pullup resistors enabled), During scan reading "LOW" will mean a pressed key switch.

	///////////////////////////////////////////////////////////////////////////////////
	//
	// Row/Column switch matrix scanning pin mapping for parallell read
	//
	// Row: Pin
	//
	// 0: D.1
	// 1: D.2
	// 2: D.3
	// 3: D.4
	// 4: D.5
	// 5: D.6
	// 6: D.7
	// 7: D.8

	// Connector/Column: Pin
	//
	// N.B. Connector 0 pins are inverted
	//
	// 0/0: C.1
	// 0/1: C.2
	// 0/2: C.3
	// 0/3: C.4
	// 0/4: C.5
	// 0/5: C.6
	// 0/6: C.7
	// 0/7: C.8
	//
	// 1/0: C.12
	// 1/1: C.13
	// 1/2: C.14
	// 1/3: C.15
	// 1/4: C.16
	// 1/5: C.17
	// 1/6: C.18
	// 1/7: C.19

	//////////////////////////////
	// KeyArray types and funtions
	//////////////////////////////

	typedef int keyArray_t[numConnectors][numRows][numCols]; // type representing the matrix of keybed keys, disregarding the number of switches per key, storing a single (32-bit) integer per key

	keyArray_t addressArray = {
		{{0, 1, 2, 3, 4, 5, 6, 7},
		 {8, 9, 10, 11, 12, 13, 14, 15},
		 {16, 17, 18, 19, 20, 21, 22, 23},
		 {24, 25, 26, 27, 28, 29, 30, 31}},
		{{32, 33, 34, 35, 36, 37, 38, 39},
		 {40, 41, 42, 43, 44, 45, 46, 47},
		 {48, 49, 50, 51, 52, 53, 54, 55},
		 {56, 57, 58, 59, 60, 61, 52, 53}}};

	keyArray_t keyState = {
		{{RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED},
		 {RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED},
		 {RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED},
		 {RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED}},
		{{RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED},
		 {RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED},
		 {RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED},
		 {RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED, RELEASED}}};

	keyArray_t keyVelocityStopwatch = {
		{{0, 0, 0, 0, 0, 0, 0, 0},
		 {0, 0, 0, 0, 0, 0, 0, 0},
		 {0, 0, 0, 0, 0, 0, 0, 0},
		 {0, 0, 0, 0, 0, 0, 0, 0}},
		{{0, 0, 0, 0, 0, 0, 0, 0},
		 {0, 0, 0, 0, 0, 0, 0, 0},
		 {0, 0, 0, 0, 0, 0, 0, 0},
		 {0, 0, 0, 0, 0, 0, 0, 0}}};

	typedef unsigned char switchArray_t[numConnectors][numRows][numSwitches][numCols]; // type representing the matrix of switches on the keybed

	switchArray_t switchMuteTimer = {
		{{{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}},
		 {{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}},
		 {{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}},
		 {{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}}},
		{{{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}},
		 {{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}},
		 {{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}},
		 {{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}}}};

	switchArray_t switchArray = // switch array for the matrix representing the entire 5 octave keybed
		{{{{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}}},
		 {{{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}}}};

	switchArray_t switchArrayFake = // switch array for the matrix representing the entire 5 octave keybed, simulating the real switches
		{{{{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}}},
		 {{{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}},
		  {{HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}, {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}}}};

	//////////////////////////////////
	// velocityMap types and functions
	//////////////////////////////////

	const int velocityStopWatchMaxValue = 1000;								 // Note that lowest value is 0
	const int sizeVelocityStopWatchMaxValue = velocityStopWatchMaxValue + 1; // size is one larger than max value
	typedef uint8_t velocityMap_t[sizeVelocityStopWatchMaxValue];			 // type mapping a velocity stop-watch value to actual MIDI velocity
	velocityMap_t velocityMap;

	enum mapType_t
	{
		LIN_STD,
		LIN_1,
		LIN_2,
		LOG_STD,
		LOG_1,
		LOG_2,
		EXP_8
	};

	void initVelocityMap(velocityMap_t &map, int mapType)
	{
		int expN;
		switch (mapType)
		{
		case LIN_STD:
			for (int stopWatchIx = 0; stopWatchIx < sizeVelocityStopWatchMaxValue; stopWatchIx++)
			{
				velocityMap[stopWatchIx] = (1 - (static_cast<float>(stopWatchIx) / velocityStopWatchMaxValue)) * 127;
			}
			break;
		case EXP_8:
			expN = 8;
			for (int stopWatchIx = 0; stopWatchIx < sizeVelocityStopWatchMaxValue; stopWatchIx++)
			{
				velocityMap[stopWatchIx] = ((1.f / expN) * pow(126 * expN, (1 - (static_cast<float>(stopWatchIx) / velocityStopWatchMaxValue)))) + 1;
			}
		default:
			break;
		}
	}

	void dumpVelocityMap(velocityMap_t velocityMap)
	{
		for (int stopWatchIx = 0; stopWatchIx < velocityStopWatchMaxValue; stopWatchIx++)
		{
			SerialUSB.print(velocityMap[stopWatchIx]);
			SerialUSB.print(" ");
		}
	}

}

void scanKeybed()
{
#define ENABLE_KEY_DEBOUNCE
#define DISABLE_HIZ

	using namespace keybed;

	int row;
	int mkbk;
	// int colKeySwitch[numConnectors][numCols];

	// Scan the keybed

	// Scan the row/mkbk pins

#ifdef LOG_KEYSWITCHES
	int _thisTick = microSeconds;
#endif
	togglePinA();

	// loop over (numRows*numSwitches), to scan rows and mkbk switches ( Note: <numConnectors> connectors are scanned in parallell).
	for (int rowMkbk = 0; rowMkbk < (numRows * numSwitches); rowMkbk++)
	{

		togglePinB();

		// read REG_PIOC_PDSR
		uint32_t pioc_input = REG_PIOC_PDSR;

		// pack data to 16 LSB bits, starting from this 32 bit value (straight from PIOC input register, port "C" name is implicit):
		// PIOC input port bits: [31..20][19..12][11..9][8..1][0]
		//                                ^^^^^^         ^^^^
		// Interpretation:         (switch1,col7..0)  (switch0,col7..0)

		// ending with this 16 bit value (port "C" name is implicit) in dedicated column variable:
		// PIOC input port bits: [19..12][8..1]
		// column variable bits:  15..8   7..0
		// Interpretation:   (sw1,c7..0) (sw0,c7..0)
		//

		uint16_t colKeySwitchBM = 0;							// Bit Matrix corresponding to the values read from key switch columns from all switches (all switches + all columns)
		colKeySwitchBM |= ((pioc_input >> (8 - 7)) & 0x00FF);	// pack 8 LSB bits
		colKeySwitchBM |= ((pioc_input >> (19 - 15)) & 0xFF00); // pack 8 MSB bits

		// togglePinA();

		// row = rowMkbkLoopSeq[rowMkbk + 1].row;  // let row reference next lap
		// mkbk = rowMkbkLoopSeq[rowMkbk + 1].mkbk; // let mkbk reference next lap
		ROW_PORT_DISABLE_OUTPUT_DATA_REG = _rowPortBitPattern;				// Deactivate "old" port bit by setting to HIGH (= Set Output Data Register). // deactivateRowPin(_rowPin);
		_rowPortBitPattern = (_rowPortBitPattern << 1) & ROW_PORT_BIT_MASK; // Calculate next bit pattern // _rowPin = rowPinList[row][mkbk];
		ROW_PORT_ENABLE_OUTPUT_DATA_REG = _rowPortBitPattern;				// Activate new port bit by setting to LOW (= Clear Output Data Register). // activateRowPin(_rowPin); // prepare for next lap by enabling the output pin already now. For the last lap the output pin will be set for the first lap of the next loop (see the definition of "rowMkbkLoopSeq[]"")

		// SerialUSB.print(_rowPortBitPattern,BIN);
		// SerialUSB.print("..");

		row = rowMkbkLoopSeq[rowMkbk].row;	 // let row reference current lap
		mkbk = rowMkbkLoopSeq[rowMkbk].mkbk; // let mkbk reference current lap

		// Scan columns based on previously read column pins
		for (int con = 0; con < numConnectors; con++)
		{
			for (int col = 0; col < numCols; col++)
			{

				// int keySwitch = colKeySwitch[con][col];
				int keySwitch = colKeySwitchBM & 0x01; // Mask out LSB from colKeySwitchBM
				colKeySwitchBM >>= 1;				   // Shift entire colKeySwitchBM variable one bit right

				if (con == 0)
					keySwitch = 1 - keySwitch; // negate keySwitch for connector 0 , assuming 74HC14 is in series with conenctor 0 output. (74HC14 vs 74hc9015 workaround)
#ifdef ENABLE_KEY_DEBOUNCE

				///////////////////////////////////////////////////////
				// Handle key mute timers and key velocity stop watches
				///////////////////////////////////////////////////////

				if (switchMuteTimer[con][row][mkbk][col] > 0)
					switchMuteTimer[con][row][mkbk][col]--; // decrement switchMuteTimer until reaches zero
				if (mkbk == MK)
				{ // Velocity: Only act on MK switch scan, to avoid double stopwatch increments per key (NOTE! this does not mean that velocity stopwatch is only handled when MK switch is closed, just that it is handled only once per key)
					int stopwatch = keyVelocityStopwatch[con][row][col];
					if (stopwatch > 0 && stopwatch < velocityStopWatchMaxValue)
						keyVelocityStopwatch[con][row][col]++; // increment keyVelocityStopwatch within limits, 0 = not started; 1 = min, 1000 = max
				}
#endif

				int prevKeySwitch = switchArray[con][row][mkbk][col]; // recall the switch state (for the current output/input pin-pair) from previous scan
				switchArray[con][row][mkbk][col] = keySwitch;		  // update the switchArray history (for the current output/input pin-pair)

				// midiLogSwitchChange(con,row,mkbk,col,keySwitch,prevKeySwitch);

				//////////////////////////////
				// Handle keypresses/releases
				/////////////////////////////

				if (keySwitch != prevKeySwitch && switchMuteTimer[con][row][mkbk][col] == 0)
				{ // if there is a new value from this scan, compared to previous scan AND switch is not muted

					int currentKeyState = keyState[con][row][col];

					//////////////////////////////////////////////////////////
					// key down, bottom switch, from RELEASED state => note-on
					if (keySwitch == LOW && mkbk == MK && currentKeyState == RELEASED)
					{
#ifdef ENABLE_KEY_DEBOUNCE
						switchMuteTimer[con][row][mkbk][col] = switchMuteTimerStartValueMK; // set+start MK switchMuteTimer
#endif
						int address = addressArray[con][row][col];
						enqueueNoteOn(address + 24, velocityMap[keyVelocityStopwatch[con][row][col]], 1);
						keyState[con][row][col] = PRESSED; // Set key state to new value (key has been properly pressed)
					}

					//////////////////////////////////////////////////////////////////////////////////////////////////
					// key down, bottom switch, from PRESSED state => incomplete retrigger attempt of note-on, discard
					else if (keySwitch == LOW && mkbk == MK && currentKeyState == PRESSED)
					{
						; // Do nothing
					}

					//////////////////////////////////////////////////////////////////////////////////////
					// key down, top switch, from (should always be) RELEASED state => prepare for note-on
					else if (keySwitch == LOW && mkbk == BK && currentKeyState == RELEASED)
					{
#ifdef ENABLE_KEY_DEBOUNCE
						switchMuteTimer[con][row][mkbk][col] = switchMuteTimerStartValueBK; // set+start BK switchMuteTimer
#endif
						keyVelocityStopwatch[con][row][col] = 1; // (Re)start key velocity clock (for anticipated note on)
					}

					/////////////////////////////////////////////////////
					// key up, top switch, from PRESSED state => note-off
					else if (keySwitch == HIGH && mkbk == BK && currentKeyState == PRESSED)
					{
						enqueueNoteOff(addressArray[con][row][col] + 24, 64, 1);
						keyState[con][row][col] = RELEASED; // Set key state to new value (key has been properly released)
					}

					/////////////////////////////////////////////////////////////
					// key up, top switch, from RELEASED state => aborted note-on
					else if (keySwitch == HIGH && mkbk == BK && currentKeyState == RELEASED)
					{
						keyVelocityStopwatch[con][row][col] = 0; // Reset key velocity clock (aborted note on)
					}
				}
			} // end columns scan
		}	  // end connectors scan

		togglePinB();

	} // end rowMkbk loop

	// since this was the last rowMkBk lap; reset _rowPortBitPattern to initial state again to prepare for next loop() invocation
	ROW_PORT_DISABLE_OUTPUT_DATA_REG = _rowPortBitPattern && (ROW_PORT_BIT_MASK); // deactivate current port bit (if at all needed)
	_rowPortBitPattern = ROW_PORT_INITIAL_BIT_PATTERN;							  // Preprare for activating the "next" (= initial) port bit pattern, aka restarting the loop "in advance"...
	ROW_PORT_ENABLE_OUTPUT_DATA_REG = _rowPortBitPattern;						  // ...and do it!

	// SerialUSB.print(_rowPortBitPattern,BIN);
	// SerialUSB.println("::");

#ifdef LOG_KEYSWITCHES
	logKeySwitches(_thisTick, switchArrayLo);
#endif

	togglePinA();
}

PurpleReign::Task keybedTask(scanKeybed, _tickDeltaMajor);

//  * Read ADC channel values, store in memory, interpret the values and enqueue MIDI controller messages
//  * Restart ADC convertion
void scanAdc()
{
	// handle ADC channel 0 (pitch bend)
	int adcValCh0 = adc_get_channel_value(ADC, ADC_CHANNEL_0); // Connected to pitch bend
	if (adcChange(16, adcValCh0, adcValPrevCh0))
	{
		enqueuePitchBend(adcValCh0, 1);
		// enqueueSimpleCC(adcValCh0, 1);
		adcValPrevCh0 = adcValCh0;
	}

	// handle ADC channel 1 (modulation)
	int adcValCh1 = adc_get_channel_value(ADC, ADC_CHANNEL_1); // Connected to modulation
	if (adcChange(16, adcValCh1, adcValPrevCh1))
	{
		enqueueCC(ccNumModulation, adcValCh1, 1);
		// enqueueSimpleCC(adcValCh1, 1);
		adcValPrevCh1 = adcValCh1;
	}
	// handle ADC channel 2 (GP1)
	int adcValCh2 = adc_get_channel_value(ADC, ADC_CHANNEL_2); // Connected to modulation
	if (adcValCh2 != adcValPrevCh2)
		// enqueueCC(ccNumGeneralPurpose1, adcValCh2, 1);
		adcValPrevCh2 = adcValCh2;

	// handle ADC channel 3 (GP2)
	int adcValCh3 = adc_get_channel_value(ADC, ADC_CHANNEL_3); // Connected to modulation
	if (adcValCh3 != adcValPrevCh3)
		// enqueueCC(ccNumGeneralPurpose2, adcValCh3, 1);
		adcValPrevCh3 = adcValCh3;

	// handle ADC channel 4 (GP3)
	int adcValCh4 = adc_get_channel_value(ADC, ADC_CHANNEL_4); // Connected to modulation
	if (adcValCh4 != adcValPrevCh4)
		// enqueueCC(ccNumGeneralPurpose3, adcValCh4, 1);
		adcValPrevCh4 = adcValCh4;

	// handle ADC channel 5 (GP4)
	int adcValCh5 = adc_get_channel_value(ADC, ADC_CHANNEL_5); // Connected to modulation
	if (adcValCh5 != adcValPrevCh5)
		// enqueueCC(ccNumGeneralPurpose4, adcValCh5, 1);
		adcValPrevCh5 = adcValCh5;

	// Restart ADC conversion
	adc_start(ADC);
}

PurpleReign::Task adcTask(scanAdc, _tickDeltaADC);

// send a MIDI note whenever possible, but not faster than every _nextTickMinor microsec.
void sendMidi()
{
	// togglePinB();
	sendOldestMidiPacket();
	MidiUSB.flush();
	// togglePinB();
}

PurpleReign::Task midiTask(sendMidi, _tickDeltaMinor);

///// SETUP!!! ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
	using namespace keybed;

	initVelocityMap(velocityMap, EXP_8);

	delay(5000);

	SerialUSB.begin(115200); // Initialize serial debug port
	SerialUSB.println("Serial debug port initialized!");

	dumpVelocityMap(velocityMap);

#if defined(LOG_KEYSWITCHES) || defined(LOG_MISSED_TICKS) || defined(PURE_DEBUG)
	SerialUSB.begin(115200); // Initialize serial debug port
	SerialUSB.println("Serial debug port initialized!");
#endif

	////////////////////////////////////////////////////////////////
	// Configure PIO
	////////////////////////////////////////////////////////////////

	// Initialize keybed scanning pins by making them all inactive

	for (int row = 0; row < numRows; row++)
	{
		for (int mkbk = 0; mkbk < numSwitches; mkbk++)
		{
			int pin = rowPinList[row][mkbk];
#ifdef DISABLE_HIZ
			pinMode(pin, OUTPUT);
			digitalWrite(pin, HIGH); // HIGH = inactive
#else
			pinMode(pin, INPUT);
#endif
		}
	}

	// Configure column scan pins as inputs. Configure it without pullup resistors since an external buffer IC will be connected to input pins.
	for (int con = 0; con < numConnectors; con++)
	{
		for (int col = 0; col < numCols; col++)
		{
			int pin = colPinList[con][col];
			pinMode(pin, INPUT);
		}
	}

	// Enable row pin for very first row scan

	// int rowMkbk = 0;
	// int row = rowMkbkLoopSeq[rowMkbk].row;
	// int mkbk = rowMkbkLoopSeq[rowMkbk].mkbk;
	// _rowPin=rowPinList[row][mkbk];
	// activateRowPin(_rowPin);

	ROW_PORT_DISABLE_OUTPUT_DATA_REG = ROW_PORT_BIT_MASK; // Disable all row port bits (the mask is equal to all bits set to "1")
	_rowPortBitPattern = ROW_PORT_INITIAL_BIT_PATTERN;	  // Now prepare to enable only the initial first row port bit (at the same time setting the global variable that will remember the pattern between various loops)...
	ROW_PORT_ENABLE_OUTPUT_DATA_REG = _rowPortBitPattern; // ... and do it!

	// SerialUSB.print(_rowPortBitPattern,BIN);
	// SerialUSB.println("::");

	initPinA();
	initPinB();

	/////////////////////////////////////////////////////////////////////////
	// Configure midi controller mappings
	/////////////////////////////////////////////////////////////////////////

	// Assign adcToControllerMapArray indices to controllers
	// N.B.! index 0 is reserved for Pitch Bend!
	atcmArrIxPerCC[ccNumModulation] = 1;
	atcmArrIxPerCC[ccNumGeneralPurpose1] = 2;
	atcmArrIxPerCC[ccNumGeneralPurpose2] = 2;
	atcmArrIxPerCC[ccNumGeneralPurpose3] = 2;
	atcmArrIxPerCC[ccNumGeneralPurpose3] = 2;

	// TODO: Make these values part of a dynamic callibration procedure and store them in EEPROM or similar
	{
		borderList_t tmpBorderList = {{0x16 << 5, 0}, {0x42 << 5, 8192}, {0x46 << 5, 8192}, {0x72 << 5, 16383}};
		setAdcToCtrlMap(&adcToCtrlMapArr[atcmIxPitchbend], 4, tmpBorderList); // ADC value of 2048 is the center value with +/- 48 as "dead zone".
		printCtrlMap(&adcToCtrlMapArr[1]);
	}
	{
		borderList_t tmpBorderList = {{0x1E << 5, 16383}, {0x44 << 5, 0}};
		setAdcToCtrlMap(&adcToCtrlMapArr[1], 2, tmpBorderList);
		printCtrlMap(&adcToCtrlMapArr[1]);
	}
	{
		borderList_t tmpBorderList = {{10, 0}, {4085, 16383}};
		setAdcToCtrlMap(&adcToCtrlMapArr[2], 2, tmpBorderList);
		printCtrlMap(&adcToCtrlMapArr[1]);
	}

	/////////////////////////////////////////////////////////////////////////
	// Configure ADC
	/////////////////////////////////////////////////////////////////////////

	{ // Configure ADC

		////////////////////////////////////////////////////////////////////////
		//  ---------------------------------------
		//  * Resets entire ADC (sets SWRESET flag)
		//  * Resets ADC_MR
		//  * Disables PDC transfer (for ADC peripheral)
		//  * Sets ADC_MR:PRESCAL (based on ul_mck and ul_adc_clk parameters)
		//  * Sets ADC_MR:STARTUP (based on uc_startup parameter)
		//  ---------------------------------------
		//  * Master Clock (MCK) = 84 MHz (VARIANT_MCK). Note: DUE external 12 MHz X-tal OSC multiplied by 7 through PLL/prescaler yields 84 MHz.
		//  * ADC Clock = 1 MHz (ADC_FREQ_MIN) is good enough for ~100Hz sample rate on 16 channels (~1600 Hz ADC sample rate).
		//  * Startup time = 0, since sleep mode will be disabled.

		adc_init(ADC, VARIANT_MCK, ADC_FREQ_MIN, 0); // Note that doxygen comments for adc_init() erroneously refers to the formal parameter "ul_mck" as "main clock", but it really is "master clock".

		////////////////////////////////////////////////////////////////////////
		//  ----------------------
		//  * Sets ADC_MR:TRACKTIM
		//  * Sets ADC_MR:SETTLING
		//  * Sets ADC_MR:TRANSFER
		//  ----------------------
		//  * 12-bit tracking time = 0.054 * Z_source + 205 (ns); where Z_source is source output impedance in ohms. See Atmel SAM3X data sheet section 45.7.2.1
		//    - Assuming wire capacitance, inductance and resistance can be neglected (which might not be true!) Z_source comes only form the attached potentiometer, which are all around 10K ohm. If an analog input buffer is added between potentiometer and ADC input, the impedance will be a lot lower (equivalent to the output impedance of the buffer, a few 10's of ohms).
		//    - Assuming 10K potentiometer without buffer circuit, Tracking time = 0.054 * 10,000 + 205 (ns) = 540 + 205 (ns) = 745 ns.
		//      + Number of ADC clock cycles needed for tracking = 745 ns / (1 / ADC_FREQ [=1E6]) = (745 * 10E-9) s * (ADC_FREQ [=1E6]) Hz = 745 * 10E-3 = 0.745 (= 1 clock cycles, rounding up)
		//      + Adjusting for 0-based value coding: 1 clock cycle is coded as 1 - 1 = 0.
		//
		//  * Settling time: Probably irrelevant since ADC_MR:ANACH field (probably?) will be set to NONE since same analog configuration (probably?) will be used for all channels.
		//
		//  * Transfer time; Probably irrelevant since no DMA transfer will be used.

		adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_0, 0);

		// Set ADC_MR:TRGEN to DIS (disable HW triggering and thus only allow SW triggering). Set ADC_MR:FREERUN to OFF (disable freerun mode).
		adc_configure_trigger(ADC, ADC_TRIG_SW, ADC_MR_FREERUN_OFF);

		// Set ADC_MR:LOWRES to BITS_12 (use 12-bit ADC resolution).
		adc_set_resolution(ADC, ADC_12_BITS);

		// Set ADC_MR:SLEEP to NORMAL (disable sleep mode). Set ADC_MR:FWUP to OFF (disable fast wakeup).
		adc_configure_power_save(ADC, ADC_MR_SLEEP_NORMAL, ADC_MR_FWUP_OFF);

		{ // Analog configuration

			// Set ADC_MR:IBCTL to 1 (even though 00 might be enough for ADC sampling fq below 500 KHz)
			adc_set_bias_current(ADC, ADC_ACR_IBCTL(1));

			// Set ADC_MR:ANCH to NONE (disallow changes in analog settings between different channels). This results in ADC_CGR:GAIN0, ADC_COR:OFF0 and ADC_COR:DIFF0 being used for all channels.
			adc_disable_anch(ADC);

			// Disable differential input mode for ADC channel 0 (which also applies to all channels when ADC_MR:ANCH is set to NONE).
			adc_disable_channel_differential_input(ADC, ADC_CHANNEL_0);

			// Disable input offset for ADC channel 0 (which also applies to all channels when ADC_MR:ANCH is set to NONE).
			adc_disable_channel_input_offset(ADC, ADC_CHANNEL_0);

			// Set input gain to 1 for ADC channel 0 (which also applies to all channels when ADC_MR:ANCH is set to NONE).
			adc_set_channel_input_gain(ADC, ADC_CHANNEL_0, ADC_GAINVALUE_1);
		}

		// Enable ADC channels.
		adc_enable_channel(ADC, ADC_CHANNEL_0);
		adc_enable_channel(ADC, ADC_CHANNEL_1);

		// Disable ADC channel sequencer, instead use simple numeric order.
		adc_stop_sequencer(ADC);

		// Disable all ADC interrupts
		adc_disable_interrupt(ADC, 0xFFFFFFFF);

		// Enable the ADC to be clocked by MCK.
		pmc_enable_periph_clk(ID_ADC);

		// Start first ADC conversion
		adc_start(ADC);

		delayMicroseconds(1000); // Wait 1 ms, should be enough to finish conversion of all (<=16) channels, even at slowest conversion rate (~50kS/s)

		adcValPrevCh0 = adc_get_channel_value(ADC, ADC_CHANNEL_0); // Connected to pitch bend
		adcValPrevCh1 = adc_get_channel_value(ADC, ADC_CHANNEL_1); // Connected to modulation wheel
		adcValPrevCh2 = adc_get_channel_value(ADC, ADC_CHANNEL_2); // Connected to general purpose controller 1
		adcValPrevCh3 = adc_get_channel_value(ADC, ADC_CHANNEL_3); // Connected to general purpose controller 2
		adcValPrevCh4 = adc_get_channel_value(ADC, ADC_CHANNEL_4); // Connected to general purpose controller 3
		adcValPrevCh5 = adc_get_channel_value(ADC, ADC_CHANNEL_5); // Connected to general purpose controller 4
	}

	delay(1000); // Wait for MIDI to stabilize

	mynoteon(99, 99, 16); // Hello world!

	// _nextTickMajor = micros();												 // initiate next tick time = now
	// _nextTickMinor = _nextTickMajor + _tickDeltaMinor; // At least in theory the very first _nextTickMinor will happen shortly after, but not exactly on, the first _nextTickMajor. Maybe put them in off-phase? Hmmm...
	// _nextTickADC = _nextTickMajor;										 // should be ok?
}

// THE LOOP!!! ////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
	keybedTask.schedule();
	adcTask.schedule();
	midiTask.schedule();
}