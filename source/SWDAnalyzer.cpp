/*
    The SW-DP protocol is described by the following publicly available documents:

    DDI 0316 CoreSight™ DAP-Lite Technical Reference Manual
    http://infocenter.arm.com/help/topic/com.arm.doc.ddi0316d/DDI0316D_dap_lite_trm.pdf

    Programming Internal Flash Over the Serial Wire Debug Interface
    http://www.silabs.com/Support%20Documents/TechnicalDocs/AN0062.pdf

    CY8C41xx, CY8C42xx Programming Specifications
    http://www.cypress.com/?docID=48133

    SW-DP (Serial Wire Debug Port) Analyzer plugin for the Saleae Logic

    Copyright (C) 2015 Peter Lawrence.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License version 2.1, as
    published by the Free Software Foundation.  This program is
    distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
    License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program; if not, see <http://www.gnu.org/licenses/>.
*/

#include "SWDAnalyzer.h"
#include "SWDAnalyzerSettings.h"
#include <AnalyzerChannelData.h>

SWDAnalyzer::SWDAnalyzer()
:	Analyzer2(),  
	mSettings( new SWDAnalyzerSettings() ),
	mSimulationInitialized( false )
{
	SetAnalyzerSettings( mSettings.get() );
}

SWDAnalyzer::~SWDAnalyzer()
{
	KillThread();
}

void SWDAnalyzer::SetupResults()
{
	mResults.reset( new SWDAnalyzerResults( this, mSettings.get() ) );
	SetAnalyzerResults( mResults.get() );
	mResults->AddChannelBubblesWillAppearOn( mSettings->mSWDIOChannel );
}

/* frames must not overlap, so clip a sequence frame to start after the last frame added */
void SWDAnalyzer::AddSequenceFrame( U8 type, U64 start_sample, U64 end_sample, U64 data1, U64 data2 )
{
	if ( start_sample <= mLastFrameEnd )
		start_sample = mLastFrameEnd + 1;
	if ( start_sample > end_sample )
		return;

	Frame frame;
	frame.mType = type;
	frame.mData1 = data1;
	frame.mData2 = data2;
	frame.mFlags = 0;
	frame.mStartingSampleInclusive = start_sample;
	frame.mEndingSampleInclusive = end_sample;
	mResults->AddFrame( frame );
	mLastFrameEnd = end_sample;
}

/*
    ADIv5.2/ADIv6 B5.3 sequences; all are sent LSB first, so after shifting each new bit
    in at the top of the 128-bit history (hist_hi:hist_lo), the last n bits received equal
    (hist_hi >> (64 - n)).
*/
#define SELECTION_ALERT_HI  0x19BC0EA2E3DDAFE9ull
#define SELECTION_ALERT_LO  0x86852D956209F392ull
#define JTAG_TO_SWD         0xE79E      /* 16 bits, after a line reset */
#define SWD_TO_DORMANT      0xE3BC      /* 16 bits, after a line reset */
#define JTAG_TO_DORMANT     0x33BBBBBA  /* 31 bits, after >= 5 cycles high */
#define ACTIVATION_BITS     12          /* 4 cycles low + 8-bit activation code */

void SWDAnalyzer::WorkerThread()
{
	mSWDIO = GetAnalyzerChannelData( mSettings->mSWDIOChannel );
	mSWCLK = GetAnalyzerChannelData( mSettings->mSWCLKChannel );

	if( mSWCLK->GetBitState() == BIT_HIGH )
		mSWCLK->AdvanceToNextEdge();

	U64 current_sample, onset_sample, previous_current_sample;
	U32 ones_count, data_count;

	bool rise_bit, parity;

	enum state_enum
	{
		START,
		APnDP,
		RnW,
		A0,
		A1,
		PARITY,
		STOP,
		PARK,
		TRN,
		ACK0,
		ACK1,
		ACK2,
		ACKTRN,
		DATA,
		RST,
		ENDTRN,
	};

	enum state_enum state, next_state;

	U8 command, ack;
	U32 data;

	U64 hist_hi = 0, hist_lo = 0;   /* last 128 SWDIO bits, newest in bit 63 of hist_hi */
	U64 edge_samples[128];          /* sample number of each of the last 128 bits */
	U64 bit_number = 0;
	U64 ones_start_sample = 0;
	U32 bits_since_alert = 0xFFFFFFFF;

	/*
	    A line reset is >= 50 cycles high; ADIv6 also requires >= 2 idle cycles after it, ADIv5 doesn't.
	    The low bits after the high run are only idle cycles if they don't start a JTAG-to-SWD,
	    SWD-to-Dormant or JTAG-to-Dormant sequence, so the frame is held back until that's ruled out.
	*/
	bool reset_pending = false, reset_counting_idle = false;
	U64 reset_start = 0, reset_high_end = 0, reset_idle_end = 0;
	U32 reset_ones = 0, reset_idle = 0, bits_since_reset = 0;

	state = RST;
	ones_count = 6;
	previous_current_sample = 0;
	mLastFrameEnd = 0;

	for( ; ; )
	{
		mSWCLK->AdvanceToNextEdge(); // rising edge
		current_sample = mSWCLK->GetSampleNumber();

		mSWDIO->AdvanceToAbsPosition(current_sample - 1);
		rise_bit = mSWDIO->GetBitState() == BIT_HIGH;

		Frame frame;

#if 1
		/* method of jump-starting decoding if capture doesn't include 50 ones */
		/* only while lost: hosts may legally stall SWCLK mid-transaction (e.g. at turnaround) */
		if ( (RST == state) && (current_sample - previous_current_sample) > 10000 )
			state = START;
#endif

		switch (state)
		{
		case START:
			next_state = (rise_bit) ? APnDP : START;
			if (rise_bit)
			{
				mResults->AddMarker( current_sample, AnalyzerResults::Start, mSettings->mSWDIOChannel );
				onset_sample = current_sample;	
			}
			break;
		case APnDP:
			parity = rise_bit;
			next_state = RnW;
			command &= 0x7;
			if (rise_bit)
				command |= 0x8;
			mResults->AddMarker( current_sample, (rise_bit) ? AnalyzerResults::One : AnalyzerResults::Zero, mSettings->mSWDIOChannel );
			break;
		case RnW:
			parity ^= rise_bit;
			next_state = A0;
			command &= 0xB;
			if (rise_bit)
				command |= 0x4;
			mResults->AddMarker( current_sample, (rise_bit) ? AnalyzerResults::One : AnalyzerResults::Zero, mSettings->mSWDIOChannel );
			break;
		case A0:
			parity ^= rise_bit;
			next_state = A1;
			command &= 0xE;
			if (rise_bit)
				command |= 0x1;
			mResults->AddMarker( current_sample, (rise_bit) ? AnalyzerResults::One : AnalyzerResults::Zero, mSettings->mSWDIOChannel );
			break;
		case A1:
			parity ^= rise_bit;
			next_state = PARITY;
			command &= 0xD;
			if (rise_bit)
				command |= 0x2;
			mResults->AddMarker( current_sample, (rise_bit) ? AnalyzerResults::One : AnalyzerResults::Zero, mSettings->mSWDIOChannel );
			break;
		case PARITY:
			next_state = (parity == rise_bit) ? STOP : RST;
			mResults->AddMarker( current_sample, (parity == rise_bit) ? AnalyzerResults::Dot : AnalyzerResults::ErrorDot, mSettings->mSWDIOChannel );
			break;
		case STOP:
			next_state = (rise_bit) ? RST : PARK;
			mResults->AddMarker( current_sample, (rise_bit) ? AnalyzerResults::ErrorDot : AnalyzerResults::Stop, mSettings->mSWDIOChannel );
			break;
		case PARK:
			next_state = (rise_bit) ? TRN : RST;
			break;
		case TRN:
			mResults->AddMarker( current_sample, AnalyzerResults::Square, mSettings->mSWDIOChannel );
			next_state = ACK0;
			break;
		case ACK0:
			next_state = ACK1;
			ack &= 0x6;
			if (rise_bit)
				ack |= 0x1;
			mResults->AddMarker( current_sample, (rise_bit) ? AnalyzerResults::One : AnalyzerResults::Zero, mSettings->mSWDIOChannel );
			break;
		case ACK1:
			next_state = ACK2;
			ack &= 0x5;
			if (rise_bit)
				ack |= 0x2;
			mResults->AddMarker( current_sample, (rise_bit) ? AnalyzerResults::One : AnalyzerResults::Zero, mSettings->mSWDIOChannel );
			break;
		case ACK2:
			next_state = (command & 0x4) ? DATA : ACKTRN;
			data_count = 0;
			parity = false;
			mResults->AddMarker( current_sample, (rise_bit) ? AnalyzerResults::One : AnalyzerResults::Zero, mSettings->mSWDIOChannel );

			ack &= 0x3;
			if (rise_bit)
				ack |= 0x4;

			frame.mData1 = ack & 0x7;
			switch (ack & 0x7)
			{
			case 1: /* OK */
			case 2: /* WAIT */
				frame.mFlags = 0;
				break;
			case 4: /* FAULT */
				frame.mFlags = DISPLAY_AS_WARNING_FLAG;
				break;
			default:
				frame.mFlags = DISPLAY_AS_ERROR_FLAG;
				break;
			}

			break;
		case ACKTRN:
			mResults->AddMarker( current_sample, AnalyzerResults::Square, mSettings->mSWDIOChannel );
			next_state = DATA;
			break;
		case DATA:
			data_count++;

			if (33 == data_count)
			{
				next_state = (command & 0x4) ? ENDTRN : START;
				mResults->AddMarker( current_sample, (parity == rise_bit) ? AnalyzerResults::Dot : AnalyzerResults::ErrorDot, mSettings->mSWDIOChannel );

				frame.mType = SWD_FRAME_TRANSACTION;
				frame.mData1 = (U32)command + ( (U32)ack << 4 );
				frame.mData2 = data;
				frame.mFlags = 0;
				frame.mStartingSampleInclusive = onset_sample;
				frame.mEndingSampleInclusive = current_sample;
				mResults->AddFrame( frame );
				mLastFrameEnd = current_sample;
			}
			else
			{
				data >>= 1;
				if (rise_bit)
					data |= 0x80000000;
				parity ^= rise_bit;
				next_state = DATA;
				mResults->AddMarker( current_sample, (rise_bit) ? AnalyzerResults::One : AnalyzerResults::Zero, mSettings->mSWDIOChannel );
			}
			break;
		case ENDTRN:
			mResults->AddMarker( current_sample, AnalyzerResults::Square, mSettings->mSWDIOChannel );
			next_state = START;
			break;
		case RST:
			next_state = ( (ones_count >= 50) && !rise_bit) ? START : RST;
			if (ones_count >= 50)
				mResults->AddMarker( current_sample, AnalyzerResults::UpArrow, mSettings->mSWDIOChannel);
			break;
		default:
			break;
		}

		/* out-of-band sequences, recognised regardless of the transaction state machine */
		hist_lo = (hist_lo >> 1) | (hist_hi << 63);
		hist_hi = (hist_hi >> 1) | ( (U64)rise_bit << 63 );
		edge_samples[bit_number & 127] = current_sample;
		bit_number++;

		if ( !rise_bit && (ones_count >= 50) )
		{
			reset_pending = true;
			reset_counting_idle = true;
			reset_start = ones_start_sample;
			reset_high_end = previous_current_sample;
			reset_ones = ones_count;
			reset_idle = 0;
			bits_since_reset = 0;
		}
		if ( reset_pending )
		{
			bits_since_reset++;
			if ( reset_counting_idle && !rise_bit )
			{
				reset_idle++;
				reset_idle_end = current_sample;
			}
			else
				reset_counting_idle = false;

			U32 seq = (U32)(hist_hi >> 48);
			if ( (16 == bits_since_reset) && ( (JTAG_TO_SWD == seq) || (SWD_TO_DORMANT == seq) ) )
			{
				AddSequenceFrame( SWD_FRAME_LINE_RESET, reset_start, reset_high_end, reset_ones );
				AddSequenceFrame( (JTAG_TO_SWD == seq) ? SWD_FRAME_JTAG_TO_SWD : SWD_FRAME_SWD_TO_DORMANT,
					edge_samples[(bit_number - 16) & 127], current_sample, seq );
				reset_pending = false;
				next_state = RST; /* both must be followed by a line reset or a selection alert */
			}
			else if ( (31 == bits_since_reset) &&
				( ( (U32)(hist_hi >> 33) == JTAG_TO_DORMANT ) || ( (hist_hi >> 33) == (SELECTION_ALERT_LO & 0x7FFFFFFF) ) ) )
			{
				/* JTAG-to-Dormant, or the start of a selection alert: their frames are added below */
				AddSequenceFrame( SWD_FRAME_LINE_RESET, reset_start, reset_high_end, reset_ones );
				reset_pending = false;
			}
			else if ( (bits_since_reset >= 31) && !reset_counting_idle )
			{
				AddSequenceFrame( SWD_FRAME_LINE_RESET, reset_start, reset_idle ? reset_idle_end : reset_high_end,
					reset_ones, reset_idle );
				reset_pending = false;
			}
		}
		if ( ( (U32)(hist_hi >> 33) == JTAG_TO_DORMANT ) && ( ( (hist_hi >> 28) & 0x1F ) == 0x1F ) )
		{
			AddSequenceFrame( SWD_FRAME_JTAG_TO_DORMANT, edge_samples[(bit_number - 31) & 127], current_sample, JTAG_TO_DORMANT );
			next_state = RST;
		}
		if ( (SELECTION_ALERT_HI == hist_hi) && (SELECTION_ALERT_LO == hist_lo) && (bit_number >= 128) )
		{
			AddSequenceFrame( SWD_FRAME_SELECTION_ALERT, edge_samples[bit_number & 127], current_sample, 0 );
			bits_since_alert = 0;
			next_state = RST;
		}
		else if ( bits_since_alert != 0xFFFFFFFF )
		{
			if ( ACTIVATION_BITS == ++bits_since_alert )
			{
				AddSequenceFrame( SWD_FRAME_ACTIVATION, edge_samples[(bit_number - ACTIVATION_BITS) & 127], current_sample,
					(U32)(hist_hi >> (64 - ACTIVATION_BITS)) );
				bits_since_alert = 0xFFFFFFFF;
				next_state = RST;
			}
		}

		if ( rise_bit )
		{
			if ( (0 == ones_count) || (1 == bit_number) )
				ones_start_sample = current_sample;
			ones_count++;
			/* 50 consecutive highs can't occur inside a valid transaction: abandon it */
			if ( ones_count >= 50 )
				next_state = RST;
		}
		else
			ones_count = 0;

		state = next_state;

		mResults->CommitResults();

		ReportProgress( current_sample );

		previous_current_sample = current_sample;

		mSWCLK->AdvanceToNextEdge(); // falling edge
	}
}

bool SWDAnalyzer::NeedsRerun()
{
	return false;
}

U32 SWDAnalyzer::GenerateSimulationData( U64 minimum_sample_index, U32 device_sample_rate, SimulationChannelDescriptor** simulation_channels )
{
	if( mSimulationInitialized == false )
	{
		mSimulationDataGenerator.Initialize( GetSimulationSampleRate(), mSettings.get() );
		mSimulationInitialized = true;
	}

	return mSimulationDataGenerator.GenerateSimulationData( minimum_sample_index, device_sample_rate, simulation_channels );
}

U32 SWDAnalyzer::GetMinimumSampleRateHz()
{
	return 0;
}

const char* SWDAnalyzer::GetAnalyzerName() const
{
	return "SW-DP";
}

const char* GetAnalyzerName()
{
	return "SW-DP";
}

Analyzer* CreateAnalyzer()
{
	return new SWDAnalyzer();
}

void DestroyAnalyzer( Analyzer* analyzer )
{
	delete analyzer;
}
