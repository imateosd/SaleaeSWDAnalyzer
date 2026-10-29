#ifndef SWD_ANALYZER_RESULTS
#define SWD_ANALYZER_RESULTS

#include <AnalyzerResults.h>

class SWDAnalyzer;
class SWDAnalyzerSettings;

/* Frame::mType values */
enum SWDFrameType
{
	SWD_FRAME_TRANSACTION = 0,
	SWD_FRAME_LINE_RESET,      /* mData1 = number of SWDIO high cycles, mData2 = idle cycles after them */
	SWD_FRAME_JTAG_TO_SWD,
	SWD_FRAME_SWD_TO_DORMANT,
	SWD_FRAME_JTAG_TO_DORMANT,
	SWD_FRAME_SELECTION_ALERT,
	SWD_FRAME_ACTIVATION,      /* mData1 = 12 bits following the alert: 4 low cycles + 8-bit code */
};

class SWDAnalyzerResults : public AnalyzerResults
{
public:
	SWDAnalyzerResults( SWDAnalyzer* analyzer, SWDAnalyzerSettings* settings );
	virtual ~SWDAnalyzerResults();

	virtual void GenerateBubbleText( U64 frame_index, Channel& channel, DisplayBase display_base );
	virtual void GenerateExportFile( const char* file, DisplayBase display_base, U32 export_type_user_id );

	virtual void GenerateFrameTabularText(U64 frame_index, DisplayBase display_base );
	virtual void GeneratePacketTabularText( U64 packet_id, DisplayBase display_base );
	virtual void GenerateTransactionTabularText( U64 transaction_id, DisplayBase display_base );

protected: //functions

protected:  //vars
	SWDAnalyzerSettings* mSettings;
	SWDAnalyzer* mAnalyzer;
};

#endif //SWD_ANALYZER_RESULTS
