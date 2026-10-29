#ifndef SWD_ANALYZER_H
#define SWD_ANALYZER_H

#include <Analyzer.h>
#include "SWDAnalyzerResults.h"
#include "SWDSimulationDataGenerator.h"

class SWDAnalyzerSettings;
class ANALYZER_EXPORT SWDAnalyzer : public Analyzer2
{
public:
	SWDAnalyzer();
	virtual ~SWDAnalyzer();

	virtual void SetupResults();
	virtual void WorkerThread();

	virtual U32 GenerateSimulationData( U64 newest_sample_requested, U32 sample_rate, SimulationChannelDescriptor** simulation_channels );
	virtual U32 GetMinimumSampleRateHz();

	virtual const char* GetAnalyzerName() const;
	virtual bool NeedsRerun();

protected: //functions
	void AddSequenceFrame( U8 type, U64 start_sample, U64 end_sample, U64 data1, U64 data2 = 0 );

protected: //vars
	U64 mLastFrameEnd;
	std::auto_ptr< SWDAnalyzerSettings > mSettings;
	std::auto_ptr< SWDAnalyzerResults > mResults;
	AnalyzerChannelData* mSWDIO;
	AnalyzerChannelData* mSWCLK;

	SWDSimulationDataGenerator mSimulationDataGenerator;
	bool mSimulationInitialized;
};

extern "C" ANALYZER_EXPORT const char* __cdecl GetAnalyzerName();
extern "C" ANALYZER_EXPORT Analyzer* __cdecl CreateAnalyzer( );
extern "C" ANALYZER_EXPORT void __cdecl DestroyAnalyzer( Analyzer* analyzer );

#endif //SWD_ANALYZER_H
