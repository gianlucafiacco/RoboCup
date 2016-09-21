//~ 
//~ 
//~ #include "NoWifiChallengeRecognizer.h"
//~ #include "Tools/Settings.h"
//~ #include "Tools/Debugging/DebugDrawings.h"
//~ 
//~ MAKE_MODULE(WhistleRecognizer, spqr_modules)
//~ 
//~ #ifndef WINDOWS
//~ // We currently do not have a working FFTW3 implementation
//~ // for Windows in our repository.
//~ 
//~ #include <limits>
//~ #include "DefaultTones.h"
//~ #include "Platform/Thread.h"
//~ #include <iostream>
//~ #include <map>
//~ 
//~ #include <cmath>
//~ 
//~ static SyncObject _syncObject;
//~ std::map <int, int> timeline;
//~ std::map<int,int>::iterator it = timeline.begin();
//~ float count=0;
//~ 
//~ int numerotrovato;
//~ int currentTime;
//~ bool controlMap=true;
//~ bool control= false;
//~ 
//~ 
//~ 
//~ WhistleRecognizer::WhistleRecognizer()
//~ {
    //~ for(int i = 0; i < WHISTLE_BUFF_LEN; ++i)
    //~ {
        //~ inputChannel0.push_front(0);
        //~ inputChannel1.push_front(0);
    //~ }
//~ 
    //~ maxAutoCorrelationValue = 800000; // Default value for preconfigured whistle
    //~ cmpCnt = 0;
    //~ lastGameState = STATE_INITIAL;
    //~ lastTimeWhistleDetectedInBothChannels = 0;
//~ 
    //~ // Allocate memeory for FFTW plans
    //~ whistleInput8kHz = (double*) fftw_malloc(sizeof(double) * WHISTLE_FFT_LEN);
    //~ fftDataIn        = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (WHISTLE_BUFF_LEN + 1));
    //~ corrBuff         = (double*) fftw_malloc(sizeof(double) * WHISTLE_FFT_LEN);
//~ 
    //~ // Create plans
    //~ //  - after plan has been created, in and out buffer are set to zero
    //~ //  - plan has to be created only once
    //~ // Creation of FFTW plans is not thread-safe, thus we need to synchronize with the other threads
    //~ //   - This is only relevant for simulations that contain multiple robots
    //~ SYNC;
    //~ fft2048  = fftw_plan_dft_r2c_1d(WHISTLE_FFT_LEN, whistleInput8kHz, fftDataIn, FFTW_MEASURE);   // build plan that fftw needs to compute the fft
    //~ ifft2048 = fftw_plan_dft_c2r_1d(WHISTLE_FFT_LEN, fftDataIn, corrBuff, FFTW_MEASURE); // build plan that fftw needs to compute the fft
//~ 
    //~ // Load reference whistle (memory has already been allocated)
    //~ //loadReferenceWhistle();
//~ }
//~ 
//~ WhistleRecognizer::~WhistleRecognizer()
//~ {
    //~ // Destruction of FFTW plans is not thread-safe
    //~ SYNC;
    //~ fftw_destroy_plan(fft2048);
    //~ fftw_destroy_plan(ifft2048);
    //~ fftw_free(whistleInput8kHz);
    //~ fftw_free(fftDataIn);
    //~ fftw_free(corrBuff);
//~ }
//~ 
//~ #endif
//~ 
//~ void WhistleRecognizer::update(NoWifiReceivedPacket& noWifiReceivedPacket)
//~ {
//~ 
//~ #ifdef WINDOWS // Windows implementation ends here, remaning stuff uses FFTW
//~ }
//~ #else
	//~ // Discard the packet already read
	//~ if (theNoWifiPacketRead.packetRead)
		//~ noWifiReceivedPacket.available = false;
	//~ 
    //~ // Only listen to the whistle in set state and clear
    //~ // the buffers when entering a set state:
    //~ if(lastGameState != STATE_SET && theGameInfo.state == STATE_INITIAL)
    //~ {
        //~ for(int i = 0; i < WHISTLE_BUFF_LEN; ++i)
        //~ {
            //~ inputChannel0.push_front(0);
            //~ inputChannel1.push_front(0);
        //~ }
        //~ cmpCnt = 0;
    //~ }
    //~ lastGameState = theGameInfo.state;
    //~ if(theGameInfo.state != STATE_PLAYING)
        //~ return;
//~ 
    //~ // Check input data:
    //~ if(theAudioData.channels != 2)
    //~ {
        //~ OUTPUT_TEXT("Wrong number of channels! WhistleRecognizer expects 2 channels, but AudioData has " << theAudioData.channels << "!");
    //~ }
    //~ if(theAudioData.samples.size() == 0)
        //~ return;
    //~ // else
        //~ // whistle.lastTimeOfIncomingSound = theFrameInfo.time;   // TODO: SERVE?
//~ 
    //~ // Add incoming audio data to the two buffers
    //~ unsigned int i = 0;
//~ 
    //~ while(i < theAudioData.samples.size())
    //~ {
        //~ cmpCnt++;
        //~ const float sample0 = static_cast<float>(theAudioData.samples[i]) / static_cast<float>(std::numeric_limits<short>::max());
        //~ ++i;
        //~ const float sample1 = static_cast<float>(theAudioData.samples[i]) / static_cast<float>(std::numeric_limits<short>::max());
        //~ ++i;
        //~ inputChannel0.push_front(sample0);
        //~ inputChannel1.push_front(sample1);
    //~ }
//~ 
    //~ /*DEBUG_RESPONSE_ONCE("module:WhistleRecognizer:saveReferenceWhistle")
    //~ {
        //~ saveReferenceWhistle();
    //~ }
    //~ DEBUG_RESPONSE_ONCE("module:WhistleRecognizer:recordNewReferenceWhistle")
    //~ {
        //~ if(inputChannel0.full())
        //~ {
            //~ OUTPUT_TEXT("Record new Whistle");
            //~ recordNewReferenceWhistle();
        //~ }
    //~ }*/
//~ 
    //~ // Recognize the whistle
    //~ double correlationChannel0 = 0.0;
    //~ double correlationChannel1 = 0.0;
    //~ float  currentVolume = 0.0;
    //~ if(inputChannel0.full() && cmpCnt >= WHISTLE_OVERLAP)
    //~ {
        //~ cmpCnt = 0;
        //~ const bool w0  = detectWhistle(inputChannel0, correlationChannel0);
        //~ const bool w1  = detectWhistle(inputChannel1, correlationChannel1);
        //~ currentVolume  = computeCurrentVolume();
        //~ const bool vol = currentVolume > volumeThreshold;
    //~ }
//~ 
	//~ uint16_t locationRaw[8];
	//~ // std::cerr << "sent " << sent << std::endl;
    //~ if(control==true && sent == 0 ){
        //~ int k=0;
        //~ for (it=timeline.begin(); it!=timeline.end(); ++it){
            //~ locationRaw[k] = it->second;
            //~ k++;
            //~ std::cout<< "TEMPO  " << it->first << "   VALORE  " << it->second << std::endl;
        //~ }
        //~ timeline.clear();
        //~ control=false;
     //~ /*   for(int j=0; j<8; j++){
            //~ std::cout<<"ciaooo";
            //~ std::cout<< whistle.locationMessage[j]   ;
     //~ }*/
     //~ 
		//~ // Convert to coordinates
		 //~ uint16_t x = 0, y = 0, e = 3;
		 //~ for (int k=0; k<4; k++) {
			 //~ x += locationRaw[k] * pow(10, e);
			 //~ y += locationRaw[k + 4] * pow(10, e);
			 //~ std::cerr << locationRaw[k] << " - " << x << ", "<< locationRaw[k + 4] << " - " << y << std::endl;
			 //~ --e;
		 //~ }
		 //~ 
		 //~ // New packet available
		 //~ noWifiReceivedPacket.packet.header.type = SPL_NO_WIFI_PAYLOAD_TYPE_LOCATION;
		 //~ noWifiReceivedPacket.packet.payload.location.x = x;
		 //~ noWifiReceivedPacket.packet.payload.location.y = y;	
		 //~ noWifiReceivedPacket.available = true;
		 //~ std::cerr << "WhistleRecognizer -> packet produced" << std::endl;
     //~ }
	 //~ 
    //~ // Finally, plot the whistle correlation of each channel:
    //~ if(!deactivatePlots)
    //~ {
        //~ DECLARE_PLOT("module:WhistleRecognizer:whistleCorrelationChannel0");
        //~ DECLARE_PLOT("module:WhistleRecognizer:whistleCorrelationChannel1");
        //~ PLOT("module:WhistleRecognizer:whistleCorrelationChannel0", correlationChannel0);
        //~ PLOT("module:WhistleRecognizer:whistleCorrelationChannel1", correlationChannel1);
        //~ DECLARE_PLOT("module:WhistleRecognizer:audioInput60HzChannel0");
        //~ DECLARE_PLOT("module:WhistleRecognizer:audioInput60HzChannel1");
        //~ PLOT("module:WhistleRecognizer:audioInput60HzChannel0", inputChannel0.back());
        //~ PLOT("module:WhistleRecognizer:audioInput60HzChannel1", inputChannel1.back());
        //~ DECLARE_PLOT("module:WhistleRecognizer:currentVolume");
        //~ PLOT("module:WhistleRecognizer:currentVolume", currentVolume);
    //~ }
//~ }
//~ 
//~ bool WhistleRecognizer::detectWhistle(const RingBuffer<float, WHISTLE_BUFF_LEN>& inputChannel, double& correlation)
//~ {
    //~ for (int j=0; j<WHISTLE_BUFF_LEN; j++)
    //~ {
        //~ whistleInput8kHz[j] = inputChannel[j]; // write just the half of dataIn and the rest is zero padded
    //~ }
//~ 
    //~ //compare tone
    //~ double comparazione[11] = {
        //~ compareTone(fftCmpData2600,correlation),
        //~ //compareTone(fftCmpData512,correlation),
        //~ compareTone(fftCmpData2650,correlation),
        //~ compareTone(fftCmpData2750,correlation),
        //~ compareTone(fftCmpData2850,correlation),
        //~ compareTone(fftCmpData2950,correlation),
        //~ compareTone(fftCmpData3050,correlation),
        //~ compareTone(fftCmpData3100,correlation),
        //~ compareTone(fftCmpData3150,correlation),
        //~ compareTone(fftCmpData3250,correlation),
        //~ compareTone(fftCmpData3350,correlation),
        //~ compareTone(fftCmpData3400,correlation),
    //~ };
//~ 
    //~ double max = -1;
    //~ int index = -1;
//~ 
    //~ for(int i=0; i<11; i++){
        //~ if(comparazione[i]>-2)
            //~ if(comparazione[i]>max){
                //~ max = comparazione[i];
                //~ index = i;
            //~ }
//~ 
    //~ }
//~ 
    //~ if(index == 0) {
        //~ std::cout << "Ho trovato 0" ;
        //~ numerotrovato=0;
        //~ currentTime=SystemCall::getCurrentSystemTime();
        //~ if((timeline.empty()) || (timeline.rbegin()->second != numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050) ||   (timeline.rbegin()->second == numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050)){
//~ 
            //~ timeline.insert( std::pair<int,int>(currentTime, numerotrovato));
            //~ std::cout<< "Inserito "<< numerotrovato  <<std::endl;
            //~ std::cout<< "NUMERI NELLA MAPPA        "<< timeline.size() <<std::endl;
        //~ }
        //~ std::cout<< "CORR        "<< count <<std::endl;
        //~ count=0;
		//~ sent = -1;
        //~ return true;
    //~ }
    //~ else if(index == 1) {
        //~ std::cout << "Ho trovato 1" ;
        //~ numerotrovato=1;
        //~ currentTime=SystemCall::getCurrentSystemTime();
        //~ if((timeline.empty()) || (timeline.rbegin()->second != numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050) ||   (timeline.rbegin()->second == numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050)){
//~ 
            //~ timeline.insert( std::pair<int,int>(currentTime, numerotrovato));
            //~ std::cout<< "Inserito "<< numerotrovato  <<std::endl;
            //~ std::cout<< "NUMERI NELLA MAPPA        "<< timeline.size() <<std::endl;
        //~ }
        //~ std::cout<< "CORR        "<< count <<std::endl;
        //~ count=0;
		//~ sent = -1;
        //~ return true;
    //~ }
    //~ else if(index == 2) {
        //~ std::cout << "Ho trovato 2" ;
        //~ numerotrovato=2;
        //~ currentTime=SystemCall::getCurrentSystemTime();
        //~ if((timeline.empty()) || (timeline.rbegin()->second != numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050) ||   (timeline.rbegin()->second == numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050)){
//~ 
            //~ timeline.insert( std::pair<int,int>(currentTime, numerotrovato));
            //~ std::cout<< "Inserito "<< numerotrovato  <<std::endl;
            //~ std::cout<< "NUMERI NELLA MAPPA        "<< timeline.size() <<std::endl;
        //~ }
        //~ std::cout<< "CORR        "<< count <<std::endl;
        //~ count=0;
		//~ sent = -1;
        //~ return true;
    //~ }
//~ 
    //~ else if(index == 3) {
        //~ std::cout << "Ho trovato 3" ;
        //~ numerotrovato=3;
        //~ currentTime=SystemCall::getCurrentSystemTime();
        //~ if((timeline.empty()) || (timeline.rbegin()->second != numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050) ||   (timeline.rbegin()->second == numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050)){
//~ 
            //~ timeline.insert( std::pair<int,int>(currentTime, numerotrovato));
            //~ std::cout<< "Inserito "<< numerotrovato  <<std::endl;
            //~ std::cout<< "NUMERI NELLA MAPPA        "<< timeline.size() <<std::endl;
        //~ }
        //~ std::cout<< "CORR        "<< count <<std::endl;
        //~ count=0;
		//~ sent = -1;
        //~ return true;
    //~ }
//~ 
    //~ else if(index == 4) {
        //~ std::cout << "Ho trovato 4" ;
        //~ numerotrovato=4;
        //~ currentTime=SystemCall::getCurrentSystemTime();
        //~ if((timeline.empty()) || (timeline.rbegin()->second != numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050) ||   (timeline.rbegin()->second == numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050)){
//~ 
            //~ timeline.insert( std::pair<int,int>(currentTime, numerotrovato));
            //~ std::cout<< "Inserito "<< numerotrovato  <<std::endl;
            //~ std::cout<< "NUMERI NELLA MAPPA        "<< timeline.size() <<std::endl;
        //~ }
        //~ std::cout<< "CORR        "<< count <<std::endl;
        //~ count=0;
		//~ sent = -1;
        //~ return true;
    //~ }
//~ 
    //~ else if(index == 5) {
        //~ std::cout << "Ho trovato 5" ;
        //~ numerotrovato=5;
        //~ currentTime=SystemCall::getCurrentSystemTime();
        //~ if((timeline.empty()) || (timeline.rbegin()->second != numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050) ||   (timeline.rbegin()->second == numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050)){
//~ 
            //~ timeline.insert( std::pair<int,int>(currentTime, numerotrovato));
            //~ std::cout<< "Inserito "<< numerotrovato  <<std::endl;
            //~ std::cout<< "NUMERI NELLA MAPPA        "<< timeline.size() <<std::endl;
        //~ }
        //~ std::cout<< "CORR        "<< count <<std::endl;
        //~ count=0;
		//~ sent = -1;
        //~ return true;
    //~ }
//~ 
    //~ else if(index == 6) {
        //~ std::cout << "Ho trovato 6" ;
        //~ numerotrovato=6;
        //~ currentTime=SystemCall::getCurrentSystemTime();
        //~ if((timeline.empty()) || (timeline.rbegin()->second != numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050) ||   (timeline.rbegin()->second == numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050)){
//~ 
            //~ timeline.insert( std::pair<int,int>(currentTime, numerotrovato));
            //~ std::cout<< "Inserito "<< numerotrovato  <<std::endl;
            //~ std::cout<< "NUMERI NELLA MAPPA        "<< timeline.size() <<std::endl;
        //~ }
        //~ std::cout<< "CORR        "<< count <<std::endl;
        //~ count=0;
		//~ sent = -1;
        //~ return true;
    //~ }
//~ 
    //~ else if(index == 7) {
        //~ std::cout << "Ho trovato 7" ;
        //~ numerotrovato=7;
        //~ currentTime=SystemCall::getCurrentSystemTime();
        //~ if((timeline.empty()) || (timeline.rbegin()->second != numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050) ||   (timeline.rbegin()->second == numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050)){
//~ 
            //~ timeline.insert( std::pair<int,int>(currentTime, numerotrovato));
            //~ std::cout<< "Inserito "<< numerotrovato  <<std::endl;
            //~ std::cout<< "NUMERI NELLA MAPPA        "<< timeline.size() <<std::endl;
        //~ }
        //~ std::cout<< "CORR        "<< count <<std::endl;
        //~ count=0;
		//~ sent = -1;
        //~ return true;
    //~ }
//~ 
    //~ else if(index == 8) {
        //~ std::cout << "Ho trovato 8" ;
        //~ numerotrovato=8;
        //~ currentTime=SystemCall::getCurrentSystemTime();
        //~ if((timeline.empty()) || (timeline.rbegin()->second != numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050) ||   (timeline.rbegin()->second == numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050)){
//~ 
            //~ timeline.insert( std::pair<int,int>(currentTime, numerotrovato));
            //~ std::cout<< "Inserito "<< numerotrovato  <<std::endl;
            //~ std::cout<< "NUMERI NELLA MAPPA        "<< timeline.size() <<std::endl;
        //~ }
        //~ std::cout<< "CORR        "<< count <<std::endl;
        //~ count=0;
		//~ sent = -1;
        //~ return true;
    //~ }
//~ 
    //~ else if(index == 9) {
        //~ std::cout << "Ho trovato 9" ;
        //~ numerotrovato=9;
        //~ currentTime=SystemCall::getCurrentSystemTime();
        //~ if((timeline.empty()) || (timeline.rbegin()->second != numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050) ||   (timeline.rbegin()->second == numerotrovato && (currentTime - timeline.rbegin()->first)>= 1050)){
//~ 
            //~ timeline.insert( std::pair<int,int>(currentTime, numerotrovato));
            //~ std::cout<< "Inserito "<< numerotrovato  <<std::endl;
            //~ std::cout<< "NUMERI NELLA MAPPA        "<< timeline.size() <<std::endl;
        //~ }
        //~ std::cout<< "CORR        "<< count <<std::endl;
        //~ count=0;
		//~ sent = -1;
        //~ return true;
    //~ }
//~ 
    //~ else if(index == 10) {
        //~ std::cout << "Ho trovato 10" ;
        //~ std::cout<< "CORR        "<< count <<std::endl;
        //~ count=0;
        //~ control=true;
        //~ sent ++;
        //~ return true;
    //~ }
//~ 
    //~ return false;
//~ }
//~ 
//~ double WhistleRecognizer::compareTone(fftw_complex fftCmpData[], double &correlation){
//~ 
    //~ fftw_execute(fft2048);
    //~ const int REAL = 0;     // real value of complex
    //~ const int IMAG = 1;     // imaginary value of complex
//~ 
    //~ for (int j=0; j<WHISTLE_BUFF_LEN+1; j++)
    //~ {
        //~ const double realFFTDataIn = fftDataIn[j][REAL];
        //~ fftDataIn[j][REAL] = realFFTDataIn*fftCmpData[j][REAL] - fftDataIn[j][IMAG]*fftCmpData[j][IMAG]; // real x real - imag x imag
        //~ fftDataIn[j][IMAG] = fftDataIn[j][IMAG]*fftCmpData[j][REAL] + realFFTDataIn*fftCmpData[j][IMAG]; // real x imag + imag x real
    //~ }
    //~ fftw_execute(ifft2048); // calculate the ifft which is now the same as the correlation
//~ 
    //~ /*
     //~ * Post Processing:
     //~ * Find the maximum of the correlation and before weight by the
     //~ * highest value possible, which is given by the auto correlation
     //~ */
    //~ correlation = 0.0;
//~ 
    //~ for (int j=0; j<WHISTLE_CORR_LEN; j++)
    //~ {
        //~ if(abs(corrBuff[j])>count){
            //~ count=corrBuff[j];
        //~ }
//~ 
        //~ corrBuff[j] /= maxAutoCorrelationValue;
//~ 
        //~ if(correlation < corrBuff[j])
            //~ correlation = corrBuff[j];
        //~ else if(correlation < -corrBuff[j])
            //~ correlation = -corrBuff[j];
    //~ }
//~ 
    //~ if (correlation >= whistleThreshold)
    //~ {
        //~ OUTPUT_TEXT("Whistle has been blown!" << correlation);
        //~ return correlation;
    //~ }
    //~ return -2;
//~ }
//~ 
//~ void WhistleRecognizer::recordNewReferenceWhistle()
//~ {
//~ }
//~ 
//~ float WhistleRecognizer::computeCurrentVolume()
//~ {
    //~ float volume0 = 0.0;
    //~ float volume1 = 0.0;
    //~ if(!theDamageConfigurationHead.audioChannel0Defect)
    //~ {
        //~ for(float sample : inputChannel0)
            //~ volume0 += std::abs(sample);
    //~ }
    //~ volume0 /= static_cast<float>(inputChannel0.size());
    //~ if(!theDamageConfigurationHead.audioChannel1Defect)
    //~ {
        //~ for(float sample : inputChannel1)
            //~ volume1 += std::abs(sample);
    //~ }
    //~ volume1 /= static_cast<float>(inputChannel1.size());
    //~ if(!theDamageConfigurationHead.audioChannel0Defect && !theDamageConfigurationHead.audioChannel1Defect)
        //~ return (volume0 + volume1) / 2.f;
    //~ else if(theDamageConfigurationHead.audioChannel0Defect)
        //~ return volume1;
    //~ else // this means if(theDamageConfiguration.audioChannel1Defect)
        //~ return volume0;
//~ }
//~ 
//~ void WhistleRecognizer::saveReferenceWhistle()
//~ {
//~ }
//~ 
//~ void WhistleRecognizer::loadReferenceWhistle()
//~ {
//~ }
//~ 
//~ void WhistleRecognizer::mappa(){
//~ }
//~ #endif
