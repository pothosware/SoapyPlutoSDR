#include <iio.h>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>

#define PLUTOSDR_DEFAULT_IP "192.168.2.1"
#define PLUTOSDR_DEFAULT_HOSTNAME "pluto.local"

class SoapyPlutoSDR : public SoapySDR::Device{

public:
    SoapyPlutoSDR( const SoapySDR::Kwargs & args );
    ~SoapyPlutoSDR();

    /*******************************************************************
	 * Identification API
	 ******************************************************************/

    std::string getDriverKey( void ) const;


    std::string getHardwareKey( void ) const;


    SoapySDR::Kwargs getHardwareInfo( void ) const;


    /*******************************************************************
     * Channels API
     ******************************************************************/

    size_t getNumChannels( const int ) const;


    bool getFullDuplex( const int direction, const size_t channel ) const;

    /*******************************************************************
	 * Stream API
	 ******************************************************************/

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;

    SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;

    SoapySDR::Stream *setupStream(
            const int direction,
            const std::string &format,
            const std::vector<size_t> &channels = std::vector<size_t>(),
            const SoapySDR::Kwargs &args = SoapySDR::Kwargs() );


    void closeStream( SoapySDR::Stream *stream );


    size_t getStreamMTU( SoapySDR::Stream *stream ) const;


    int activateStream(
            SoapySDR::Stream *stream,
            const int flags = 0,
            const long long timeNs = 0,
            const size_t numElems = 0 );


    int deactivateStream(
            SoapySDR::Stream *stream,
            const int flags = 0,
            const long long timeNs = 0 );


    int readStream(
            SoapySDR::Stream *stream,
            void * const *buffs,
            const size_t numElems,
            int &flags,
            long long &timeNs,
            const long timeoutUs = 100000 );


    int writeStream(
            SoapySDR::Stream *stream,
            const void * const *buffs,
            const size_t numElems,
            int &flags,
            const long long timeNs = 0,
            const long timeoutUs = 100000);

    int readStreamStatus(
            SoapySDR::Stream *stream,
            size_t &chanMask,
            int &flags,
            long long &timeNs,
            const long timeoutUs
    );


    int acquireReadBuffer(
            SoapySDR::Stream *stream,
            size_t &handle,
            const void **buffs,
            int &flags,
            long long &timeNs,
            const long timeoutUs = 100000);

    void releaseReadBuffer(
            SoapySDR::Stream *stream,
            const size_t handle);

    int acquireWriteBuffer(
            SoapySDR::Stream *stream,
            size_t &handle,
            void **buffs,
            const long timeoutUs = 100000);

    void releaseWriteBuffer(
            SoapySDR::Stream *stream,
            const size_t handle,
            const size_t numElems,
            int &flags,
            const long long timeNs = 0);

    size_t getNumDirectAccessBuffers(SoapySDR::Stream *stream);

    int getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs);

    /*******************************************************************
     * Settings API
     ******************************************************************/

    SoapySDR::ArgInfoList getSettingInfo(void) const;


    void writeSetting(const std::string &key, const std::string &value);


    std::string readSetting(const std::string &key) const;


    /*******************************************************************
     * Antenna API
     ******************************************************************/

    std::vector<std::string> listAntennas( const int direction, const size_t channel ) const;


    void setAntenna( const int direction, const size_t channel, const std::string &name );


    std::string getAntenna( const int direction, const size_t channel ) const;


    /*******************************************************************
     * Frontend corrections API
     ******************************************************************/

    bool hasDCOffsetMode( const int direction, const size_t channel ) const;


    /*******************************************************************
     * Gain API
     ******************************************************************/

    std::vector<std::string> listGains( const int direction, const size_t channel ) const;


    void setGainMode( const int direction, const size_t channel, const bool automatic );


    bool getGainMode( const int direction, const size_t channel ) const;


    void setGain( const int direction, const size_t channel, const double value );


    void setGain( const int direction, const size_t channel, const std::string &name, const double value );


    double getGain( const int direction, const size_t channel, const std::string &name ) const;


    SoapySDR::Range getGainRange( const int direction, const size_t channel, const std::string &name ) const;


    /*******************************************************************
     * Frequency API
     ******************************************************************/

    void setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs() );


    double getFrequency( const int direction, const size_t channel, const std::string &name ) const;


    SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const;


    std::vector<std::string> listFrequencies( const int direction, const size_t channel ) const;


    SoapySDR::RangeList getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const;


    /*******************************************************************
     * Sample Rate API
     ******************************************************************/

    void setSampleRate( const int direction, const size_t channel, const double rate );


    double getSampleRate( const int direction, const size_t channel ) const;


    std::vector<double> listSampleRates( const int direction, const size_t channel ) const;


    void setBandwidth( const int direction, const size_t channel, const double bw );


    double getBandwidth( const int direction, const size_t channel ) const;


    std::vector<double> listBandwidths( const int direction, const size_t channel ) const;

private:
	iio_device *dev;

};
