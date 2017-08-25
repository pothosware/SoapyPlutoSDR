#include <iio.h>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>

#define PLUTOSDR_DEFAULT_IP "192.168.2.1"
#define PLUTOSDR_DEFAULT_HOSTNAME "pluto.local"

class rx_streamer {
	public:
		rx_streamer(const iio_context *ctx, const std::string &format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args);
		~rx_streamer();
		size_t recv(void * const *buffs,
				const size_t numElems,
				int &flags,
				long long &timeNs,
				const long timeoutUs=100000);
		int start(const int flags,
				const long long timeNs,
				const size_t numElems);

		int stop(const int flags,
				const long long timeNs=100000);

	private:

		void set_buffer_size(const size_t _buffer_size);

		void channel_read(const struct iio_channel *chn, void *dst, size_t len);

		void refill_thread();

		std::thread refill_thd;
		std::mutex mutex;
		std::condition_variable cond, cond2;
		std::vector<iio_channel* > channel_list;
		volatile bool thread_stopped, please_refill_buffer;
		iio_device  *dev;

		std::vector<int16_t> buffer;
		size_t buffer_size;
		size_t byte_offset;
		size_t items_in_buffer;
		iio_buffer  *buf;
		std::string format;

};

class tx_streamer {

	public:
		tx_streamer(const iio_context *ctx, const std::string &format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args);
		~tx_streamer();
		int send(const void * const *buffs,const size_t numElems,int &flags,const long long timeNs,const long timeoutUs );

	private:

		void channel_write(iio_channel *chn,const void *src, size_t len);
		std::vector<iio_channel* > channel_list;
		iio_device  *dev;
		std::vector<int16_t> buffer;
		std::string format;
		size_t buffer_size;
		bool cyclic;
		iio_buffer  *buf;
};



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

		iio_context *ctx;
		iio_device *dev;

};
