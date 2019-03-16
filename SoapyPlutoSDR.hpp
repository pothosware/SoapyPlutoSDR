#include <iio.h>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>

typedef enum plutosdrStreamFormat {
	PLUTO_SDR_CF32,
	PLUTO_SDR_CS16,
	PLUTO_SDR_CS12,
	PLUTO_SDR_CS8
} plutosdrStreamFormat;

class rx_streamer {
	public:
		rx_streamer(const iio_device *dev, const plutosdrStreamFormat format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args);
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

		void set_buffer_size_by_samplerate(const size_t _samplerate);

        size_t get_mtu_size();

	private:

		void set_buffer_size(const size_t _buffer_size);
        void set_mtu_size(const size_t mtu_size);

		bool has_direct_copy();

		std::vector<iio_channel* > channel_list;
		const iio_device  *dev;

		size_t buffer_size;
		size_t byte_offset;
		size_t items_in_buffer;
		iio_buffer  *buf;
		const plutosdrStreamFormat format;
		bool direct_copy;
        size_t mtu_size;

};

class tx_streamer {

	public:
		tx_streamer(const iio_device *dev, const plutosdrStreamFormat format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args);
		~tx_streamer();
		int send(const void * const *buffs,const size_t numElems,int &flags,const long long timeNs,const long timeoutUs );
		int flush();

	private:
		int send_buf();
		bool has_direct_copy();

		std::vector<iio_channel* > channel_list;
		const iio_device  *dev;
		const plutosdrStreamFormat format;
		
		iio_buffer  *buf;
		size_t buf_size;
		size_t items_in_buf;
		bool direct_copy;

};

// A local spin_mutex usable with std::lock_guard
       //for lightweight locking for short periods.
class pluto_spin_mutex {

public:
    pluto_spin_mutex() = default;

    pluto_spin_mutex(const pluto_spin_mutex&) = delete;

    pluto_spin_mutex& operator=(const pluto_spin_mutex&) = delete;

    ~pluto_spin_mutex() { lock_state.clear(std::memory_order_release); }

    void lock() { while (lock_state.test_and_set(std::memory_order_acquire)); }

    void unlock() { lock_state.clear(std::memory_order_release); }

private:
    std::atomic_flag lock_state = ATOMIC_FLAG_INIT;
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


		bool hasGainMode(const int direction, const size_t channel) const;


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

        bool IsValidRxStreamHandle(SoapySDR::Stream* handle) const;
        bool IsValidTxStreamHandle(SoapySDR::Stream* handle);
       
		iio_device *dev;
		iio_device *rx_dev;
		iio_device *tx_dev;
		bool gainMode;

		mutable pluto_spin_mutex rx_device_mutex;
        mutable pluto_spin_mutex tx_device_mutex;

		bool decimation, interpolation;
		std::unique_ptr<rx_streamer> rx_stream;
        std::unique_ptr<tx_streamer> tx_stream;

        
};

