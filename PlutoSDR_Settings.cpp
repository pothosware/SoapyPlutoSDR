#include "SoapyPlutoSDR.hpp"
#ifdef HAS_AD9361_IIO
#include <ad9361.h>
#endif

static iio_context *ctx = nullptr; 

SoapyPlutoSDR::SoapyPlutoSDR( const SoapySDR::Kwargs &args ):
	dev(nullptr), rx_dev(nullptr),tx_dev(nullptr), decimation(false), interpolation(false), rx_stream(nullptr)
{

	gainMode = false;

	if (args.count("label") != 0)
		SoapySDR_logf( SOAPY_SDR_INFO, "Opening %s...", args.at("label").c_str());

	if(ctx == nullptr)
	{
	  if(args.count("uri") != 0) {

		  ctx = iio_create_context_from_uri(args.at("uri").c_str());

	  }else if(args.count("hostname")!=0){
		  ctx = iio_create_network_context(args.at("hostname").c_str());
	  }else{
		  ctx = iio_create_default_context();
	  }
	}

	if (ctx == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "not device found.");
		throw std::runtime_error("not device found");
	}

	dev = iio_context_find_device(ctx, "ad9361-phy");
	rx_dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
	tx_dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
	this->setAntenna(SOAPY_SDR_RX, 0, "A_BALANCED");
	this->setAntenna(SOAPY_SDR_TX, 0, "A");

}

SoapyPlutoSDR::~SoapyPlutoSDR(void){

	long long samplerate=0;
	if(decimation){
		iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),"sampling_frequency",&samplerate);
		iio_channel_attr_write_longlong(iio_device_find_channel(rx_dev, "voltage0", false),"sampling_frequency", samplerate);

	}

	if(interpolation){
		iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", true),"sampling_frequency",&samplerate);
		iio_channel_attr_write_longlong(iio_device_find_channel(tx_dev, "voltage0", true),"sampling_frequency", samplerate);
	}

	if(ctx)
	{
		iio_context_destroy(ctx);
		ctx = nullptr;
	}


}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyPlutoSDR::getDriverKey( void ) const
{
	return "PlutoSDR";
}

std::string SoapyPlutoSDR::getHardwareKey( void ) const
{
	return "ADALM-PLUTO";
}

SoapySDR::Kwargs SoapyPlutoSDR::getHardwareInfo( void ) const
{
	SoapySDR::Kwargs info;

	unsigned int major, minor;
	char git_tag[8];
	iio_library_get_version(&major, &minor, git_tag);
	char lib_ver[100];
	snprintf(lib_ver, 100, "%u.%u (git tag: %s)", major, minor, git_tag);
	info["library_version"] = lib_ver;

	iio_context_get_version(ctx, &major, &minor, git_tag);
	char backend_ver[100];
	snprintf(backend_ver, 100, "%u.%u (git tag: %s)", major, minor, git_tag);
	info["backend_version"] = backend_ver;

	unsigned int nb_ctx_attrs = iio_context_get_attrs_count(ctx);
	for (unsigned int i = 0; i < nb_ctx_attrs; i++) {
		const char *key, *value;
		iio_context_get_attr(ctx, i, &key, &value);
		info[key] = value;
	}

	return info;
}


/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyPlutoSDR::getNumChannels( const int dir ) const
{
	return(1);
}

bool SoapyPlutoSDR::getFullDuplex( const int direction, const size_t channel ) const
{
	return(true);
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyPlutoSDR::getSettingInfo(void) const
{
	SoapySDR::ArgInfoList setArgs;

	return setArgs;
}

void SoapyPlutoSDR::writeSetting(const std::string &key, const std::string &value)
{



}


std::string SoapyPlutoSDR::readSetting(const std::string &key) const
{
	std::string info;

	return info;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyPlutoSDR::listAntennas( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	if(direction == SOAPY_SDR_RX) options.push_back( "A_BALANCED" );
	if(direction == SOAPY_SDR_TX) options.push_back( "A" );
	return(options);
}

void SoapyPlutoSDR::setAntenna( const int direction, const size_t channel, const std::string &name )
{
   if (direction == SOAPY_SDR_RX) {
       std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", false), "rf_port_select", name.c_str());
	}

	else if (direction == SOAPY_SDR_TX) {
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", true), "rf_port_select", name.c_str());

	} 
}


std::string SoapyPlutoSDR::getAntenna( const int direction, const size_t channel ) const
{
	std::string options;

	if (direction == SOAPY_SDR_RX) {
		options = "A_BALANCED";
	}
	else if (direction == SOAPY_SDR_TX) {

		options = "A";
	}
	return options;
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyPlutoSDR::hasDCOffsetMode( const int direction, const size_t channel ) const
{
	return(true);
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyPlutoSDR::listGains( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	options.push_back("PGA");
	return(options);
}

bool SoapyPlutoSDR::hasGainMode(const int direction, const size_t channel) const
{
	if (direction == SOAPY_SDR_RX)
		return true;
	return false;
}

void SoapyPlutoSDR::setGainMode( const int direction, const size_t channel, const bool automatic )
{

	gainMode = automatic;
	if(direction==SOAPY_SDR_RX){
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		if (gainMode) {

			iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", false), "gain_control_mode", "slow_attack");

		}else{

			iio_channel_attr_write(iio_device_find_channel(dev, "voltage0", false), "gain_control_mode", "manual");
		}

	}
}

bool SoapyPlutoSDR::getGainMode(const int direction, const size_t channel) const
{
	return gainMode;
}

void SoapyPlutoSDR::setGain( const int direction, const size_t channel, const double value )
{
	long long gain = (long long) value;
	if(direction==SOAPY_SDR_RX){
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", false),"hardwaregain", gain);

	}

	else if(direction==SOAPY_SDR_TX){
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		gain = gain - 89;
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", true),"hardwaregain", gain);

	}

}

void SoapyPlutoSDR::setGain( const int direction, const size_t channel, const std::string &name, const double value )
{
	this->setGain(direction,channel,value);
}

double SoapyPlutoSDR::getGain( const int direction, const size_t channel, const std::string &name ) const
{
	long long gain = 0;

	if(direction==SOAPY_SDR_RX){

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),"hardwaregain",&gain )!=0)
			return 0;

	}

	else if(direction==SOAPY_SDR_TX){

        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", true),"hardwaregain",&gain )!=0)
			return 0;
		gain = gain + 89;
	}
	return double(gain);
}

SoapySDR::Range SoapyPlutoSDR::getGainRange( const int direction, const size_t channel, const std::string &name ) const
{
	if(direction==SOAPY_SDR_RX)
		return(SoapySDR::Range(0, 73));
	return(SoapySDR::Range(0,89));

}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyPlutoSDR::setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args )
{
	long long freq = (long long)frequency;
	if(direction==SOAPY_SDR_RX){

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "altvoltage0", true),"frequency", freq);
	}

	else if(direction==SOAPY_SDR_TX){
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "altvoltage1", true),"frequency", freq);

	}

}

double SoapyPlutoSDR::getFrequency( const int direction, const size_t channel, const std::string &name ) const
{
  	long long freq;

	if(direction==SOAPY_SDR_RX){

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "altvoltage0", true),"frequency",&freq )!=0)
			return 0;

	}

	else if(direction==SOAPY_SDR_TX){

        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "altvoltage1", true),"frequency",&freq )!=0)
			return 0;

	}

	return double(freq);

}

SoapySDR::ArgInfoList SoapyPlutoSDR::getFrequencyArgsInfo(const int direction, const size_t channel) const
{

	SoapySDR::ArgInfoList freqArgs;

	return freqArgs;
}

std::vector<std::string> SoapyPlutoSDR::listFrequencies( const int direction, const size_t channel ) const
{
	std::vector<std::string> names;
	names.push_back( "RF" );
	return(names);
}

SoapySDR::RangeList SoapyPlutoSDR::getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const
{
	return(SoapySDR::RangeList( 1, SoapySDR::Range( 70000000, 6000000000ull ) ) );

}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/
void SoapyPlutoSDR::setSampleRate( const int direction, const size_t channel, const double rate )
{
	long long samplerate =(long long) rate;

	if(direction==SOAPY_SDR_RX){
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		decimation = false;
		if (samplerate<(25e6 / 48)) {
			if (samplerate * 8 < (25e6 / 48)) {
				SoapySDR_logf(SOAPY_SDR_CRITICAL, "sample rate is not supported.");
			}

			decimation = true;
			samplerate = samplerate * 8;
		}

		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", false),"sampling_frequency", samplerate);

		iio_channel_attr_write_longlong(iio_device_find_channel(rx_dev, "voltage0", false), "sampling_frequency", decimation?samplerate/8:samplerate);

		if(rx_stream)
			rx_stream->set_buffer_size_by_samplerate(decimation ? samplerate / 8 : samplerate);
	}

	else if(direction==SOAPY_SDR_TX){
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		interpolation = false;
		if (samplerate<(25e6 / 48)) {
			if (samplerate * 8 < (25e6 / 48)) {
				SoapySDR_logf(SOAPY_SDR_CRITICAL, "sample rate is not supported.");
			}

			interpolation = true;
			samplerate = samplerate * 8;
		}


		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", true),"sampling_frequency", samplerate);
		iio_channel_attr_write_longlong(iio_device_find_channel(tx_dev, "voltage0", true), "sampling_frequency", interpolation?samplerate / 8:samplerate);

	}

#ifdef HAS_AD9361_IIO
	if(ad9361_set_bb_rate(dev,(unsigned long)samplerate))
		SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to set BB rate.");	
#endif

}

double SoapyPlutoSDR::getSampleRate( const int direction, const size_t channel ) const
{
	long long samplerate;

	if(direction==SOAPY_SDR_RX){

        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(rx_dev, "voltage0", false),"sampling_frequency",&samplerate )!=0)
			return 0;
	}

	else if(direction==SOAPY_SDR_TX){
        
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(tx_dev, "voltage0", true),"sampling_frequency",&samplerate)!=0)
			return 0;

	}

	return double(samplerate);

}

std::vector<double> SoapyPlutoSDR::listSampleRates( const int direction, const size_t channel ) const
{
	std::vector<double> options;

	options.push_back(65105);//25M/48/8+1
	options.push_back(1e6);
	options.push_back(2e6);
	options.push_back(3e6);
	options.push_back(4e6);
	options.push_back(5e6);
	options.push_back(6e6);
	options.push_back(7e6);
	options.push_back(8e6);
	options.push_back(9e6);
	options.push_back(10e6);
	return(options);

}

void SoapyPlutoSDR::setBandwidth( const int direction, const size_t channel, const double bw )
{
	long long bandwidth = (long long) bw;
	if(direction==SOAPY_SDR_RX){
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", false),"rf_bandwidth", bandwidth);
	}

	else if(direction==SOAPY_SDR_TX){
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		iio_channel_attr_write_longlong(iio_device_find_channel(dev, "voltage0", true),"rf_bandwidth", bandwidth);
	}

}

double SoapyPlutoSDR::getBandwidth( const int direction, const size_t channel ) const
{
    long long bandwidth;

	if(direction==SOAPY_SDR_RX){
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),"rf_bandwidth",&bandwidth )!=0)
			return 0;

	}

	else if(direction==SOAPY_SDR_TX){
        std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);

		if(iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", true),"rf_bandwidth",&bandwidth )!=0)
			return 0;
	}

	return double(bandwidth);

}

std::vector<double> SoapyPlutoSDR::listBandwidths( const int direction, const size_t channel ) const
{
	std::vector<double> options;
	options.push_back(0.2e6);
	options.push_back(1e6);
	options.push_back(2e6);
	options.push_back(3e6);
	options.push_back(4e6);
	options.push_back(5e6);
	options.push_back(6e6);
	options.push_back(7e6);
	options.push_back(8e6);
	options.push_back(9e6);
	options.push_back(10e6);
	return(options);

}
