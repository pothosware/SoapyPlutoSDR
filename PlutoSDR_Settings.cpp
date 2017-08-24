#include "SoapyPlutoSDR.hpp"


SoapyPlutoSDR::SoapyPlutoSDR( const SoapySDR::Kwargs &args ):
	ctx(nullptr){

	if (args.count("label") != 0)
		SoapySDR_logf( SOAPY_SDR_INFO, "Opening %s...", args.at("label").c_str());


	if(args.count("uri") != 0) {

		ctx = iio_create_context_from_uri(args.at("uri").c_str());

	}else if(args.count("hostname")!=0){
		ctx = iio_create_network_context(args.at("hostname").c_str());
	}else{
		ctx = iio_create_default_context();
	}

	if (ctx == NULL)
		throw std::runtime_error("not device found");

	this->setAntenna(SOAPY_SDR_RX, 0, "A_BALANCED");


}

SoapyPlutoSDR::~SoapyPlutoSDR(void){

	if(ctx)iio_context_destroy(ctx);


}

/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyPlutoSDR::getDriverKey( void ) const
{

	return("PlutoSDR");
}

std::string SoapyPlutoSDR::getHardwareKey( void ) const
{

	return("PlutoSDR");
}

SoapySDR::Kwargs SoapyPlutoSDR::getHardwareInfo( void ) const
{

	SoapySDR::Kwargs info;

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

		iio_channel_attr_write(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", false), "rf_port_select", name.c_str());
	}

	if (direction == SOAPY_SDR_TX) {

		iio_channel_attr_write(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", true), "rf_port_select", name.c_str());
	
	} 
}


std::string SoapyPlutoSDR::getAntenna( const int direction, const size_t channel ) const
{
	std::string options;

	if (direction == SOAPY_SDR_RX) {
		options = "A_BALANCED";
	}
	if (direction == SOAPY_SDR_TX) {
	
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

	return(options);
}

void SoapyPlutoSDR::setGainMode( const int direction, const size_t channel, const bool automatic )
{
	if(direction==SOAPY_SDR_RX){

		if(automatic) {

			iio_channel_attr_write(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", false), "gain_control_mode", "slow_attack");

		}else{

			iio_channel_attr_write(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", false), "gain_control_mode", "manual");
		}

	}
}

bool SoapyPlutoSDR::getGainMode( const int direction, const size_t channel ) const
{
	if(direction==SOAPY_SDR_RX)
		return true;
	return(false);
}

void SoapyPlutoSDR::setGain( const int direction, const size_t channel, const double value )
{
	long long gain = (long long) value;

	if(direction==SOAPY_SDR_RX){

		iio_channel_attr_write_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", false),"hardwaregain", gain);

	}

	if(direction==SOAPY_SDR_TX){

		iio_channel_attr_write_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", true),"hardwaregain", gain);

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

		if(iio_channel_attr_read_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", false),"hardwaregain",&gain )!=0)
			return 0;

	}

	if(direction==SOAPY_SDR_TX){

		if(iio_channel_attr_read_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", true),"hardwaregain",&gain )!=0)
			return 0;
	}
	return double(gain);
}

SoapySDR::Range SoapyPlutoSDR::getGainRange( const int direction, const size_t channel, const std::string &name ) const
{

	return(SoapySDR::Range( 0, 50 ) );

}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyPlutoSDR::setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args )
{
	long long freq = (long long)frequency;
	if(direction==SOAPY_SDR_RX){

		iio_channel_attr_write_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage0", true),"frequency", freq);
	}

	if(direction==SOAPY_SDR_TX){

		iio_channel_attr_write_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true),"frequency", freq);

	}

}

double SoapyPlutoSDR::getFrequency( const int direction, const size_t channel, const std::string &name ) const
{
	long long freq;

	if(direction==SOAPY_SDR_RX){

		if(iio_channel_attr_read_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage0", true),"frequency",&freq )!=0)
			return 0;

	}

	if(direction==SOAPY_SDR_TX){

		if(iio_channel_attr_read_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true),"frequency",&freq )!=0)
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
	long long samplerate = rate;

	if(direction==SOAPY_SDR_RX){

		iio_channel_attr_write_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", false),"sampling_frequency", samplerate);
	}

	if(direction==SOAPY_SDR_TX){

		iio_channel_attr_write_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", true),"sampling_frequency", samplerate);

	}

}

double SoapyPlutoSDR::getSampleRate( const int direction, const size_t channel ) const
{
	long long samplerate;

	if(direction==SOAPY_SDR_RX){

		if(iio_channel_attr_read_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", false),"sampling_frequency",&samplerate )!=0)
			return 0;
	}

	if(direction==SOAPY_SDR_TX){

		if(iio_channel_attr_read_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", true),"sampling_frequency",&samplerate)!=0)
			return 0;

	}


	return double(samplerate);

}

std::vector<double> SoapyPlutoSDR::listSampleRates( const int direction, const size_t channel ) const
{
	std::vector<double> options;

	options.push_back(2.5e6);

	return(options);

}

void SoapyPlutoSDR::setBandwidth( const int direction, const size_t channel, const double bw )
{
	long long bandwidth = bw;
	if(direction==SOAPY_SDR_RX){

		iio_channel_attr_write_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", false),"rf_bandwidth", bandwidth);
	}

	if(direction==SOAPY_SDR_TX){

		iio_channel_attr_write_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", true),"rf_bandwidth", bandwidth);

	}

}

double SoapyPlutoSDR::getBandwidth( const int direction, const size_t channel ) const
{
	long long bandwidth;

	if(direction==SOAPY_SDR_RX){

		if(iio_channel_attr_read_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", false),"rf_bandwidth",&bandwidth )!=0)
			return 0;

	}

	if(direction==SOAPY_SDR_TX){

		if(iio_channel_attr_read_longlong(iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "voltage0", true),"rf_bandwidth",&bandwidth )!=0)
			return 0;
	}

	return double(bandwidth);

}

std::vector<double> SoapyPlutoSDR::listBandwidths( const int direction, const size_t channel ) const
{
	std::vector<double> options;

	return(options);

}
