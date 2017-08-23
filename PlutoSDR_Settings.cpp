#include "SoapyPlutoSDR.hpp"


SoapyPlutoSDR::SoapyPlutoSDR( const SoapySDR::Kwargs &args ){

	if (args.count("label") != 0)
		SoapySDR_logf( SOAPY_SDR_INFO, "Opening %s...", args.at("label").c_str());

	if (args.count("backend") == 0)
		throw std::runtime_error("no PlutoSDR device matches");

	if(args.at("backend")=="network"){
		ctx = iio_create_network_context(args.at("hostname").c_str());
	}else if(args.at("backend")=="url"){
		ctx = iio_create_context_from_uri(args.at("url").c_str());	
	}else{
		ctx = iio_create_default_context();
	}

	phy_dev=iio_context_find_device(ctx, "ad9361-phy");
	rx_stream.dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
	tx_stream.dev =iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");	

	rx_stream.phy_chn = iio_device_find_channel(phy_dev, "voltage0", false);
	tx_stream.phy_chn = iio_device_find_channel(phy_dev, "voltage0", true);
	rx_stream.lo_chn = iio_device_find_channel(phy_dev, "altvoltage0", true);
	tx_stream.lo_chn = iio_device_find_channel(phy_dev, "altvoltage1", true);

	rx_stream.direction=SOAPY_SDR_RX;
	rx_stream.rfport = "A_BALANCED";
	tx_stream.direction=SOAPY_SDR_TX;
	tx_stream.rfport = "A";

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
	/* TODO delete this function or throw if name != RX... */
}


std::string SoapyPlutoSDR::getAntenna( const int direction, const size_t channel ) const
{
	std::string options;
	if(direction == SOAPY_SDR_RX) options=rx_stream.rfport;
	if(direction == SOAPY_SDR_TX) options=tx_stream.rfport;
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

			iio_channel_attr_write(rx_stream.phy_chn, "gain_control_mode", "slow_attack");

		}else{

			iio_channel_attr_write(rx_stream.phy_chn, "gain_control_mode", "manual");
			this->setGain(direction,channel,rx_stream.gain);


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
	if(direction==SOAPY_SDR_RX){

		iio_channel_attr_write_longlong(rx_stream.phy_chn,"hardwaregain",rx_stream.gain);

	}

	if(direction==SOAPY_SDR_TX){

		iio_channel_attr_write_longlong(tx_stream.phy_chn,"hardwaregain",tx_stream.gain);

	}

}

void SoapyPlutoSDR::setGain( const int direction, const size_t channel, const std::string &name, const double value )
{
	this->setGain(direction,channel,value);

}

double SoapyPlutoSDR::getGain( const int direction, const size_t channel, const std::string &name ) const
{
	long long gain;

	if(direction==SOAPY_SDR_RX){

		if(iio_channel_attr_read_longlong(rx_stream.lo_chn,"hardwaregain",&gain )!=0)
			return 0;

		rx_stream.gain = gain;

	}

	if(direction==SOAPY_SDR_TX){

		if(iio_channel_attr_read_longlong(tx_stream.lo_chn,"hardwaregain",&gain )!=0)
			return 0;

		tx_stream.gain = gain;

	}
	return gain;
}

SoapySDR::Range SoapyPlutoSDR::getGainRange( const int direction, const size_t channel, const std::string &name ) const
{

	return(SoapySDR::Range( 0, 0 ) );

}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyPlutoSDR::setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args )
{
	if(direction==SOAPY_SDR_RX){
		rx_stream.lo_hz = frequency;
		iio_channel_attr_write_longlong(rx_stream.lo_chn,"frequency",rx_stream.lo_hz );
	}

	if(direction==SOAPY_SDR_TX){
		tx_stream.lo_hz = frequency;
		iio_channel_attr_write_longlong(tx_stream.lo_chn,"frequency",tx_stream.lo_hz );

	}

}

double SoapyPlutoSDR::getFrequency( const int direction, const size_t channel, const std::string &name ) const
{
	long long freq;

	if(direction==SOAPY_SDR_RX){

		if(iio_channel_attr_read_longlong(rx_stream.lo_chn,"frequency",&freq )!=0)
			return 0;

		rx_stream.lo_hz=freq;

	}

	if(direction==SOAPY_SDR_TX){

		if(iio_channel_attr_read_longlong(tx_stream.lo_chn,"frequency",&freq )!=0)
			return 0;

		tx_stream.lo_hz=freq;
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
	if(direction==SOAPY_SDR_RX){
		rx_stream.fs_hz = rate;
		iio_channel_attr_write_longlong(rx_stream.phy_chn,"sampling_frequency",rx_stream.fs_hz );
	}

	if(direction==SOAPY_SDR_TX){
		tx_stream.fs_hz = rate;
		iio_channel_attr_write_longlong(tx_stream.phy_chn,"sampling_frequency",tx_stream.fs_hz );

	}

}

double SoapyPlutoSDR::getSampleRate( const int direction, const size_t channel ) const
{
	long long samplerate;

	if(direction==SOAPY_SDR_RX){

		if(iio_channel_attr_read_longlong(rx_stream.phy_chn,"sampling_frequency",&samplerate )!=0)
			return 0;

		rx_stream.fs_hz=samplerate;
	}

	if(direction==SOAPY_SDR_TX){

		if(iio_channel_attr_read_longlong(tx_stream.phy_chn,"sampling_frequency",&samplerate)!=0)
			return 0;
		tx_stream.fs_hz=samplerate;
	}


	return double(samplerate);

}

std::vector<double> SoapyPlutoSDR::listSampleRates( const int direction, const size_t channel ) const
{
	std::vector<double> options;

	return(options);

}

void SoapyPlutoSDR::setBandwidth( const int direction, const size_t channel, const double bw )
{
	if(direction==SOAPY_SDR_RX){
		rx_stream.bw_hz = bw;
		iio_channel_attr_write_longlong(rx_stream.phy_chn,"rf_bandwidth",rx_stream.bw_hz );
	}

	if(direction==SOAPY_SDR_TX){
		tx_stream.bw_hz = bw;
		iio_channel_attr_write_longlong(tx_stream.phy_chn,"rf_bandwidth",tx_stream.bw_hz );

	}

}

double SoapyPlutoSDR::getBandwidth( const int direction, const size_t channel ) const
{
	long long bandwidth;

	if(direction==SOAPY_SDR_RX){

		if(iio_channel_attr_read_longlong(rx_stream.phy_chn,"rf_bandwidth",&bandwidth )!=0)
			return 0;
		rx_stream.bw_hz=bandwidth;

	}

	if(direction==SOAPY_SDR_TX){

		if(iio_channel_attr_read_longlong(tx_stream.phy_chn,"rf_bandwidth",&bandwidth )!=0)
			return 0;
		tx_stream.bw_hz=bandwidth;
	}

	return double(bandwidth);

}

std::vector<double> SoapyPlutoSDR::listBandwidths( const int direction, const size_t channel ) const
{
	std::vector<double> options;

	return(options);

}
