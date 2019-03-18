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
	/*
	18 device-specific attributes
		 0: calib_mode value: auto
		 1: calib_mode_available value: auto manual manual_tx_quad tx_quad rf_dc_offs rssi_gain_step
		 2: dcxo_tune_coarse ERROR: Operation not supported by device (-19)
		 3: dcxo_tune_coarse_available value: [0 0 0]
		 4: dcxo_tune_fine ERROR: Operation not supported by device (-19)
		 5: dcxo_tune_fine_available value: [0 0 0]
		 6: ensm_mode value: fdd
		 7: ensm_mode_available value: sleep wait alert fdd pinctrl pinctrl_fdd_indep
		 8: filter_fir_config value: FIR Rx: 0,0 Tx: 0,0
		 9: gain_table_config ERROR: Input/output error (-5)
		10: multichip_sync ERROR: Permission denied (-13)
		11: rssi_gain_step_error value: lna_error: 0 0 0 0\nmixer_error: 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\ngain_step_calib_reg_val: 0 0 0 0 0
		12: rx_path_rates value: BBPLL:983040004 ADC:245760001 R2:122880000 R1:61440000 RF:30720000 RXSAMP:30720000
		13: trx_rate_governor value: nominal
		14: trx_rate_governor_available value: nominal highest_osr
		15: tx_path_rates value: BBPLL:983040004 DAC:122880000 T2:122880000 T1:61440000 TF:30720000 TXSAMP:30720000
		16: xo_correction value: 39999976
		17: xo_correction_available value: [39991977 1 40007975]
	*/
	unsigned int attrs_count = iio_device_get_attrs_count(dev);

	for (unsigned int index = 0; index < attrs_count; ++index)
	{
		SoapySDR::ArgInfo arg;

		const char *attr = iio_device_get_attr(dev, index);
		arg.key = attr;
		//arg.value = "true";
		arg.name = attr;
		arg.description = attr;
		//arg.type = SoapySDR::ArgInfo::BOOL;
		setArgs.push_back(arg);
	}

	return setArgs;
}

void SoapyPlutoSDR::writeSetting(const std::string &key, const std::string &value)
{
	std::lock_guard<pluto_spin_mutex> rx_lock(rx_device_mutex);
	std::lock_guard<pluto_spin_mutex> tx_lock(rx_device_mutex);
	iio_device_attr_write(dev, key.c_str(), value.c_str());
}


std::string SoapyPlutoSDR::readSetting(const std::string &key) const
{
	std::string info;
	char value[128];
	ssize_t len;

	{
		std::lock_guard<pluto_spin_mutex> rx_lock(rx_device_mutex);
		std::lock_guard<pluto_spin_mutex> tx_lock(rx_device_mutex);
		len = iio_device_attr_read(dev, key.c_str(), value, sizeof(value));
	}

	if (len > 0)
	{
		info.assign(value, len);
	}

	return info;
}

SoapySDR::ArgInfoList SoapyPlutoSDR::getSettingInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList setArgs;
	/*
	voltage2:  (output) 8 channel-specific attributes found:
		0: filter_fir_en value: 0
		1: raw value: 306
		2: rf_bandwidth value: 18000000
		3: rf_bandwidth_available value: [200000 1 40000000]
		4: rf_port_select_available value: A B
		5: sampling_frequency value: 30720000
		6: sampling_frequency_available value: [2083333 1 61440000]
		7: scale value: 1.000000
	voltage2:  (input)	13 channel-specific attributes found:
		 0: bb_dc_offset_tracking_en value: 1
		 1: filter_fir_en value: 0
		 2: gain_control_mode_available value: manual fast_attack slow_attack hybrid
		 3: offset value: 57
		 4: quadrature_tracking_en value: 1
		 5: raw value: 2004
		 6: rf_bandwidth value: 18000000
		 7: rf_bandwidth_available value: [200000 1 56000000]
		 8: rf_dc_offset_tracking_en value: 1
		 9: rf_port_select_available value: A_BALANCED B_BALANCED C_BALANCED A_N A_P B_N B_P C_N C_P TX_MONITOR1 TX_MONITOR2 TX_MONITOR1_2
		10: sampling_frequency value: 30720000
		11: sampling_frequency_available value: [2083333 1 61440000]
		12: scale value: 0.305250
	*/

	iio_channel *chn = iio_device_find_channel(dev, "voltage2", (direction == SOAPY_SDR_TX));

	unsigned int attrs_count = iio_channel_get_attrs_count(chn);

	for (unsigned int index = 0; index < attrs_count; ++index)
	{
		SoapySDR::ArgInfo arg;

		const char *attr = iio_channel_get_attr(chn, index);
		arg.key = attr;
		//arg.value = "true";
		arg.name = attr;
		arg.description = attr;
		//arg.type = SoapySDR::ArgInfo::BOOL;
		setArgs.push_back(arg);
	}

	return setArgs;
}

void SoapyPlutoSDR::writeSetting(const int direction, const size_t channel, const std::string &key, const std::string &value)
{
	if (direction == SOAPY_SDR_RX)
	{
		std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		iio_channel_attr_write(iio_device_find_channel(dev, "voltage2", false), key.c_str(), value.c_str());
	}

	else if (direction == SOAPY_SDR_TX)
	{
		std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		iio_channel_attr_write(iio_device_find_channel(dev, "voltage2", true), key.c_str(), value.c_str());
	}
}

std::string SoapyPlutoSDR::readSetting(const int direction, const size_t channel, const std::string &key) const
{
	std::string info;
	char value[128]; // widest attr value seen is 95 chars
	ssize_t len;

	if (direction == SOAPY_SDR_RX)
	{
		std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		len = iio_channel_attr_read(iio_device_find_channel(dev, "voltage2", false), key.c_str(), value, sizeof(value));
	}

	else if (direction == SOAPY_SDR_TX)
	{
		std::lock_guard<pluto_spin_mutex> lock(tx_device_mutex);
		len = iio_channel_attr_read(iio_device_find_channel(dev, "voltage2", true), key.c_str(), value, sizeof(value));
	}

	if (len > 0)
	{
		info.assign(value, len);
	}

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
