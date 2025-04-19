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
		SoapySDR_logf(SOAPY_SDR_ERROR, "no device context found.");
		throw std::runtime_error("no device context found");
	}

	dev = iio_context_find_device(ctx, "ad9361-phy");
	rx_dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
	tx_dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");

	if (dev == nullptr || rx_dev == nullptr || tx_dev == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "no device found in this context.");
		throw std::runtime_error("no device found in this context");
	}

	this->setAntenna(SOAPY_SDR_RX, 0, "A_BALANCED");
	this->setGainMode(SOAPY_SDR_RX, 0, false);
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
 * Sensor API
 ******************************************************************/

bool SoapyPlutoSDR::is_sensor_channel(struct iio_channel *chn) const
{
	return (!iio_channel_is_output(chn) &&
			(iio_channel_find_attr(chn, "raw") ||
			iio_channel_find_attr(chn, "input")));
}

double SoapyPlutoSDR::double_from_buf(const char *buf) const
{
	std::istringstream val_as_string(buf);
	val_as_string.imbue(std::locale::classic()); // ignore global C++ locale

	double val = 0.0;
	val_as_string >> val;

	return val;
}

double SoapyPlutoSDR::get_sensor_value(struct iio_channel *chn) const
{
	char buf[32];
	double val = 0.0;

	if (iio_channel_find_attr(chn, "input")) {
		if (iio_channel_attr_read(chn, "input", buf, sizeof(buf)) > 0) {
			val = double_from_buf(buf);
		}
	} else {
		if (iio_channel_attr_read(chn, "raw", buf, sizeof(buf)) > 0) {
			val = double_from_buf(buf);
		}

		if (iio_channel_find_attr(chn, "offset")) {
			if (iio_channel_attr_read(chn, "offset", buf, sizeof(buf)) > 0) {
				val += double_from_buf(buf);
			}
		}

		if (iio_channel_find_attr(chn, "scale")) {
			if (iio_channel_attr_read(chn, "scale", buf, sizeof(buf)) > 0) {
				val *= double_from_buf(buf);
			}
		}
	}

	return val / 1000.0;
}

std::string SoapyPlutoSDR::id_to_unit(const std::string& id) const
{
	static std::map<std::string, std::string> id_to_unit_table = {
		{ "current",	"A" },
		{ "power",	"W" },
		{ "temp",	"C" },
		{ "voltage",	"V" },
	};

	for (auto it_match : id_to_unit_table) {

		//if the id starts with a known prefix, retreive its unit.
		if (id.substr(0, it_match.first.size()) == it_match.first) {
			return it_match.second;
		}
	}
	return std::string();
}

std::vector<std::string> SoapyPlutoSDR::listSensors(void) const
{
	/*
	iio:device2: xadc
		10 channels found:
			temp0:  (input)
			voltage0: vccint (input)
			voltage1: vccaux (input)
			voltage2: vccbram (input)
			voltage3: vccpint (input)
			voltage4: vccpaux (input)
			voltage5: vccoddr (input)
			voltage6: vrefp (input)
			voltage7: vrefn (input)
			voltage8:  (input)
	iio:device0: adm1177
		2 channels found:
			current0:  (input)
			voltage0:  (input)
 	iio:device1: ad9361-phy
		9 channels found:
			temp0:  (input)
			voltage2:  (input)
	*/
	std::vector<std::string> sensors;

	sensors.push_back("xadc_temp0");
	sensors.push_back("xadc_voltage0");
	sensors.push_back("xadc_voltage1");
	sensors.push_back("xadc_voltage2");
	sensors.push_back("xadc_voltage3");
	sensors.push_back("xadc_voltage4");
	sensors.push_back("xadc_voltage5");
	sensors.push_back("xadc_voltage6");
	sensors.push_back("xadc_voltage7");
	sensors.push_back("xadc_voltage8");
	sensors.push_back("adm1177_current0");
	sensors.push_back("adm1177_voltage0");
	sensors.push_back("ad9361-phy_temp0");
	sensors.push_back("ad9361-phy_voltage2");

	return sensors;
}

SoapySDR::ArgInfo SoapyPlutoSDR::getSensorInfo(const std::string &key) const
{
	SoapySDR::ArgInfo info;

	std::size_t dash = key.find("_");
	if (dash < std::string::npos)
	{
		std::string deviceStr = key.substr(0, dash);
		std::string channelStr = key.substr(dash + 1);

		iio_device *dev = iio_context_find_device(ctx, deviceStr.c_str());
		if (!dev)
			return info;
		iio_channel *chn = iio_device_find_channel(dev, channelStr.c_str(), false);
		if (!chn)
			return info;

		const char *name = iio_channel_get_name(chn);
		info.key = key;
		if (name)
			info.name = name;
		info.type = SoapySDR::ArgInfo::FLOAT;
		info.value = "0.0";
		info.units = id_to_unit(channelStr);
	}

	return info;
}

std::string SoapyPlutoSDR::readSensor(const std::string &key) const
{
	std::string sensorValue;

	std::size_t dash = key.find("_");
	if (dash < std::string::npos)
	{
		std::string deviceStr = key.substr(0, dash);
		std::string channelStr = key.substr(dash + 1);

		iio_device *dev = iio_context_find_device(ctx, deviceStr.c_str());
		if (!dev)
			return sensorValue;
		iio_channel *chn = iio_device_find_channel(dev, channelStr.c_str(), false);
		if (!chn)
			return sensorValue;

		double value = get_sensor_value(chn);
		sensorValue.assign(std::to_string(value));
	}

	return sensorValue;
}


/*******************************************************************
 * Settings API
 ******************************************************************/

std::vector<std::string> Split(const std::string &subject)
{
	std::istringstream ss{subject};
	using StrIt = std::istream_iterator<std::string>;
	std::vector<std::string> container{StrIt{ss}, StrIt{}};
	return container;
}

SoapySDR::ArgInfoList SoapyPlutoSDR::getSettingInfo(void) const
{
	SoapySDR::ArgInfoList setArgs;
	/*
	18 device-specific attributes
		 0: calib_mode 'auto'
		 	1: calib_mode_available 'auto manual manual_tx_quad tx_quad rf_dc_offs rssi_gain_step'
		 2: dcxo_tune_coarse ERROR: Operation not supported by device (-19)
		 	3: dcxo_tune_coarse_available '[0 0 0]'
		 4: dcxo_tune_fine ERROR: Operation not supported by device (-19)
		 	5: dcxo_tune_fine_available '[0 0 0]'
		 6: ensm_mode 'fdd'
		 	7: ensm_mode_available 'sleep wait alert fdd pinctrl pinctrl_fdd_indep'
		 8: filter_fir_config 'FIR Rx: 0,0 Tx: 0,0'
		 9: gain_table_config ERROR: Input/output error (-5)
		10: multichip_sync ERROR: Permission denied (-13)
		11: rssi_gain_step_error 'lna_error: 0 0 0 0\nmixer_error: 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0\ngain_step_calib_reg_val: 0 0 0 0 0'
		12: rx_path_rates 'BBPLL:983040004 ADC:245760001 R2:122880000 R1:61440000 RF:30720000 RXSAMP:30720000'
		13: trx_rate_governor 'nominal'
			14: trx_rate_governor_available 'nominal highest_osr'
		15: tx_path_rates 'BBPLL:983040004 DAC:122880000 T2:122880000 T1:61440000 TF:30720000 TXSAMP:30720000'
		16: xo_correction '39999976'
			17: xo_correction_available '[39991977 1 40007975]'
	*/

	// This should work for the general case but isn't very useful
	/*
	unsigned int attrs_count = iio_device_get_attrs_count(dev);
	const char *attrs[attrs_count];
	for (unsigned int index = 0; index < attrs_count; ++index)
	{
		attrs[index] = iio_device_get_attr(dev, index);
	}

	for (unsigned int index = 0; index < attrs_count; ++index)
	{
		const char *attr = attrs[index];
		size_t attr_len = strlen(attr);
		if (attr_len < (sizeof("_available") - 1)
				|| strcmp(attr + attr_len - (sizeof("_available") - 1), "_available"))
		{
			SoapySDR::ArgInfo arg;
			arg.key = attr;
			//arg.value = "true";
			arg.name = attr;
			//arg.description = attr;
			arg.type = SoapySDR::ArgInfo::STRING;
			if (index <= 9) // huh?
			setArgs.push_back(arg);
		}
	}
	*/

	std::vector<std::string> calib_mode_available;
	calib_mode_available.push_back("auto");
	calib_mode_available.push_back("manual");
	calib_mode_available.push_back("manual_tx_quad");
	calib_mode_available.push_back("tx_quad");
	calib_mode_available.push_back("rf_dc_offs");
	calib_mode_available.push_back("rssi_gain_step");

	SoapySDR::ArgInfo calib_mode;
	calib_mode.key = "calib_mode";
	calib_mode.value = "auto";
	calib_mode.name = "Calibration Mode";
	calib_mode.options = calib_mode_available;
	calib_mode.type = SoapySDR::ArgInfo::STRING;
	setArgs.push_back(calib_mode);

	std::vector<std::string> ensm_mode_available;
	ensm_mode_available.push_back("sleep");
	ensm_mode_available.push_back("wait");
	ensm_mode_available.push_back("alert");
	ensm_mode_available.push_back("fdd");
	ensm_mode_available.push_back("pinctrl");
	ensm_mode_available.push_back("pinctrl_fdd_indep");

	SoapySDR::ArgInfo ensm_mode;
	ensm_mode.key = "ensm_mode";
	ensm_mode.value = "fdd";
	ensm_mode.name = "Ensm Mode";
	ensm_mode.options = ensm_mode_available;
	ensm_mode.type = SoapySDR::ArgInfo::STRING;
	setArgs.push_back(ensm_mode);

	std::vector<std::string> trx_rate_governor_available;
	trx_rate_governor_available.push_back("nominal");
	trx_rate_governor_available.push_back("highest_osr");

	SoapySDR::ArgInfo trx_rate_governor;
	trx_rate_governor.key = "trx_rate_governor";
	trx_rate_governor.value = "nominal";
	trx_rate_governor.name = "TRX Rate Governor";
	trx_rate_governor.options = trx_rate_governor_available;
	trx_rate_governor.type = SoapySDR::ArgInfo::STRING;
	setArgs.push_back(trx_rate_governor);

	SoapySDR::Range xo_correction_available(39991977, 40007975, 1);

	SoapySDR::ArgInfo xo_correction;
	xo_correction.key = "xo_correction";
	xo_correction.value = "40000000";
	xo_correction.name = "XO Correction";
	xo_correction.range = xo_correction_available;
	xo_correction.type = SoapySDR::ArgInfo::STRING;
	setArgs.push_back(xo_correction);

	return setArgs;
}

void SoapyPlutoSDR::writeSetting(const std::string &key, const std::string &value)
{
	//std::lock_guard<pluto_spin_mutex> rx_lock(rx_device_mutex);
	//std::lock_guard<pluto_spin_mutex> tx_lock(tx_device_mutex);
	iio_device_attr_write(dev, key.c_str(), value.c_str());
}


std::string SoapyPlutoSDR::readSetting(const std::string &key) const
{
	std::string info;
	char value[128];
	ssize_t len;

	{
		//std::lock_guard<pluto_spin_mutex> rx_lock(rx_device_mutex);
		//std::lock_guard<pluto_spin_mutex> tx_lock(tx_device_mutex);
		len = iio_device_attr_read(dev, key.c_str(), value, sizeof(value));
	}

	if (len > 0)
	{
		info.assign(value, len);
	}
	else
	{
		iio_strerror(len, value, sizeof(value));
		info.assign(value);
	}
	return info;
}

SoapySDR::ArgInfoList SoapyPlutoSDR::getSettingInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList setArgs;
	/*
	voltage2:  (output) 8 channel-specific attributes found:
		0: filter_fir_en '0'
		1: raw '306'
		2: rf_bandwidth '18000000'
			3: rf_bandwidth_available '[200000 1 40000000]'
			4: rf_port_select_available 'A B'
		5: sampling_frequency '30720000'
			6: sampling_frequency_available '[2083333 1 61440000]'
		7: scale '1.000000'
	voltage2:  (input)	13 channel-specific attributes found:
		 0: bb_dc_offset_tracking_en '1'
		 1: filter_fir_en '0'
		 	2: gain_control_mode_available 'manual fast_attack slow_attack hybrid'
		 3: offset '57'
		 4: quadrature_tracking_en '1'
		 5: raw '2004'
		 6: rf_bandwidth '18000000'
		 	7: rf_bandwidth_available '[200000 1 56000000]'
		 8: rf_dc_offset_tracking_en '1'
		 	9: rf_port_select_available 'A_BALANCED B_BALANCED C_BALANCED A_N A_P B_N B_P C_N C_P TX_MONITOR1 TX_MONITOR2 TX_MONITOR1_2'
		10: sampling_frequency '30720000'
			11: sampling_frequency_available '[2083333 1 61440000]'
		12: scale '0.305250'

	Whitelist only these?
	voltage2:  (output):
		0: filter_fir_en '0'
	voltage2:  (input):
		0: bb_dc_offset_tracking_en '1'
		1: filter_fir_en '0'
		4: quadrature_tracking_en '1'
		8: rf_dc_offset_tracking_en '1'
	*/

	iio_channel *chn = iio_device_find_channel(dev, "voltage2", (direction == SOAPY_SDR_TX));

	unsigned int attrs_count = iio_channel_get_attrs_count(chn);
	const char *attrs[attrs_count];
	for (unsigned int index = 0; index < attrs_count; ++index)
	{
		attrs[index] = iio_channel_get_attr(chn, index);
	}

	// TODO: convert "_available" [min step max] to Range(minimum, maximum, step)
	// TODO: convert "_available" 'bare words' to std::vector<std::string> options

	for (unsigned int index = 0; index < attrs_count; ++index)
	{
		const char *attr = attrs[index];
		size_t attr_len = strlen(attr);
		if ((attr_len < (sizeof("_available") - 1)
				|| strcmp(attr + attr_len - (sizeof("_available") - 1), "_available"))
				&& strcmp(attr, "input")
				&& strcmp(attr, "raw")
				&& strcmp(attr, "offset")
				&& strcmp(attr, "scale")
				&& strcmp(attr, "rf_bandwidth")
				&& strcmp(attr, "sampling_frequency"))
		{
			SoapySDR::ArgInfo arg;
			arg.key = attr;
			//arg.value = "true";
			arg.name = attr;
			//arg.description = attr;
			arg.type = SoapySDR::ArgInfo::BOOL;
			setArgs.push_back(arg);
		}
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
	else
	{
		iio_strerror(len, value, sizeof(value));
		info.assign(value);
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
	return(false);
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
  	long long freq = 0;

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
#ifdef HAS_AD9361_IIO
	int const fir = 4; // assume ad9361_set_bb_rate() will load x4 FIR if needed
#else
	int const fir = 1;
#endif

	// note: sample rates below 25e6/12 need x8 decimation/interpolation or x4 FIR to 25e6/48,
	// below 25e6/96 need x8 decimation/interpolation and x4 FIR, minimum is 25e6/384
	// if libad9361 is available it will load an approporiate FIR.
	if(direction==SOAPY_SDR_RX){
        std::lock_guard<pluto_spin_mutex> lock(rx_device_mutex);
		decimation = false;
		if (samplerate < (25e6 / (12 * fir))) {
			if (samplerate * 8 < (25e6 / 48)) {
				SoapySDR_logf(SOAPY_SDR_CRITICAL, "sample rate is not supported.");
			}
			else if (samplerate * 8 < (25e6 / 12)) {
				SoapySDR_logf(SOAPY_SDR_NOTICE, "sample rate needs a FIR setting loaded.");
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
		if (samplerate < (25e6 / (12 * fir))) {
			if (samplerate * 8 < (25e6 / 48)) {
				SoapySDR_logf(SOAPY_SDR_CRITICAL, "sample rate is not supported.");
			}
			else if (samplerate * 8 < (25e6 / 12)) {
				SoapySDR_logf(SOAPY_SDR_NOTICE, "sample rate needs a FIR setting loaded.");
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
	long long samplerate = 0;

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

SoapySDR::RangeList SoapyPlutoSDR::getSampleRateRange( const int direction, const size_t channel ) const
{
	SoapySDR::RangeList results;

	// note that there are some gaps and rounding errors since we get truncated values form IIO
	// e.g. 25e6/12 = 2083333.333 is read as 2083333 but written as 2083334
#ifdef HAS_AD9361_IIO
	// assume ad9361_set_bb_rate(), if available, will load x4 FIR as needed
	// below 25e6/96 needs x8 decimation/interpolation and x4 FIR, minimum is 25e6/384
	results.push_back(SoapySDR::Range(25e6 / 384, 61440000));
#else
	// sample rates below 25e6/12 need x8 decimation/interpolation (or x4 FIR to 25e6/48)
	results.push_back(SoapySDR::Range(25e6 / 96, 61440000));
#endif

	return results;
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
    long long bandwidth = 0;

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
