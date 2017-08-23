#include "SoapyPlutoSDR.hpp"


SoapyPlutoSDR::SoapyPlutoSDR( const SoapySDR::Kwargs &args ){




}

SoapyPlutoSDR::~SoapyPlutoSDR(void){




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
    options.push_back( "A_BALANCED" );
    return(options);
}

void SoapyPlutoSDR::setAntenna( const int direction, const size_t channel, const std::string &name )
{
    /* TODO delete this function or throw if name != RX... */
}


std::string SoapyPlutoSDR::getAntenna( const int direction, const size_t channel ) const
{
    return("A_BALANCED");
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
    /* enable AGC if the hardware supports it, or remove this function */
}

bool SoapyPlutoSDR::getGainMode( const int direction, const size_t channel ) const
{
    return(true);
}

void SoapyPlutoSDR::setGain( const int direction, const size_t channel, const double value )
{


}

void SoapyPlutoSDR::setGain( const int direction, const size_t channel, const std::string &name, const double value )
{


}

double SoapyPlutoSDR::getGain( const int direction, const size_t channel, const std::string &name ) const
{
    double gain;

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

}

double SoapyPlutoSDR::getFrequency( const int direction, const size_t channel, const std::string &name ) const
{
    double freq;
    return freq;

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


}

double SoapyPlutoSDR::getSampleRate( const int direction, const size_t channel ) const
{
    double samplerate;
    return samplerate;

}

std::vector<double> SoapyPlutoSDR::listSampleRates( const int direction, const size_t channel ) const
{
    std::vector<double> options;

    return(options);

}

void SoapyPlutoSDR::setBandwidth( const int direction, const size_t channel, const double bw )
{


}

double SoapyPlutoSDR::getBandwidth( const int direction, const size_t channel ) const
{
    double bandwidth;
    return bandwidth;

}

std::vector<double> SoapyPlutoSDR::listBandwidths( const int direction, const size_t channel ) const
{
    std::vector<double> options;

    return(options);

}
