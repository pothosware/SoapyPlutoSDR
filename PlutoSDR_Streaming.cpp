#include "SoapyPlutoSDR.hpp"


std::vector<std::string> SoapyPlutoSDR::getStreamFormats(const int direction, const size_t channel) const
{

	std::vector<std::string> formats;

	formats.push_back(SOAPY_SDR_CS8);
	formats.push_back(SOAPY_SDR_CS16);
	formats.push_back(SOAPY_SDR_CF32);
	formats.push_back(SOAPY_SDR_CF64);

	return formats;

}

std::string SoapyPlutoSDR::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
	fullScale = 2048;
	return SOAPY_SDR_CS12;
}

SoapySDR::ArgInfoList SoapyPlutoSDR::getStreamArgsInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList streamArgs;

	return streamArgs;
}

SoapySDR::Stream *SoapyPlutoSDR::setupStream(
		const int direction,
		const std::string &format,
		const std::vector<size_t> &channels,
		const SoapySDR::Kwargs &args )
{
	if(direction ==SOAPY_SDR_RX){

		rx_stream.i=iio_device_find_channel(rx_stream.dev,"voltage0",false);
		if(!rx_stream.i)
			rx_stream.i=iio_device_find_channel(rx_stream.dev,"altvoltage0",false);

		rx_stream.q=iio_device_find_channel(rx_stream.dev,"voltage1",false);
		if(!rx_stream.q)
			rx_stream.q=iio_device_find_channel(rx_stream.dev,"altvoltage1",false);


		iio_channel_enable(rx_stream.i);
		iio_channel_enable(rx_stream.q);

		return (SoapySDR::Stream*)&rx_stream;

	}


	if(direction ==SOAPY_SDR_TX){

		tx_stream.i=iio_device_find_channel(tx_stream.dev,"voltage0",true);
		if(!tx_stream.i)
			tx_stream.i=iio_device_find_channel(tx_stream.dev,"altvoltage0",true);

		rx_stream.q=iio_device_find_channel(tx_stream.dev,"voltage1",true);
		if(!tx_stream.q)
			tx_stream.q=iio_device_find_channel(tx_stream.dev,"altvoltage1",true);


		iio_channel_enable(tx_stream.i);
		iio_channel_enable(tx_stream.q);

		return (SoapySDR::Stream*)&tx_stream;

	}


	return nullptr;
}

void SoapyPlutoSDR::closeStream( SoapySDR::Stream *stream )
{
	stream_cfg* _stream=(stream_cfg*)stream;
	if(_stream->activate)
		this->deactivateStream(stream,0,0);

	if(_stream->i)iio_channel_disable(_stream->i);
	if(_stream->q)iio_channel_disable(_stream->q);	

}

size_t SoapyPlutoSDR::getStreamMTU( SoapySDR::Stream *stream ) const
{
	return 4096;

}

int SoapyPlutoSDR::activateStream(
		SoapySDR::Stream *stream,
		const int flags,
		const long long timeNs,
		const size_t numElems )
{
	stream_cfg* _stream=(stream_cfg*)stream;
	if(_stream->activate)
		this->deactivateStream(stream,0,0);

	_stream->buffer = iio_device_create_buffer(_stream->dev,numElems*sizeof(int16_t),false);	
	if(!_stream->buffer)
		_stream->activate=true;
	return 0;
}

int SoapyPlutoSDR::deactivateStream(
		SoapySDR::Stream *stream,
		const int flags,
		const long long timeNs )
{
	stream_cfg* _stream=(stream_cfg*)stream;
	if(_stream->activate){

		if(_stream->buffer){
			iio_buffer_destroy(_stream->buffer);
			_stream->buffer=nullptr;
		}
	}

	return 0;
}

int SoapyPlutoSDR::readStream(
		SoapySDR::Stream *stream,
		void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs )
{
	return numElems;
}

int SoapyPlutoSDR::writeStream(
		SoapySDR::Stream *stream,
		const void * const *buffs,
		const size_t numElems,
		int &flags,
		const long long timeNs,
		const long timeoutUs )
{
	return numElems;

}

int SoapyPlutoSDR::readStreamStatus(
		SoapySDR::Stream *stream,
		size_t &chanMask,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{
	return 0;
}

int SoapyPlutoSDR::acquireReadBuffer(
		SoapySDR::Stream *stream,
		size_t &handle,
		const void **buffs,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{

	return this->getStreamMTU(stream);
}


void SoapyPlutoSDR::releaseReadBuffer(
		SoapySDR::Stream *stream,
		const size_t handle)
{


}


int SoapyPlutoSDR::acquireWriteBuffer(
		SoapySDR::Stream *stream,
		size_t &handle,
		void **buffs,
		const long timeoutUs)
{

	return this->getStreamMTU(stream);
}

void SoapyPlutoSDR::releaseWriteBuffer(
		SoapySDR::Stream *stream,
		const size_t handle,
		const size_t numElems,
		int &flags,
		const long long timeNs)
{

}

size_t SoapyPlutoSDR::getNumDirectAccessBuffers(
		SoapySDR::Stream *stream)
{

	return 1;
}

int SoapyPlutoSDR::getDirectAccessBufferAddrs(
		SoapySDR::Stream *stream,
		const size_t handle,
		void **buffs)
{

	return 0;

}
