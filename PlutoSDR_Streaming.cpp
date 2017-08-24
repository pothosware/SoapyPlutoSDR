#include "SoapyPlutoSDR.hpp"
#include <memory>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <iterator> 

struct PlutoSDRStream
{
	std::shared_ptr<rx_streamer> rx;
	std::shared_ptr<tx_streamer> tx;
};

std::vector<std::string> SoapyPlutoSDR::getStreamFormats(const int direction, const size_t channel) const
{

	std::vector<std::string> formats;

	//	formats.push_back(SOAPY_SDR_CS8);
	formats.push_back(SOAPY_SDR_CS16);
	//	formats.push_back(SOAPY_SDR_CF32);
	//	formats.push_back(SOAPY_SDR_CF64);

	return formats;

}

std::string SoapyPlutoSDR::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
	fullScale = 2048;
	return SOAPY_SDR_CS16;
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
	PlutoSDRStream *stream = new PlutoSDRStream();

	if(direction ==SOAPY_SDR_RX){	

		stream->rx = std::shared_ptr<rx_streamer>(new rx_streamer(ctx, format,channels, args));

	}

	if (direction == SOAPY_SDR_TX) {

		stream->tx = std::shared_ptr<tx_streamer>(new tx_streamer());
	}

	return reinterpret_cast<SoapySDR::Stream *>(stream);

}

void SoapyPlutoSDR::closeStream( SoapySDR::Stream *handle)
{
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);
	delete stream;

}

size_t SoapyPlutoSDR::getStreamMTU( SoapySDR::Stream *handle) const
{
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);
	
	return 8196;
}

int SoapyPlutoSDR::activateStream(
		SoapySDR::Stream *handle,
		const int flags,
		const long long timeNs,
		const size_t numElems )
{
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);

	if (not stream->rx)
		return 0;

	return stream->rx->start(flags,timeNs,numElems);
}

int SoapyPlutoSDR::deactivateStream(
		SoapySDR::Stream *handle,
		const int flags,
		const long long timeNs )
{
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);

	if (not stream->rx)
		return 0;

	return stream->rx->stop(flags,timeNs);
}

int SoapyPlutoSDR::readStream(
		SoapySDR::Stream *handle,
		void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs )
{
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);

	return int(stream->rx->recv(buffs, numElems, flags, timeNs, timeoutUs));
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



rx_streamer::rx_streamer(const iio_context *ctx, const std::string &format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args):
	buf(nullptr),buffer_size(65535),dev(nullptr), buffer_num(15)

{
	dev=iio_context_find_device(ctx, "cf-ad9361-lpc");
	if(dev == NULL)
		throw std::runtime_error("rx dev not found.");

	channel_list.push_back(iio_device_find_channel(dev, "voltage0", false));
	channel_list.push_back(iio_device_find_channel(dev, "voltage1", false));

	for(unsigned int i=0;i<channel_list.size(); ++i){

		iio_channel_enable(channel_list[i]);
	}

	buffer.reserve(buffer_size);
	buffer.resize(buffer_size);

}

rx_streamer::~rx_streamer() 
{
	if (buf) { iio_buffer_destroy(buf); }

	for(unsigned int i=0;i<channel_list.size(); ++i)
		iio_channel_disable(channel_list[i]);

}

size_t rx_streamer::recv(void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{

	//std::lock_guard<std::mutex> lock(mutex);

	if (thread_stopped){

		return -1;
	}

	size_t returnedElems = std::min(numElems,buffer_size);

		std::unique_lock <std::mutex> lock(mutex);
		if (i_deque.size() < returnedElems)
		{
			{
				cond.wait_for(lock, std::chrono::microseconds(timeoutUs));
				if (i_deque.size() < returnedElems)
					return SOAPY_SDR_TIMEOUT;
			}
		}

		if (overflow) {
			overflow = false;
			SoapySDR::log(SOAPY_SDR_SSI, "O");
			return  SOAPY_SDR_OVERFLOW;
		}

	
	int16_t *samples_cs16 = (int16_t *)buffs[0];

		for (size_t i = 0; i < returnedElems ; ++i)
		{
			samples_cs16[i*2] = i_deque.front();
			i_deque.pop_front();
			samples_cs16[i * 2+1] = q_deque.front();
			q_deque.pop_front();

		}


	return(returnedElems);

}

int rx_streamer::start(const int flags,
		const long long timeNs,
		const size_t numElems)
{
	std::unique_lock<std::mutex> lock(mutex);

	buf = iio_device_create_buffer(dev, buffer_size, false);

	if (buf) {
		recv_thd = std::thread(&rx_streamer::recv_thread, this);
	} else {
		return -1;
	}

	return 0;

}

int rx_streamer::stop(const int flags,
		const long long timeNs)
{
	if (buf)
		iio_buffer_cancel(buf);

	std::unique_lock<std::mutex> lock(mutex);

	if(!thread_stopped){
		thread_stopped=true;
		recv_thd.join();
	}
	if (buf) {
		iio_buffer_destroy(buf);
		buf = NULL;
	}

	return 0;


}

void rx_streamer::set_buffer_size(const size_t _buffer_size){

	std::unique_lock<std::mutex> lock(mutex);

	this->buffer_size=_buffer_size;

}

void rx_streamer::recv_thread(){


	ssize_t ret;

	thread_stopped=false;	
	while(!thread_stopped) {


		ret = iio_buffer_refill(buf);

		if (ret < 0)
			break;

		
		if(i_deque.size()>=buffer_num*buffer_size){
			overflow=true;
			return ;
		}


		{
			std::lock_guard<std::mutex> lock(mutex);
			
			channel_read(channel_list[0], buffer.data(), buffer.size());
			std::copy(buffer.begin(), buffer.end(), std::back_inserter(i_deque));
		
			channel_read(channel_list[1], buffer.data(), buffer.size());
			std::copy(buffer.begin(), buffer.end(), std::back_inserter(q_deque));

			cond.notify_one();
		}
		
	}
	thread_stopped = true;

}


void rx_streamer::channel_read(const struct iio_channel *chn, void *dst, size_t len) {

	uintptr_t src_ptr, dst_ptr = (uintptr_t)dst, end = dst_ptr + len;
	unsigned int length = iio_channel_get_data_format(chn)->length / 8;
	uintptr_t buf_end = (uintptr_t)iio_buffer_end(buf);
	ptrdiff_t buf_step = iio_buffer_step(buf);

	for (src_ptr = (uintptr_t)iio_buffer_first(buf, chn);
		src_ptr < buf_end && dst_ptr + length <= end;
		src_ptr += buf_step, dst_ptr += length)
		iio_channel_convert(chn,
		(void *)dst_ptr, (const void *)src_ptr);
}
