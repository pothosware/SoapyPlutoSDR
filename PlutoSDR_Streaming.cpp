#include "SoapyPlutoSDR.hpp"
#include <memory>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <iterator> 
#include <algorithm> 

struct PlutoSDRStream
{
	std::shared_ptr<rx_streamer> rx;
	std::shared_ptr<tx_streamer> tx;
};

std::vector<std::string> SoapyPlutoSDR::getStreamFormats(const int direction, const size_t channel) const
{

	std::vector<std::string> formats;

	formats.push_back(SOAPY_SDR_CS8);
	formats.push_back(SOAPY_SDR_CS16);
	formats.push_back(SOAPY_SDR_CF32);

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
	std::lock_guard<std::mutex> lock(device_mutex);

	PlutoSDRStream *stream = new PlutoSDRStream();

	if(direction ==SOAPY_SDR_RX){	

		stream->rx = std::shared_ptr<rx_streamer>(new rx_streamer(ctx, format,channels, args));

	}

	if (direction == SOAPY_SDR_TX) {

		stream->tx = std::shared_ptr<tx_streamer>(new tx_streamer(ctx, format,channels, args));
	}

	return reinterpret_cast<SoapySDR::Stream *>(stream);

}

void SoapyPlutoSDR::closeStream( SoapySDR::Stream *handle)
{
	std::lock_guard<std::mutex> lock(device_mutex);
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);
	delete stream;

}

size_t SoapyPlutoSDR::getStreamMTU( SoapySDR::Stream *handle) const
{
	return 8196;
}

int SoapyPlutoSDR::activateStream(
		SoapySDR::Stream *handle,
		const int flags,
		const long long timeNs,
		const size_t numElems )
{
	std::lock_guard<std::mutex> lock(device_mutex);
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
	std::lock_guard<std::mutex> lock(device_mutex);
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
		SoapySDR::Stream *handle,
		const void * const *buffs,
		const size_t numElems,
		int &flags,
		const long long timeNs,
		const long timeoutUs )
{
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);

	return stream->tx->send(buffs,numElems,flags,timeNs,timeoutUs);;

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



rx_streamer::rx_streamer(const iio_context *ctx, const std::string &_format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args):
	dev(nullptr),buffer_size(16384),buf(nullptr)

{
	dev=iio_context_find_device(ctx, "cf-ad9361-lpc");
	if (dev == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "cf-ad9361-lpc not found!");
		throw std::runtime_error("cf-ad9361-lpc not found!");
	}
	unsigned int nb_channels = iio_device_get_channels_count(dev), i;
	for (i = 0; i < nb_channels; i++)
		iio_channel_disable(iio_device_get_channel(dev, i));

	if (channels.empty()) {

		for (i = 0; i < nb_channels; i++) {
			struct iio_channel *chn = iio_device_get_channel(dev, i);
			iio_channel_enable(chn);
			channel_list.push_back(chn);
		}

	}
	else {
		for (i = 0; i < channels.size()*2; i++) {
			struct iio_channel *chn = iio_device_get_channel(dev, i);

			iio_channel_enable(chn);
			channel_list.push_back(chn);
		}
	
	
	}

	buffer.reserve(buffer_size);
	buffer.resize(buffer_size);

	format = _format;

	if (format == SOAPY_SDR_CF32) {
		const float scale = 1.0f / 2048.0f;
		for (int i = 0; i < 4096; ++i)
		{
			lut[i] = (((i + 2048) % 4096) - 2048) * scale;
		}
	}
		
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

	std::unique_lock<std::mutex> lock(mutex);

	if (thread_stopped){

		return SOAPY_SDR_TIMEOUT;
	}

	if (!please_refill_buffer && !items_in_buffer) {
		please_refill_buffer = true;
		cond.notify_all();
	}

	while (please_refill_buffer) {
		cond2.wait_for(lock,std::chrono::milliseconds(timeoutUs));

		if (thread_stopped)
			return SOAPY_SDR_TIMEOUT;

	}

	size_t items = std::min(items_in_buffer,numElems);

	buffer.resize(items);
	for (unsigned int i = 0; i < channel_list.size(); i++) {
		unsigned int index = i / 2;

		channel_read(channel_list[i], buffer.data(), items * sizeof(int16_t));

		if (format == SOAPY_SDR_CS16) {
			int16_t *samples_cs16 = (int16_t *)buffs[index];
			for (size_t j = 0; j < items; ++j) {
				samples_cs16[j*2 + i]=buffer[j];
			}		
		}else if (format == SOAPY_SDR_CF32) {
			float *samples_cf32 = (float *)buffs[index];
			for (size_t j = 0; j < items; ++j) {
				samples_cf32[j * 2 + i] = lut[buffer[j] & 0x0FFF];
			}
		}
		else if (format == SOAPY_SDR_CS8) {
			int8_t *samples_cs8 = (int8_t *)buffs[index];
			for (size_t j = 0; j < items; ++j) {
				samples_cs8[j * 2 + i] = buffer[j] >> 4;
			}
		}
		
	}


	items_in_buffer -= items;
	byte_offset += items * iio_buffer_step(buf);


	return(items);

}

int rx_streamer::start(const int flags,
		const long long timeNs,
		const size_t numElems)
{
	std::unique_lock<std::mutex> lock(mutex);

	items_in_buffer = 0;
	please_refill_buffer = false;
	thread_stopped = false;

	buf = iio_device_create_buffer(dev, buffer_size, false);

	if (buf) {
		refill_thd = std::thread(&rx_streamer::refill_thread, this);
	} else {
		SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
		throw std::runtime_error("Unable to create buffer!\n");
	}

	return 0;

}

int rx_streamer::stop(const int flags,
		const long long timeNs)
{
	if (buf)
		iio_buffer_cancel(buf);

	std::unique_lock<std::mutex> lock(mutex);

	please_refill_buffer = true;
	cond.notify_all();
	lock.unlock();

	refill_thd.join();

	if (buf) {
		iio_buffer_destroy(buf);
		buf = nullptr;
	}

	return 0;


}

void rx_streamer::set_buffer_size(const size_t _buffer_size){

	std::unique_lock<std::mutex> lock(mutex);

	this->buffer_size=_buffer_size;

}

void rx_streamer::refill_thread(){

	std::unique_lock<std::mutex> lock(mutex);
	ssize_t ret;

	for (;;) {

		while (!please_refill_buffer)
			cond.wait(lock);

		please_refill_buffer = false;

		lock.unlock();
		ret = iio_buffer_refill(buf);
		lock.lock();
		if (ret < 0)
			break;

		items_in_buffer = (unsigned long)ret / iio_buffer_step(buf);
		byte_offset = 0;

		cond2.notify_one();

	}
	thread_stopped = true;
	cond2.notify_all();
}


void rx_streamer::channel_read(const struct iio_channel *chn, void *dst, size_t len) {

	uintptr_t src_ptr, dst_ptr = (uintptr_t)dst, end = dst_ptr + len;
	unsigned int length = iio_channel_get_data_format(chn)->length / 8;
	uintptr_t buf_end = (uintptr_t)iio_buffer_end(buf);
	ptrdiff_t buf_step = iio_buffer_step(buf);

	for (src_ptr = (uintptr_t)iio_buffer_first(buf, chn)+ byte_offset;
			src_ptr < buf_end && dst_ptr + length <= end;
			src_ptr += buf_step, dst_ptr += length)
		iio_channel_convert(chn,
				(void *)dst_ptr, (const void *)src_ptr);
}


tx_streamer::tx_streamer(const iio_context *ctx, const std::string &_format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args) :
	dev(nullptr), buf(nullptr)
{
	dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
	if (dev == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "cf-ad9361-dds-core-lpc not found!");
		throw std::runtime_error("cf-ad9361-dds-core-lpc not found!");
	}

	unsigned int nb_channels = iio_device_get_channels_count(dev), i;
	for (i = 0; i < nb_channels; i++)
		iio_channel_disable(iio_device_get_channel(dev, i));

	if (channels.empty()) {

		for (i = 0; i < nb_channels; i++) {
			iio_channel *chn = iio_device_get_channel(dev, i);
			iio_channel_enable(chn);
			channel_list.push_back(chn);
		}

	}
	else {
		for (i = 0; i < channels.size() * 2; i++) {
			iio_channel *chn = iio_device_get_channel(dev, i);
			iio_channel_enable(chn);
			channel_list.push_back(chn);
		}

	}
	format = _format;
}

tx_streamer::~tx_streamer(){

	if (buf) { iio_buffer_destroy(buf); }

	for(unsigned int i=0;i<channel_list.size(); ++i)
		iio_channel_disable(channel_list[i]);

}

int tx_streamer::send(	const void * const *buffs,
		const size_t numElems,
		int &flags,
		const long long timeNs,
		const long timeoutUs )

{
	std::lock_guard<std::mutex> lock(mutex);

	if (buffer.size() != numElems) {
		if (buf) 
			iio_buffer_destroy(buf);

		buffer.reserve(numElems);
		buffer.resize(numElems);
		buf = iio_device_create_buffer(dev, buffer.size(), false);
		if (!buf) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
			throw std::runtime_error("Unable to create buffer!");
		}

	}

	for (unsigned int i = 0; i < channel_list.size(); i++) {
		unsigned int index = i / 2;
		if(format==SOAPY_SDR_CS16){

			int16_t *samples_cs16 = (int16_t *)buffs[index];
			for (size_t j = 0; j < numElems; ++j) {
				buffer[j]=samples_cs16[j*2+i];
			}

			channel_write(channel_list[i],buffer.data(), numElems * sizeof(int16_t));

		}else if(format==SOAPY_SDR_CF32){

			float *samples_cf32 = (float *)buffs[index];
			for (size_t j = 0; j < numElems; ++j) {
				buffer[j]=(int16_t)(samples_cf32[j*2+i]*2048);
			}
			channel_write(channel_list[i],buffer.data(), numElems * sizeof(int16_t));

		}
		else if (format == SOAPY_SDR_CS8) {

			int8_t *samples_cs8 = (int8_t *)buffs[index];
			for (size_t j = 0; j < numElems; ++j) {
				buffer[j] = (int16_t)(samples_cs8[j*2 + i] <<4);
			}
			channel_write(channel_list[i], buffer.data(), numElems * sizeof(int16_t));
		}

	}

	ssize_t ret = iio_buffer_push(buf);

	if (ret < 0){

		return SOAPY_SDR_ERROR;

	}

	return ret/ iio_buffer_step(buf);

}

void tx_streamer::channel_write(iio_channel *chn,const void *src, size_t len){

	uintptr_t dst_ptr, src_ptr = (uintptr_t) src, end = src_ptr + len;
	unsigned int length = iio_channel_get_data_format(chn)->length / 8;
	uintptr_t buf_end = (uintptr_t) iio_buffer_end(buf);
	ptrdiff_t buf_step = iio_buffer_step(buf);

	for (dst_ptr = (uintptr_t) iio_buffer_first(buf, chn);
			dst_ptr < buf_end && src_ptr + length <= end;
			dst_ptr += buf_step, src_ptr += length)
		iio_channel_convert_inverse(chn,(void *) dst_ptr, (const void *) src_ptr);
}
