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

		stream->rx = std::shared_ptr<rx_streamer>(new rx_streamer(rx_dev, format,channels, args));
		rx_stream = stream->rx;
	}

	if (direction == SOAPY_SDR_TX) {

		stream->tx = std::shared_ptr<tx_streamer>(new tx_streamer(tx_dev, format,channels, args));
	}

	return reinterpret_cast<SoapySDR::Stream *>(stream);

}

void SoapyPlutoSDR::closeStream( SoapySDR::Stream *handle)
{
	std::lock_guard<std::mutex> lock(device_mutex);
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);
	if (stream->rx)
		rx_stream.reset();
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

void rx_streamer::set_buffer_size_by_samplerate(const size_t _samplerate) {

	uint32_t n = 1, x = uint32_t(_samplerate);
	if ((x >> 16) == 0) { n = n + 16; x = x << 16; }
	if ((x >> 24) == 0) { n = n + 8; x = x << 8; }
	if ((x >> 28) == 0) { n = n + 4; x = x << 4; }
	if ((x >> 30) == 0) { n = n + 2; x = x << 2; }
	n = n - (x >> 31);

	this->set_buffer_size(std::max(1 << (31 - n - 2), 16384));

	SoapySDR_logf(SOAPY_SDR_INFO, "Auto setting Buffer Size: %d", buffer_size);
}

rx_streamer::rx_streamer(const iio_device *_dev, const std::string &_format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args):
	dev(_dev),buffer_size(16384),buf(nullptr)

{
	if (dev == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "cf-ad9361-lpc not found!");
		throw std::runtime_error("cf-ad9361-lpc not found!");
	}
	unsigned int nb_channels = iio_device_get_channels_count(dev), i;
	for (i = 0; i < nb_channels; i++)
		iio_channel_disable(iio_device_get_channel(dev, i));

	//default to channel 0, if none were specified
	const std::vector<size_t> &channelIDs = channels.empty() ? std::vector<size_t>{0} : channels;

	for (i = 0; i < channelIDs.size() * 2; i++) {
		struct iio_channel *chn = iio_device_get_channel(dev, i);
		iio_channel_enable(chn);
		channel_list.push_back(chn);
	}

	if ( args.count( "bufflen" ) != 0 ){

		try
		{
			size_t bufferLength = std::stoi(args.at("bufflen"));
			if (bufferLength > 0)
				this->set_buffer_size(bufferLength);
		}
		catch (const std::invalid_argument &){}

	}else{

		long long samplerate;
		
		iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),"sampling_frequency",&samplerate);
		
		this->set_buffer_size_by_samplerate(samplerate);
	
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

	if (buf && this->buffer_size != _buffer_size) {
		iio_buffer_destroy(buf);

		buf = iio_device_create_buffer(dev, _buffer_size, false);
		if (!buf) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
			throw std::runtime_error("Unable to create buffer!\n");
		}
			
	}

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


tx_streamer::tx_streamer(const iio_device *_dev, const std::string &_format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args) :
	dev(_dev), buf(nullptr)
{

	if (dev == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "cf-ad9361-dds-core-lpc not found!");
		throw std::runtime_error("cf-ad9361-dds-core-lpc not found!");
	}

	unsigned int nb_channels = iio_device_get_channels_count(dev), i;
	for (i = 0; i < nb_channels; i++)
		iio_channel_disable(iio_device_get_channel(dev, i));

	//default to channel 0, if none were specified
	const std::vector<size_t> &channelIDs = channels.empty() ? std::vector<size_t>{0} : channels;

	for (i = 0; i < channelIDs.size() * 2; i++) {
		iio_channel *chn = iio_device_get_channel(dev, i);
		iio_channel_enable(chn);
		channel_list.push_back(chn);
	}

	buf = iio_device_create_buffer(dev, 4096, false);
	if (!buf) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
		throw std::runtime_error("Unable to create buffer!");
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
	size_t items = std::min(4096, (int)numElems);

	if (buffer.size() != items) {
		buffer.reserve(items);
		buffer.resize(items);
	}

	for (unsigned int i = 0; i < channel_list.size(); i++) {
		unsigned int index = i / 2;
		if(format==SOAPY_SDR_CS16){

			int16_t *samples_cs16 = (int16_t *)buffs[index];
			for (size_t j = 0; j < items; ++j) {
				buffer[j]=samples_cs16[j*2+i];
			}

			channel_write(channel_list[i],buffer.data(), items * sizeof(int16_t));

		}else if(format==SOAPY_SDR_CF32){

			float *samples_cf32 = (float *)buffs[index];
			for (size_t j = 0; j < items; ++j) {
				buffer[j]=(int16_t)(samples_cf32[j*2+i]*2048);
			}
			channel_write(channel_list[i],buffer.data(), items * sizeof(int16_t));

		}
		else if (format == SOAPY_SDR_CS8) {

			int8_t *samples_cs8 = (int8_t *)buffs[index];
			for (size_t j = 0; j < items; ++j) {
				buffer[j] = (int16_t)(samples_cs8[j*2 + i] <<4);
			}
			channel_write(channel_list[i], buffer.data(), items * sizeof(int16_t));
		}

	}

	ssize_t ret = iio_buffer_push(buf);

	if (ret < 0){

		return SOAPY_SDR_ERROR;

	}

	return int(ret / iio_buffer_step(buf));

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
