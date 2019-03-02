#include "SoapyPlutoSDR.hpp"
#include <memory>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <iterator> 
#include <algorithm> 
#include <chrono>

# define RX_STREAM_MTU   (65536)


std::vector<std::string> SoapyPlutoSDR::getStreamFormats(const int direction, const size_t channel) const
{

	std::vector<std::string> formats;

	formats.push_back(SOAPY_SDR_CS8);
	formats.push_back(SOAPY_SDR_CS12);
	formats.push_back(SOAPY_SDR_CS16);
	formats.push_back(SOAPY_SDR_CF32);

	return formats;

}

std::string SoapyPlutoSDR::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
	if (direction == SOAPY_SDR_RX) {
		fullScale = 2048; // RX expects 12 bit samples LSB aligned
	}
	else if (direction == SOAPY_SDR_TX) {
		fullScale = 32768; // TX expects 12 bit samples MSB aligned
	}
	return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList SoapyPlutoSDR::getStreamArgsInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList streamArgs;

	return streamArgs;
}


bool SoapyPlutoSDR::IsValidRxStreamHandle(SoapySDR::Stream* handle)
{
    if (handle == nullptr) {
        return false;
    }

    //handle is an opaque pointer hiding either rx_stream or tx_streamer:
    //check that the handle matches one of them, onsistently with direction:
    if (rx_stream) {
        //test if these handles really belong to us:
        if (reinterpret_cast<rx_streamer*>(handle) == rx_stream.get()) {
            return true;
        } 
    } 
 
    return false;
}

bool SoapyPlutoSDR::IsValidTxStreamHandle(SoapySDR::Stream* handle)
{
    if (handle == nullptr) {
        return false;
    }

    //handle is an opaque pointer hiding either rx_stream or tx_streamer:
    //check that the handle matches one of them, onsistently with direction:
    if (tx_stream) {
        //test if these handles really belong to us:
        if (reinterpret_cast<tx_streamer*>(handle) == tx_stream.get()) {
            return true;
        }
    }

    return false;
}

SoapySDR::Stream *SoapyPlutoSDR::setupStream(
		const int direction,
		const std::string &format,
		const std::vector<size_t> &channels,
		const SoapySDR::Kwargs &args )
{
	//check the format
	plutosdrStreamFormat streamFormat;
	if (format == SOAPY_SDR_CF32) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
		streamFormat = PLUTO_SDR_CF32;
	}
	else if (format == SOAPY_SDR_CS16) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
		streamFormat = PLUTO_SDR_CS16;
	}
	else if (format == SOAPY_SDR_CS12) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CS12.");
		streamFormat = PLUTO_SDR_CS12;
	}
	else if (format == SOAPY_SDR_CS8) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CS8.");
		streamFormat = PLUTO_SDR_CS8;
	}
	else {
		throw std::runtime_error(
			"setupStream invalid format '" + format + "' -- Only CS8, CS12, CS16 and CF32 are supported by SoapyPlutoSDR module.");
	}

    std::lock_guard<std::mutex> lock(device_mutex);

	if(direction == SOAPY_SDR_RX){	

        this->rx_stream = std::make_unique<rx_streamer>(rx_dev, streamFormat, channels, args);

        return reinterpret_cast<SoapySDR::Stream*>(this->rx_stream.get());
	}

	if (direction == SOAPY_SDR_TX) {

        this->tx_stream = std::make_unique<tx_streamer>(tx_dev, streamFormat, channels, args);

        return reinterpret_cast<SoapySDR::Stream*>(this->tx_stream.get());
	}

	return nullptr;

}

void SoapyPlutoSDR::closeStream( SoapySDR::Stream *handle)
{
	std::lock_guard<std::mutex> lock(device_mutex);

    if (IsValidRxStreamHandle(handle)) {
        this->rx_stream.reset();
    } else if (IsValidTxStreamHandle(handle)) {
       this->tx_stream.reset();
    }
}

size_t SoapyPlutoSDR::getStreamMTU( SoapySDR::Stream *handle) const
{
	return RX_STREAM_MTU;
}

int SoapyPlutoSDR::activateStream(
		SoapySDR::Stream *handle,
		const int flags,
		const long long timeNs,
		const size_t numElems )
{	
	if (flags & ~SOAPY_SDR_END_BURST)
		return SOAPY_SDR_NOT_SUPPORTED;

    std::lock_guard<std::mutex> lock(device_mutex);

    if (IsValidRxStreamHandle(handle)) {
        return this->rx_stream->start(flags, timeNs, numElems);
    }

    return 0;
}

int SoapyPlutoSDR::deactivateStream(
		SoapySDR::Stream *handle,
		const int flags,
		const long long timeNs )
{
    std::lock_guard<std::mutex> lock(device_mutex);

    if (IsValidRxStreamHandle(handle)) {
        return this->rx_stream->stop(flags, timeNs);
    } else if (IsValidTxStreamHandle(handle)) {
        this->tx_stream->flush();
        return 0;
    }

	return 0;
}

int SoapyPlutoSDR::readStream(
		SoapySDR::Stream *handle,
		void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs )
{

    if (IsValidRxStreamHandle(handle)) {
        return int(this->rx_stream->recv(buffs, numElems, flags, timeNs, timeoutUs));
    } else {
        return SOAPY_SDR_NOT_SUPPORTED;
    }
}

int SoapyPlutoSDR::writeStream(
		SoapySDR::Stream *handle,
		const void * const *buffs,
		const size_t numElems,
		int &flags,
		const long long timeNs,
		const long timeoutUs )
{

    if (IsValidTxStreamHandle(handle)) {
        return this->tx_stream->send(buffs, numElems, flags, timeNs, timeoutUs);;
    } else {
        return SOAPY_SDR_NOT_SUPPORTED;
    }
  
}

int SoapyPlutoSDR::readStreamStatus(
		SoapySDR::Stream *stream,
		size_t &chanMask,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{
	return SOAPY_SDR_NOT_SUPPORTED;
}

void rx_streamer::set_buffer_size_by_samplerate(const size_t _samplerate) {

	uint32_t n = 1, x = uint32_t(_samplerate);
	if ((x >> 16) == 0) { n = n + 16; x = x << 16; }
	if ((x >> 24) == 0) { n = n + 8; x = x << 8; }
	if ((x >> 28) == 0) { n = n + 4; x = x << 4; }
	if ((x >> 30) == 0) { n = n + 2; x = x << 2; }
	n = n - (x >> 31);

	//this->set_buffer_size(std::max(1 << (31 - n - 2), 16384)); // too large for CubicSDR
    //TODO: find smarter way w.r.t MTU and sample rate ?
	this->set_buffer_size((size_t)(RX_STREAM_MTU));

	SoapySDR_logf(SOAPY_SDR_INFO, "Auto setting Buffer Size: %lu", (unsigned long)buffer_size);
}

rx_streamer::rx_streamer(const iio_device *_dev, const plutosdrStreamFormat _format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args):
	dev(_dev), buffer_size(16384), buf(nullptr), format(_format)

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
}

rx_streamer::~rx_streamer() 
{
	if (buf) {
        iio_buffer_cancel(buf);
        iio_buffer_destroy(buf); 
    }

	for(unsigned int i=0;i<channel_list.size(); ++i)
		iio_channel_disable(channel_list[i]);

}

size_t rx_streamer::recv(void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{
    //
	if (items_in_buffer <= 0) {

       // auto before = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

	    if (!buf) {
		    return 0;
	    }

		ssize_t ret = iio_buffer_refill(buf);

        // auto after = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

		if (ret < 0)
			return SOAPY_SDR_TIMEOUT;

		items_in_buffer = (unsigned long)ret / iio_buffer_step(buf);

        // SoapySDR_logf(SOAPY_SDR_INFO, "iio_buffer_refill took %d ms to refill %d items", (int)(after - before), items_in_buffer);

		byte_offset = 0;
	}

	size_t items = std::min(items_in_buffer,numElems);

	ptrdiff_t buf_step = iio_buffer_step(buf);

	if (direct_copy) {
		// optimize for single RX, 2 channel (I/Q), same endianess direct copy
		// note that RX is 12 bits LSB aligned, i.e. fullscale 2048
		uint8_t *src = (uint8_t *)iio_buffer_start(buf) + byte_offset;
		int16_t const *src_ptr = (int16_t *)src;

		if (format == PLUTO_SDR_CS16) {

			::memcpy(buffs[0], src_ptr, 2 * sizeof(int16_t) * items);

		}
		else if (format == PLUTO_SDR_CF32) {

			float *dst_cf32 = (float *)buffs[0];

			for (size_t index = 0; index < items * 2; ++index) {
				*dst_cf32 = float(*src_ptr) / 2048.0f;
				src_ptr++;
				dst_cf32++;
			}

		}
		else if (format == PLUTO_SDR_CS12) {

			int8_t *dst_cs12 = (int8_t *)buffs[0];

			for (size_t index = 0; index < items; ++index) {
				int16_t i = *src_ptr++;
				int16_t q = *src_ptr++;
				// produce 24 bit (iiqIQQ), note the input is LSB aligned, scale=2048
				// note: byte0 = i[7:0]; byte1 = {q[3:0], i[11:8]}; byte2 = q[11:4];
				*dst_cs12++ = uint8_t(i);
				*dst_cs12++ = uint8_t((q << 4) | ((i >> 8) & 0x0f));
				*dst_cs12++ = uint8_t(q >> 4);
			}
		}
		else if (format == PLUTO_SDR_CS8) {

			int8_t *dst_cs8 = (int8_t *)buffs[0];

			for (size_t index = 0; index < items * 2; index++) {
				*dst_cs8 = int8_t(*src_ptr >> 4);
				src_ptr++;
				dst_cs8++;
			}
		}
	}
	else {
		int16_t conv = 0, *conv_ptr = &conv;

		for (unsigned int i = 0; i < channel_list.size(); i++) {
			iio_channel *chn = channel_list[i];
			unsigned int index = i / 2;

			uint8_t *src = (uint8_t *)iio_buffer_first(buf, chn) + byte_offset;
			int16_t const *src_ptr = (int16_t *)src;

			if (format == PLUTO_SDR_CS16) {

				int16_t *dst_cs16 = (int16_t *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					iio_channel_convert(chn, conv_ptr, src_ptr);
					src_ptr += buf_step;
					dst_cs16[j * 2 + i] = conv;
				}
			}
			else if (format == PLUTO_SDR_CF32) {

				float *dst_cf32 = (float *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					iio_channel_convert(chn, conv_ptr, src_ptr);
					src_ptr += buf_step;
					dst_cf32[j * 2 + i] = float(conv) / 2048.0f;
				}
			}
			else if (format == PLUTO_SDR_CS8) {

				int8_t *dst_cs8 = (int8_t *)buffs[index];

				for (size_t j = 0; j < items; ++j) {
					iio_channel_convert(chn, conv_ptr, src_ptr);
					src_ptr += buf_step;
					dst_cs8[j * 2 + i] = int8_t(conv >> 4);
				}
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
    //force proper stop before
    stop(flags, timeNs);

	items_in_buffer = 0;
	
    // re-create buffer 
	buf = iio_device_create_buffer(dev, buffer_size, false);

	if (!buf) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
		throw std::runtime_error("Unable to create buffer!\n");
	}

	direct_copy = has_direct_copy();

	SoapySDR_logf(SOAPY_SDR_INFO, "Has direct RX copy: %d", (int)direct_copy);

	return 0;

}

int rx_streamer::stop(const int flags,
		const long long timeNs)
{
    //cancel first
    if (buf) {
        iio_buffer_cancel(buf);
    }
    //then destroy
	if (buf) {
		iio_buffer_destroy(buf);
		buf = nullptr;
	}

	return 0;

}

void rx_streamer::set_buffer_size(const size_t _buffer_size){

	if (!buf || this->buffer_size != _buffer_size) {
		if (buf) {
			iio_buffer_destroy(buf);
		}

		items_in_buffer = 0;
		buf = iio_device_create_buffer(dev, _buffer_size, false);
		if (!buf) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
			throw std::runtime_error("Unable to create buffer!\n");
		}
			
	}

	this->buffer_size=_buffer_size;

}

// return wether can we optimize for single RX, 2 channel (I/Q), same endianess direct copy
bool rx_streamer::has_direct_copy()
{
	if (channel_list.size() != 2) // one RX with I + Q
		return false;

	ptrdiff_t buf_step = iio_buffer_step(buf);

	if (buf_step != 2 * sizeof(int16_t))
		return false;

	if (iio_buffer_start(buf) != iio_buffer_first(buf, channel_list[0]))
		return false;

	int16_t test_dst, test_src = 0x1234;
	iio_channel_convert(channel_list[0], &test_dst, (const void *)&test_src);

	return test_src == test_dst;

}


tx_streamer::tx_streamer(const iio_device *_dev, const plutosdrStreamFormat _format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args) :
	dev(_dev), format(_format), buf(nullptr)
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

	buf_size = 4096;
	items_in_buf = 0;
	buf = iio_device_create_buffer(dev, buf_size, false);
	if (!buf) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
		throw std::runtime_error("Unable to create buffer!");
	}

	direct_copy = has_direct_copy();

	SoapySDR_logf(SOAPY_SDR_INFO, "Has direct TX copy: %d", (int)direct_copy);

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

	size_t items = std::min(buf_size - items_in_buf, numElems);

	int16_t src = 0;
	int16_t const *src_ptr = &src;
	uint8_t *dst_ptr;
	ptrdiff_t buf_step = iio_buffer_step(buf);

	if (direct_copy && format == PLUTO_SDR_CS16) {
		// optimize for single TX, 2 channel (I/Q), same endianess direct copy
		dst_ptr = (uint8_t *)iio_buffer_start(buf) + items_in_buf * 2 * sizeof(int16_t);

		memcpy(dst_ptr, buffs[0], 2 * sizeof(int16_t) * items);
	}
	else if (direct_copy && format == PLUTO_SDR_CS12) {

		dst_ptr = (uint8_t *)iio_buffer_start(buf) + items_in_buf * 2 * sizeof(int16_t);
		int8_t *samples_cs12 = (int8_t *)buffs[0];

		for (size_t index = 0; index < items; ++index) {
			// consume 24 bit (iiqIQQ)
			uint16_t src0 = uint16_t(*(samples_cs12++));
			uint16_t src1 = uint16_t(*(samples_cs12++));
			uint16_t src2 = uint16_t(*(samples_cs12++));
			// produce 2x 16 bit, note the output is MSB aligned, scale=32768
			// note: byte0 = i[11:4]; byte1 = {q[7:4], i[15:12]}; byte2 = q[15:8];
			*dst_ptr = int16_t((src1 << 12) | (src0 << 4));
			dst_ptr++;
			*dst_ptr = int16_t((src2 << 8) | (src1 & 0xf0));
			dst_ptr++;
		}
	}
	else if (format == PLUTO_SDR_CS12) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "CS12 not available with this endianess or channel layout");
		throw std::runtime_error("CS12 not available with this endianess or channel layout");
	}
	else

	for (unsigned int k = 0; k < channel_list.size(); k++) {
		iio_channel *chn = channel_list[k];
		unsigned int index = k / 2;

		dst_ptr = (uint8_t *)iio_buffer_first(buf, chn) + items_in_buf * buf_step;

		// note that TX expects samples MSB aligned, unlike RX which is LSB aligned
		if (format == PLUTO_SDR_CS16) {

			int16_t *samples_cs16 = (int16_t *)buffs[index];

			for (size_t j = 0; j < items; ++j) {
				src = samples_cs16[j*2+k];
				iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
				dst_ptr += buf_step;
			}
		}
		else if (format == PLUTO_SDR_CF32) {

			float *samples_cf32 = (float *)buffs[index];

			for (size_t j = 0; j < items; ++j) {
				src = (int16_t)(samples_cf32[j*2+k] * 32767.999f); // 32767.999f (0x46ffffff) will ensure better distribution
				iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
				dst_ptr += buf_step;
			}
		}
		else if (format == PLUTO_SDR_CS8) {

			int8_t *samples_cs8 = (int8_t *)buffs[index];

			for (size_t j = 0; j < items; ++j) {
				src = (int16_t)(samples_cs8[j*2+k] << 8);
				iio_channel_convert_inverse(chn, dst_ptr, src_ptr);
				dst_ptr += buf_step;
			}
		}
	}

	items_in_buf += items;

	if (items_in_buf == buf_size || (flags & SOAPY_SDR_END_BURST && numElems == items)) {
		int ret = send_buf();

		if (ret < 0) {
			return SOAPY_SDR_ERROR;
		}

		if ((size_t)ret != buf_size) {
			return SOAPY_SDR_ERROR;
		}
	}

	return items;

}

int tx_streamer::flush()
{
	std::lock_guard<std::mutex> lock(mutex);

	return send_buf();
}

int tx_streamer::send_buf()
{

	if (items_in_buf > 0) {
		if (items_in_buf < buf_size) {
			ptrdiff_t buf_step = iio_buffer_step(buf);
			uint8_t *buf_ptr = (uint8_t *)iio_buffer_start(buf) + items_in_buf * buf_step;
			uint8_t *buf_end = (uint8_t *)iio_buffer_end(buf);

			memset(buf_ptr, 0, buf_end - buf_ptr);
		}

		ssize_t ret = iio_buffer_push(buf);
		items_in_buf = 0;

		if (ret < 0) {
			return ret;
		}

		return int(ret / iio_buffer_step(buf));
	}

	return 0;

}

// return wether can we optimize for single TX, 2 channel (I/Q), same endianess direct copy
bool tx_streamer::has_direct_copy()
{

	if (channel_list.size() != 2) // one TX with I/Q
		return false;

	ptrdiff_t buf_step = iio_buffer_step(buf);

	if (buf_step != 2 * sizeof(int16_t))
		return false;

	if (iio_buffer_start(buf) != iio_buffer_first(buf, channel_list[0]))
		return false;

	int16_t test_dst, test_src = 0x1234;
	iio_channel_convert_inverse(channel_list[0], &test_dst, (const void *)&test_src);

	return test_src == test_dst;

}
