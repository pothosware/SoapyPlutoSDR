#include "SoapyPlutoSDR.hpp"
#include <SoapySDR/Registry.hpp>

static std::vector<SoapySDR::Kwargs> find_PlutoSDR(const SoapySDR::Kwargs &args){

	std::vector<SoapySDR::Kwargs> results;

	unsigned int ret;	
	iio_context *ctx;
	iio_scan_context *scan_ctx;
	iio_context_info **info;

	scan_ctx = iio_create_scan_context(NULL, 0);

	ret = iio_scan_context_get_info_list(scan_ctx, &info);

	if(ret == 0){

		ctx=iio_create_network_context(PLUTOSDR_DEFAULT_IP);
		if(ctx == NULL)
			ctx=iio_create_network_context(PLUTOSDR_DEFAULT_HOSTNAME);
		if(ctx == NULL)
			return results;
	}else if (ret == 1){

		ctx = iio_create_context_from_uri(iio_context_info_get_uri(info[0]));

	}else{

		//Multiple contexts found

	}

	SoapySDR::Kwargs options;
	unsigned int nb_ctx_attrs = iio_context_get_attrs_count(ctx);
	for (unsigned int i = 0; i < nb_ctx_attrs; i++){
		const char *key, *value;
		iio_context_get_attr(ctx, i, &key, &value);
		options[key]=value;

	}

	results.push_back(options);	

	return results;
}

static SoapySDR::Device *make_PlutoSDR(const SoapySDR::Kwargs &args)
{
	return new SoapyPlutoSDR(args);
}

static SoapySDR::Registry register_plutosdr("plutosdr", &find_PlutoSDR, &make_PlutoSDR, SOAPY_SDR_ABI_VERSION);
