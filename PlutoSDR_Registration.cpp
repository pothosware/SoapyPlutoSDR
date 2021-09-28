#include "SoapyPlutoSDR.hpp"
#include <SoapySDR/Registry.hpp>

static std::vector<SoapySDR::Kwargs> results;
static std::vector<SoapySDR::Kwargs> find_PlutoSDR(const SoapySDR::Kwargs &args) {

	if (!results.empty())
		return results;

	ssize_t ret = 0;
	iio_context *ctx = nullptr;
	iio_scan_context *scan_ctx;
	iio_context_info **info;
	SoapySDR::Kwargs options;

	// Backends can error, scan each one individually
	std::vector<std::string> backends = {"local", "usb", "ip"};
	for (std::vector<std::string>::iterator it = backends.begin(); it != backends.end(); it++) {

		scan_ctx = iio_create_scan_context(it->c_str(), 0);
		if (scan_ctx == nullptr) {
			SoapySDR_logf(SOAPY_SDR_WARNING, "Unable to setup %s scan\n", it->c_str());
			continue;
		}

		info = nullptr;
		ret = iio_scan_context_get_info_list(scan_ctx, &info);
		if (ret < 0) {
			SoapySDR_logf(SOAPY_SDR_WARNING, "Unable to scan %s: %li\n", it->c_str(), (long)ret);
			iio_context_info_list_free(info);
			iio_scan_context_destroy(scan_ctx);
			continue;
		}

		char label_str[100];
		options["device"] = "plutosdr";
		if (ret == 0) {
			iio_context_info_list_free(info);
			iio_scan_context_destroy(scan_ctx);

			//no devices discovered, the user must specify a hostname
			if (args.count("hostname") == 0) continue;

			//try to connect at the specified hostname
			ctx = iio_create_network_context(args.at("hostname").c_str());
			if (ctx == nullptr) continue; //failed to connect
			options["hostname"] = args.at("hostname");

			sprintf(label_str, "%s #%d %s", options["device"].c_str(), 0, options["hostname"].c_str());
			options["label"] = label_str;
			results.push_back(options);
			if (ctx != nullptr) iio_context_destroy(ctx);

		} else {
			for (int i = 0; i < ret; i++) {
				ctx = iio_create_context_from_uri(iio_context_info_get_uri(info[i]));
				if (ctx != nullptr) {
					options["uri"] = std::string(iio_context_info_get_uri(info[i]));
					sprintf(label_str, "%s #%d %s", options["device"].c_str(), i, options["uri"].c_str());
					results.push_back(options);
					if (ctx != nullptr) iio_context_destroy(ctx);
				}

			}
			iio_context_info_list_free(info);
			iio_scan_context_destroy(scan_ctx);
		}

	}
	return results;
}

static SoapySDR::Device *make_PlutoSDR(const SoapySDR::Kwargs &args)
{
	return new SoapyPlutoSDR(args);
}

static SoapySDR::Registry register_plutosdr("plutosdr", &find_PlutoSDR, &make_PlutoSDR, SOAPY_SDR_ABI_VERSION);
