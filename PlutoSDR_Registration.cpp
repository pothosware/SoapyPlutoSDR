#include "SoapyPlutoSDR.hpp"
#include <SoapySDR/Registry.hpp>
#include <sstream>
#include <chrono>
#include <thread>
#ifdef HAS_LIBUSB1
#include <libusb.h>
#endif

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
	// The filtered "usb" backend is available starting from Libiio 0.24
	std::vector<std::string> backends = {"local", "usb=0456:b673", "ip"};
	for (std::vector<std::string>::iterator it = backends.begin(); it != backends.end(); it++) {

		if (*it == "usb=0456:b673") {
#ifdef HAS_LIBUSB1
			// Abort early if no known ADALM-Pluto USB VID:PID (0456:b673) is found,
			// that way we won't block USB access for other drivers' enumeration on Libiio before 0.24.
			libusb_context *usb_ctx = nullptr;
			int r = libusb_init(&usb_ctx);
			if (r < 0) {
				SoapySDR_logf(SOAPY_SDR_WARNING, "libusb init error (%d)\n", r);
			}
			else {
				// This is what libusb_open_device_with_vid_pid(usb_ctx, 0x0456, 0xb673) does,
				// but without actually opening a device.
				struct libusb_device **devs;
				// this is cached in libusb, we won't block USB access for other drivers
				r = libusb_get_device_list(usb_ctx, &devs);
				if (r < 0) {
					SoapySDR_logf(SOAPY_SDR_WARNING, "libusb get device list error (%d)\n", r);
					continue; // iio scan context will most likely fail too?
				}

				bool found = false;
				struct libusb_device *dev;
				size_t i = 0;
				while ((dev = devs[i++]) != NULL) {
					struct libusb_device_descriptor desc;
					// this is cached in libusb, we won't block USB access for other drivers
					r = libusb_get_device_descriptor(dev, &desc);
					if (r < 0) {
						break;
					}
					if (desc.idVendor == 0x0456 && desc.idProduct == 0xb673) {
						found = true;
						break;
					}
				}

				libusb_free_device_list(devs, 1);

				if (found) {
					SoapySDR_logf(SOAPY_SDR_DEBUG, "ADALM-Pluto VID:PID found");
				}
				else {
					SoapySDR_logf(SOAPY_SDR_DEBUG, "No ADALM-Pluto VID:PID found");
					continue;
				}
			}
#endif
			// Defer to other drivers, prevent a race condition on USB enumeration with Libiio before 0.24,
			// the value of 500ms has not been confirmed and might be 50ms to 1s possibly.
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}

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

		options["device"] = "PlutoSDR";
		if (ret == 0) {
			iio_context_info_list_free(info);
			iio_scan_context_destroy(scan_ctx);

			//no devices discovered, the user must specify a hostname
			if (args.count("hostname") == 0) continue;

			//try to connect at the specified hostname
			ctx = iio_create_network_context(args.at("hostname").c_str());
			if (ctx == nullptr) continue; //failed to connect
			options["hostname"] = args.at("hostname");

			std::ostringstream label_str;
			label_str << options["device"] << " #0 " << options["hostname"];
			options["label"] = label_str.str();

			results.push_back(options);
			if (ctx != nullptr) iio_context_destroy(ctx);

		} else {
			for (int i = 0; i < ret; i++) {
				ctx = iio_create_context_from_uri(iio_context_info_get_uri(info[i]));
				if (ctx != nullptr) {
					options["uri"] = std::string(iio_context_info_get_uri(info[i]));

					// check if discovered libiio context can be a PlutoSDR (and not some other sensor),
          // it must contain "ad9361-phy", "cf-ad9361-lpc" and "cf-ad9361-dds-core-lpc" devices
					iio_device *dev = iio_context_find_device(ctx, "ad9361-phy");
					iio_device *rx_dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
					iio_device *tx_dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");

					if (dev != nullptr && rx_dev != nullptr && tx_dev != nullptr) {
						// if uri is specified in kwargs, discovered uri must match
						if (args.count("uri") == 0 || options["uri"] == args.at("uri")) {
							std::ostringstream label_str;
							label_str << options["device"] << " #" << i << " " << options["uri"];
							options["label"] = label_str.str();

							results.push_back(options);
						}
					}

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
