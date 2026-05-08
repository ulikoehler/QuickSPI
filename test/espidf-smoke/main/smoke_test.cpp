#include <QuickSPI.h>

namespace {

class SmokeDevice : public QuickSPIDevice {
public:
    using QuickSPIDevice::QuickSPIDevice;
};

static_assert(std::is_base_of<QuickSPIDevice, SmokeDevice>::value, "SmokeDevice must derive from QuickSPIDevice");

}

extern "C" void app_main(void) {
    spi_device_interface_config_t config = {};
    config.clock_speed_hz = 1000000;
    config.mode = 0;
    config.spics_io_num = -1;
    config.queue_size = 1;

    (void)config;
}
