#ifndef AFE4960_H_
#define AFE4960_H_

#define DT_DRV_COMPAT ti_afe4960
#include <zephyr/device.h>


typedef int  (*afe4960_api_read_data)(const struct device * dev, uint8_t *dest);


struct afe4960_driver_api_funcs {
    afe4960_api_read_data read_data;
};


__syscall     int        afe4960_read_data(const struct device * dev, uint8_t *dest);
static inline int z_impl_afe4960_read_data(const struct device * dev, uint8_t *dest)
{
    const struct afe4960_driver_api_funcs *api = dev->api;

    __ASSERT(api->read_data, "Callback pointer should not be NULL");

    return api->read_data(dev, dest);
}


#include <syscalls/afe4960.h>
#endif