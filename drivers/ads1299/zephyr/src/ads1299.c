//ads1299.c
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>



#include <ncs_version.h>
#if NCS_VERSION_NUMBER >= 0x20600
#include <zephyr/internal/syscall_handler.h>
#else
#include <zephyr/syscall_handler.h>
#endif


#include "ads1299.h"


#define PRINT_BIT(val, n) ((val & BIT(n)) ? '1' : '0')

#define SPIOP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)
// #define SPIOP (SPI_WORD_SET(8))



LOG_MODULE_REGISTER(ADS1299, LOG_LEVEL_DBG);


struct ads1299_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec drdy;
    struct gpio_dt_spec pwdn;
    struct gpio_dt_spec reset;
    struct gpio_dt_spec start;
};

static struct gpio_callback drdy_cb_data;
static void drdy_interrupt_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    printk("DRDY interrupt triggered!\n");
}
static int init_gpios(const struct device *dev)
{
    const struct ads1299_config *cfg = dev->config;
    int err;

    // Ensure the GPIO devices are ready
    if (!device_is_ready(cfg->drdy.port)) {
        printk("DRDY GPIO device not ready\n");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->pwdn.port)) {
        printk("PWDN GPIO device not ready\n");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->reset.port)) {
        printk("RESET GPIO device not ready\n");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->start.port)) {
        printk("START GPIO device not ready\n");
        return -ENODEV;
    }

    // Configure DRDY pin as input with interrupt
    err = gpio_pin_configure_dt(&cfg->drdy, GPIO_INPUT);
    if (err < 0) {
        printk("Error configuring DRDY pin\n");
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&cfg->drdy, GPIO_INT_EDGE_TO_ACTIVE);
    if (err < 0) {
        printk("Error configuring interrupt for DRDY pin\n");
        return err;
    }
    gpio_init_callback(&drdy_cb_data, drdy_interrupt_callback, BIT(cfg->drdy.pin));
    err = gpio_add_callback(cfg->drdy.port, &drdy_cb_data);
    if (err < 0) {
        printk("Error adding DRDY callback\n");
        return err;
    }

    // Configure PWDN pin as output and set inactive (high)
    err = gpio_pin_configure_dt(&cfg->pwdn, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        printk("Error configuring PWDN pin\n");
        return err;
    }

    // Configure RESET pin as output and set inactive (high)
    err = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        printk("Error configuring RESET pin\n");
        return err;
    }

    // Configure START pin as output and set inactive (depends on active level)
    err = gpio_pin_configure_dt(&cfg->start, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        printk("Error configuring START pin\n");
        return err;
    }

    printk("GPIOs initialized\n");
    return 0;
}



// This function writes a value of one byte to a register in the ADS1299
static int ads1299_write_register(const struct device *dev, uint8_t reg, uint8_t data)
{
    int err;
    const struct ads1299_config *cfg = dev->config;
    uint8_t tx_buf[3];

    tx_buf[0] = (0x04 << 5) | reg;
    tx_buf[1] = 0x00;
    tx_buf[2] = data;

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        printk("spi_write_dt() failed, err %d", err);
        return err;
    }

    return 0;

}

static int ads1299_send_command(const struct device *dev, uint8_t command)
{
    int err;
    const struct ads1299_config *cfg = dev->config;
    uint8_t tx_buf[1];

    tx_buf[0] = command;

    struct spi_buf tx_spi_buf = {.buf = (void*)&tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        printk("spi_write_dt() failed, err %d", err);
        return err;
    }

    return 0;
}

static int ads1299_read_data(const struct device *dev, uint8_t * read_buffer, uint8_t read_size)
{
    int err;

}

// This function reads a value of one byte from a register in the ADS1299
static int ads1299_read_reg(const struct device *dev, uint8_t reg, uint8_t *data, int size)
{
    int err;
    const int total_length = 2 + size; // 2 command bytes + data bytes
    uint8_t tx_buffer[total_length];
    uint8_t rx_buffer[total_length];

    // Form the RREG command
    tx_buffer[0] = 0x20 | reg;      // RREG command with register address
    tx_buffer[1] = size - 1;        // Number of registers to read minus one

    // Fill in dummy bytes to clock out the data
    memset(&tx_buffer[2], 0x00, size);

    printk("Sending: ");
    for (int i = 0; i < total_length; i++) {
        printk("%02x ", tx_buffer[i]);
    }
    printk("\n");

    struct spi_buf tx_spi_buf = {.buf = tx_buffer, .len = total_length};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_buf = {.buf = rx_buffer, .len = total_length};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count = 1};

    const struct ads1299_config *cfg = dev->config;

    err = spi_transceive_dt(&cfg->spi, &tx_spi_buf_set, &rx_spi_buf_set);
    if (err < 0) {
        printk("spi_transceive_dt() failed, err: %d\n", err);
        return err;
    }

    // Extract the received data starting after the command bytes
    memcpy(data, &rx_buffer[2], size);

    printk("Received: ");
    for (int i = 0; i < size; i++) {
        printk("%02x ", data[i]);
    }
    printk("\n");

    return 0;
}






static int init_ads1299(const struct device *dev)
{
    int err;
    const struct ads1299_config *cfg = dev->config;

    err = init_gpios(dev);
    if (err < 0) {
        printk("Error initializing GPIOs\n");
        return err;
    }

    k_msleep(100); // Allow time for the device to power up

    // The PWDN and RESET pins are already set to inactive (high) during initialization

    // Send SDATAC command to stop continuous data mode
    err = ads1299_send_command(dev, 0x11);
    if (err < 0) {
        printk("Error sending SDATAC command\n");
        return err;
    }
    k_msleep(1); // Small delay to ensure command is processed

    // Optional: Send WAKEUP command if the device is in standby
    err = ads1299_send_command(dev, 0x02);
    if (err < 0) {
        printk("Error sending WAKEUP command\n");
        return err;
    }
    k_msleep(1);

    // Read Device ID
    uint8_t device_id;
    err = ads1299_read_reg(dev, 0x01, &device_id, 1);
    if (err < 0) {
        printk("Error reading device ID\n");
        return err;
    }
    printk("Device ID: 0x%x\n", device_id);

    printk("ADS1299 driver initialized\n");
    return 0;
}


#define ADS1299_CONFIG_SPI(inst) \
{ \
    .spi = SPI_DT_SPEC_INST_GET(inst, SPIOP, 0), \
    .drdy = GPIO_DT_SPEC_INST_GET(inst, drdy_gpios), \
    .pwdn = GPIO_DT_SPEC_INST_GET(inst, pwdn_gpios), \
    .reset = GPIO_DT_SPEC_INST_GET(inst, reset_gpios), \
    .start = GPIO_DT_SPEC_INST_GET(inst, start_gpios), \
}


#define INST_DRDY_GPIO_SPEC(idx)                              \
    static const struct gpio_dt_spec drdy_##idx = \
        GPIO_DT_SPEC_GET(DT_DRV_INST(idx), drdy_gpios);\

#define ADS1299_DEFINE(inst)						\
    static struct ads1299_data ads1299_data_##inst;			\
    static const struct ads1299_config ads1299_config_##inst = ADS1299_CONFIG_SPI(inst);	\
    INST_DRDY_GPIO_SPEC(inst)						\
    DEVICE_DT_INST_DEFINE(inst,			\
                init_ads1299,							\
                NULL,							\
                &ads1299_data_##inst,	\
                &ads1299_config_##inst,\
                POST_KERNEL, \
                CONFIG_MAX30003_INIT_PRIORITY, \
                NULL);

/* STEP 5.2 - Create the struct device for every status "okay" node in the devicetree */
DT_INST_FOREACH_STATUS_OKAY(ADS1299_DEFINE)




