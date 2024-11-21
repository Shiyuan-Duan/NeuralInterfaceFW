//afe4960.c
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


#include "afe4960.h"

#define PRINT_BIT(val, n) ((val & BIT(n)) ? '1' : '0')

#define SPIOP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

// headers
static int _flush_fifo(const struct device *dev, uint8_t *dest);



LOG_MODULE_REGISTER(AFE4960, LOG_LEVEL_DBG);

struct afe4960_data
{
    uint8_t chip_id;
    struct k_sem drdy_sem;            // Semaphore for DRDY signaling
    struct gpio_callback drdy_cb;     // GPIO callback structure for DRDY
};

struct afe4960_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec drdy;
    struct gpio_dt_spec rst;
};

static struct gpio_callback drdy_cb_data;

static int _afe4960_enable_read(const struct device *dev)
{
    int err;
    const struct afe4960_config *cfg = dev->config;
    uint8_t tx_buf[4];
    tx_buf[0] = 0x00;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00 | BIT(0); // Enable read

    uint8_t rx_buf[4] = {0};
    // tx_buf[3] |= BIT(6); // Enable FIFO
    LOG_DBG("Enabling read writing 0x%02x\n", tx_buf[3]);

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi = {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_buf = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set rx_spi = {.buffers = &rx_spi_buf, .count = 1};

    err = spi_transceive_dt(&cfg->spi, &tx_spi, &rx_spi);

    for(int i = 0; i < 4; i++)
    {
        printk("Enable read RX Data[%d]: %x\n", i, rx_buf[i]);
    }

    if (err < 0) {
        LOG_ERR("Error enabling read");
        return err;
    }

    return 0;

}

static int _afe4960_enable(const struct device *dev)
{
    int err;
    const struct afe4960_config *cfg = dev->config;
    uint8_t tx_buf[4] = {0};
    tx_buf[0] = 0x1D;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi = {.buffers = &tx_spi_buf, .count = 1};

    err = spi_write_dt(&cfg->spi, &tx_spi);
    if (err < 0) {
        LOG_ERR("Error enabling AFE4960");
        return err;
    }

    tx_buf[0] = 0x00;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00 | BIT(1); // TM_COUNT_RST

    err = spi_write_dt(&cfg->spi, &tx_spi);

    tx_buf[3] = 0x00 | BIT(6); // ENABLE_FIFO

    err = spi_write_dt(&cfg->spi, &tx_spi);

    tx_buf[0] = 0x1D;
    tx_buf[1] = (0x00 | BIT(6)) | BIT(7); // // RAC_COUNTER_ENALBE | TIMER_ENALBE
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;

    err = spi_write_dt(&cfg->spi, &tx_spi);
    
    return 0;

}
// static int _afe4960_disable_read(const struct device *dev)
// {
//     int err;
//     const struct afe4960_config *cfg = dev->config;
//     uint8_t tx_buf[4] = {0};
//     tx_buf[0] = 0x00;
//     tx_buf[1] = 0x00;
//     tx_buf[2] = 0x00;

//     tx_buf[3] |= BIT(6); // Enable FIFO
//     LOG_DBG("Enabling read writing 0x%02x\n", tx_buf[3]);

//     struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
//     struct spi_buf_set tx_spi = {.buffers = &tx_spi_buf, .count = 1};

//     err = spi_write_dt(&cfg->spi, &tx_spi);
//     if (err < 0) {
//         LOG_ERR("Error enabling read");
//         return err;
//     }
//     return 0;
// }

static int _afe4960_software_reset(const struct device *dev)
{
    int err;
    const struct afe4960_config *cfg = dev->config;
    uint8_t tx_buf[4] = {0};
    tx_buf[0] = 0x00;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00 | BIT(3); // Reset

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi = {.buffers = &tx_spi_buf, .count = 1};

    err = spi_write_dt(&cfg->spi, &tx_spi);
    if (err < 0) {
        LOG_ERR("Error enabling AFE4960");
        return err;
    }
    k_msleep(2); // Wait for 2 milliseconds
    return 0;
}

static int _afe4960_reset(const struct device *dev)
{
    const struct afe4960_config *cfg = dev->config;
    int err;

    // Pull the reset GPIO pin low
    err = gpio_pin_configure_dt(&cfg->rst, GPIO_OUTPUT_ACTIVE);
    if (err < 0) {
        LOG_ERR("Error setting RST pin low");
        return err;
    }

    uint8_t tx_buf[4];
    tx_buf[0] = 0x00;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00 | BIT(3); // Reset

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error sending reset command");
        return err;
    }

    // Wait for 50 microseconds
    k_usleep(40);

    // Pull the reset GPIO pin high
    err = gpio_pin_configure_dt(&cfg->rst, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        LOG_ERR("Error setting RST pin high");
        return err;
    }

    k_msleep(2); // Wait for 2 milliseconds




    LOG_INF("AFE4960 reset completed");
    return 0;
}

static int drdy_interrupt_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    struct afe4960_data *data = CONTAINER_OF(cb, struct afe4960_data, drdy_cb);
    // printk("DRDY interrupt triggered\n");
    // k_sem_give(&data->drdy_sem); // Release the semaphore
    return 0;
}


static int init_gpios(const struct device *dev)
{
    struct afe4960_data *data = dev->data;
    const struct afe4960_config *cfg = dev->config;
    int err;

    /* Ensure all GPIO devices are ready */
    if (!device_is_ready(cfg->drdy.port)) {
        LOG_ERR("DRDY GPIO device not ready\n");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->rst.port)) {
        LOG_ERR("RST GPIO device not ready\n");
        return -ENODEV;
    }else{
        LOG_INF("RST GPIO device ready\n");
    }

    /* Configure DRDY pin as input with pull-up and interrupt on falling edge */
    err = gpio_pin_configure_dt(&cfg->drdy, GPIO_INPUT | GPIO_PULL_DOWN);
    if (err < 0) {
        LOG_ERR("Error configuring DRDY pin\n");
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&cfg->drdy, GPIO_INT_EDGE_RISING);
    if (err < 0) {
        LOG_ERR("Error configuring DRDY interrupt\n");
        return err;
    }

    /* Initialize the GPIO callback */
    gpio_init_callback(&data->drdy_cb, drdy_interrupt_callback, BIT(cfg->drdy.pin));
    err = gpio_add_callback(cfg->drdy.port, &data->drdy_cb);
    if (err < 0) {
        LOG_ERR("Error adding DRDY callback\n");
        return err;
    }

    /* Configure RST pin as output and set it inactive (high) */
    err = gpio_pin_configure_dt(&cfg->rst, GPIO_OUTPUT_INACTIVE | GPIO_PULL_UP);
    if (err < 0) {
        LOG_ERR("Error configuring RST pin\n");
        return err;
    }

    // LOG_INF("GPIOs initialized successfully\n");
    return 0;
}

static int _afe4960_read_reg(const struct device *dev)
{
    int err;


    return err;
}
static int _afe4960_check_id(const struct device *dev)
{
    int err;
    const struct afe4960_config *cfg = dev->config;

    if (err < 0) {
        LOG_ERR("Failed to enable read mode");
        return err;
    }

    uint8_t tx_buf[4] = {0x28, 0X00, 0X00, 0X00};
    uint8_t rx_buf[4] = {1};


    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_buf = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count = 1};


    err = spi_transceive_dt(&cfg->spi, &tx_spi_buf_set, &rx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("spi_transceive_dt() failed, err: %d\n", err);
        return err;
    }
    LOG_INF("TX Data: %x\n", tx_buf[0]);
    for(int i = 0; i < 4; i++)
    {
        LOG_INF("rx Data[%d]: %x\n", i, rx_buf[i]);
        k_msleep(10);
    }


    if (err < 0) {
        LOG_ERR("Failed to reset device");
        return err;
    }

    


    return 0;
}
static int _afe4960_read_fifo(const struct device *dev)
{
    int err;
    const struct afe4960_config *cfg = dev->config;
    uint8_t tx_buf[1] = {0xFF};
    uint8_t rx_buf[40] = {1};

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_buf = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count = 1};

    err = spi_transceive_dt(&cfg->spi, &tx_spi_buf_set, &rx_spi_buf_set);

    if (err < 0) {
        LOG_ERR("spi_transceive_dt() failed, err: %d\n", err);
        return err;
    }
    for(int i = 0; i < 40; i++)
    {
        LOG_INF("fifo rx Data[%d]: %x\n", i, rx_buf[i]);
        k_msleep(100);
    }

    return 0;
}

static int _afe4960_config_electrodes(const struct device *dev)
{
    int err;
    const struct afe4960_config *cfg = dev->config;
    // Configure bioz connection
    uint8_t tx_buf[4] = {0};
    tx_buf[0] = 0xCB;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x78;
    tx_buf[3] = 0x34;

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring electrodes");
        return err;
    }
    // Configure ECG connection
    tx_buf[0] = 0xC9;
    tx_buf[1] = 0x42;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;

    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring electrodes");
        return err;
    }

    return 0;
}

static int _afe4960_config_fifo(const struct device *dev)
{   
    int err;
    const struct afe4960_config *cfg = dev->config;

    uint8_t tx_buf[4] = {0};

    // Set FIFO WM to 100
    // tx_buf[0] = 0x42;
    // tx_buf[1] = 0x00; // Set fifo interrupt to be adc_rdy
    // tx_buf[2] = 0x19;
    // tx_buf[3] = 0x20;


    // Set FIFO WM to 128

    tx_buf[0] = 0x42;
    tx_buf[1] = 0x00; // Set fifo interrupt to be adc_rdy
    tx_buf[2] = 0x1F;
    tx_buf[3] = 0xD0;
    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring FIFO");
        return err;
    }

    tx_buf[0] = 0xBE;
    tx_buf[1] = 0x40;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;

    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring FIFO\n");
        return err;
    }
    
    return 0;
}

static int _flush_fifo(const struct device *dev, uint8_t *dest)
{
    int err;
    const struct afe4960_config *cfg = dev->config;
    const struct afe4960_data *data = dev->data;
    k_sem_take(&data->drdy_sem, K_FOREVER);

    uint8_t tx_buf[1] = {0xFF};
    uint8_t rx_buf[128*3 + 1] = {0};

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_buf = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count = 1};

    err = spi_transceive_dt(&cfg->spi, &tx_spi_buf_set, &rx_spi_buf_set);

    if (err < 0) {
        LOG_ERR("spi_transceive_dt() failed, err: %d\n", err);
        return err;
    }

    memcpy(dest, &rx_buf[1], 128*3);
    return 0;
}

static int _afe4960_config_rac(const struct device *dev)
{
    int err;
    const struct afe4960_config *cfg = dev->config;

    uint8_t tx_buf[4] = {0};
    // Set RA
    // Set reg C1 bit 15-0 to 256 
    tx_buf[0] = 0xC1;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x01;
    tx_buf[3] = 0x00;

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring RAC");
        return err;
    }

    // CONFIG_TSM
    tx_buf[0] = 0xC2;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    // tx_buf[3] = 0x19; // TS1 = E, TS2 = B;
    tx_buf[3] = 0x01; // TS1 = E, TS2 = Dummy;

    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring RAC");
        return err;
    }

    // CONFIG_NUM_TS_MIX1
    tx_buf[0] = 0xC4;
    tx_buf[1] = 0x00;
    // tx_buf[2] = 0x04;
    // tx_buf[3] = 0x12;

    tx_buf[2] = 0x00;
    tx_buf[3] = 0x10;


    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring RAC");
        return err;
    }


    return 0;
}
static int _afe4960_set_bioz_current_freq(const struct device *dev)
{
    int err;
    const struct afe4960_config *cfg = dev->config;
    uint8_t tx_buf[4] = {0};
    // Set bioz current frequency
    tx_buf[0] = 0xC0;
    tx_buf[1] = 0x02;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring bioz current frequency");
        return err;
    }

    // phi step = 256
    tx_buf[0] = 0xCC;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x01;
    tx_buf[3] = 0x00;

    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring bioz current frequency");
        return err;
    }

    // Set bioz V 
    tx_buf[0] = 0xCD;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x08;

    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring bioz current frequency");
        return err;
    }




    return 0;
}
static int _afe4960_config_acq(const struct device *dev)
{
    int err;
    const struct afe4960_config *cfg = dev->config;
    uint8_t tx_buf[4] = {0};
    // Set ACQ
    tx_buf[0] = 0x0A;
    tx_buf[1] = ((0x00 | BIT(7)) | BIT(4)) | BIT(2); // EN_ECG_RX EN_BIOZ_RX EN_BIOZ_TX
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x4A; // Set to 74(Decimal)

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};

    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring ACQ");
        return err;
    }


    // DIS_BUF_PDN_ON_ADC
    tx_buf[0] = 0x4B;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x01;
    tx_buf[3] = 0x00;

    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("Error configuring ACQ");
        return err;
    }

    return 0;
}

static int init_afe4960(const struct device *dev)
{
    int err;
    const struct afe4960_config *cfg = dev->config;
    struct afe4960_data *data = dev->data;

    k_sem_init(&data->drdy_sem, 0, 1);
    
    
    LOG_INF("Initializing AFE4960");
    err = init_gpios(dev);
    err = _afe4960_software_reset(dev);
    err = _afe4960_enable_read(dev);
    k_msleep(2); // Wait for 2 milliseconds
    err = _afe4960_check_id(dev);

    err = _afe4960_software_reset(dev);
    k_msleep(2); // Wait for 2 milliseconds


    err = _afe4960_config_acq(dev);
    err = _afe4960_config_rac(dev);
    err = _afe4960_config_fifo(dev);
    err = _afe4960_config_electrodes(dev);
    // err = _afe4960_set_bioz_current_freq(dev);


    err = _afe4960_enable(dev);



}



// static const struct ads1299_driver_api_funcs ads1299_api = {


static const struct afe4960_driver_api_funcs afe4960_api = {
    .read_data = _flush_fifo,
};

#define AFE4960_CONFIG(inst) \
{ \
    .spi = SPI_DT_SPEC_INST_GET(inst, SPIOP, 0), \
    .drdy = GPIO_DT_SPEC_INST_GET(inst, drdy_gpios), \
    .rst = GPIO_DT_SPEC_INST_GET(inst, rst_gpios), \
}



#define AFE4960_DEFINE(inst)                                    \
    static struct afe4960_data afe4960_data_##inst;            \
    static const struct afe4960_config afe4960_config_##inst = \
        AFE4960_CONFIG(inst);                              \
    DEVICE_DT_INST_DEFINE(inst,                                \
                init_afe4960,                                   \
                NULL,                                           \
                &afe4960_data_##inst,                          \
                &afe4960_config_##inst,                        \
                POST_KERNEL,                                    \
                CONFIG_AFE4960_INIT_PRIORITY,                  \
                &afe4960_api);

/* Instantiate the driver for each device tree node with status "okay" */
DT_INST_FOREACH_STATUS_OKAY(AFE4960_DEFINE)