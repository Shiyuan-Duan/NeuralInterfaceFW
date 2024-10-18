

#define PRINT_BIT(val, n) ((val & BIT(n)) ? '1' : '0')
#define SPIOP	SPI_WORD_SET(8)

#include "w25m02gw.h"


LOG_MODULE_REGISTER(W25M02GW, LOG_LEVEL_DBG);



struct w25m02gw_config
{
    struct spi_dt_spec spi;
    struct gpio_dt_spec hold;
    struct gpio_dt_spec wp;
};

struct w25m02gw_data
{

    uint8_t chip_id[3];
    uint16_t num_of_blocks;
    uint16_t current_block; 
    uint8_t current_page;


};

static int _uint16to8(uint16_t val, uint8_t *buf)
{
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
    return 0;
}




static int w25m02gw_read_reg(const struct device *dev, uint8_t reg, uint8_t *data, int size)
{
    int err;
    uint8_t tx_buffer[2]; 
    tx_buffer[0] = reg;
    tx_buffer[1] = 0x00;




    struct spi_buf tx_spi_buf = {.buf = (void *)&tx_buffer, .len = sizeof(tx_buffer)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_buf = {.buf = data, .len = size};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count=1};

    const struct w25m02gw_config *cfg = dev->config;
    
    err = spi_transceive_dt(&cfg->spi, &tx_spi_buf_set, &rx_spi_buf_set);
    if (err < 0) {
        printk("spi_transceive_dt() failed, err: %d", err);
        return err;
    }

    return 0;

}

static int w25m02gw_spi_transceive(const struct device *dev, uint8_t * tx_buf, size_t tx_size, uint8_t * rx_buf, int rx_size)
{
    int err;

    struct spi_buf tx_spi_buf = {.buf = (void *)tx_buf, .len = tx_size};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_buf = {.buf = rx_buf, .len = rx_size};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count=1};

    const struct w25m02gw_config *cfg = dev->config;
    
    err = spi_transceive_dt(&cfg->spi, &tx_spi_buf_set, &rx_spi_buf_set);
    if (err < 0) {
        printk("spi_transceive_dt() failed, err: %d", err);
        return err;
    }

    return 0;

}

static bool _is_busy(const struct device *dev)
{
    int err = 0;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[3];

    tx_buffer[0] = INSTRU_READ_STATUS;
    tx_buffer[1] = 0xC0;

    err = w25m02gw_spi_transceive(dev, tx_buffer, 2, rx_buffer, 3);
    uint8_t status3 = rx_buffer[2];

    return (status3 & BIT(0));

}

static bool _is_write_enabled(const struct device *dev)
{
    int err = 0;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[3];

    tx_buffer[0] = INSTRU_READ_STATUS;
    tx_buffer[1] = 0xC0;

    err = w25m02gw_spi_transceive(dev, tx_buffer, 2, rx_buffer, 3);
    uint8_t status3 = rx_buffer[2];

    return (status3 & BIT(1));
}
static bool _is_pfail(const struct device *dev)
{
    int err = 0;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[3];

    tx_buffer[0] = INSTRU_READ_STATUS;
    tx_buffer[1] = 0xC0;

    err = w25m02gw_spi_transceive(dev, tx_buffer, 2, rx_buffer, 3);
    uint8_t status3 = rx_buffer[2];

    return (status3 & BIT(3));
}
static int _read_s1(const struct device *dev)
{
    int err = 0;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[3];

    tx_buffer[0] = INSTRU_READ_STATUS;
    tx_buffer[1] = 0xA0;

    err = w25m02gw_spi_transceive(dev, tx_buffer, 2, rx_buffer, 3);
    uint8_t status1 = rx_buffer[2];

    LOG_INF("Status1: %x", status1);
    return err;

}

static int _unlock_all_blocks(const struct device *dev)
{
    int err = 0;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[3];

    tx_buffer[0] = INSTRU_READ_STATUS;
    tx_buffer[1] = 0xA0;

    err = w25m02gw_spi_transceive(dev, tx_buffer, 2, rx_buffer, 3);
    uint8_t status1 = rx_buffer[2];


    status1 &= 0x83;


    uint8_t sr_tx_buffer[3];
    sr_tx_buffer[0] = INSTRU_WRITE_STATUS;
    sr_tx_buffer[1] = 0XA0;
    sr_tx_buffer[2] = status1;

    err = w25m02gw_spi_transceive(dev, sr_tx_buffer, 3, NULL, 0);
    if (err < 0) {
        printk("Error writing status");
        return err;
    }

    
    return err;
}

static int _enable_otp(const struct device *dev)
{
    int err = 0;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[3];

    tx_buffer[0] = INSTRU_READ_STATUS;
    tx_buffer[1] = 0xB0;

    err = w25m02gw_spi_transceive(dev, tx_buffer, 2, rx_buffer, 3);
    uint8_t status2 = rx_buffer[2];

    uint8_t sr_tx_buffer[3];
    sr_tx_buffer[0] = INSTRU_WRITE_STATUS;
    sr_tx_buffer[1] = 0XB0;
    sr_tx_buffer[2] = status2 | BIT(6);

    err = w25m02gw_spi_transceive(dev, sr_tx_buffer, 3, NULL, 0);
    if (err < 0) {
        printk("Error writing status");
        return err;
    }

    return err;
}

static int _read_s2(const struct device *dev)
{
    int err = 0;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[3];

    tx_buffer[0] = INSTRU_READ_STATUS;
    tx_buffer[1] = 0xB0;

    err = w25m02gw_spi_transceive(dev, tx_buffer, 2, rx_buffer, 3);
    uint8_t status2 = rx_buffer[2];

    LOG_INF("Status2: %x", status2);
    return err;

}

static int w25m02gw_read_id(const struct device *dev)
{
    int err = 0;

    uint8_t chip_id_buf[5];
    err = w25m02gw_read_reg(dev, INSTRU_W25M02GW_ID, chip_id_buf, sizeof(chip_id_buf));

    printk("Chip ID: %x %x %x %x %x", chip_id_buf[0], chip_id_buf[1], chip_id_buf[2], chip_id_buf[3], chip_id_buf[4]);


    return err;
}

static int w25m02gw_read_lut(const struct device *dev)
{
    int err = 0;

    uint8_t lut_buf[80];
    err = w25m02gw_read_reg(dev, INSTRU_W25M02GW_READ_LUT, lut_buf, sizeof(lut_buf));

    for(int i = 0; i < 80; i++)
    {
        LOG_INF("LUT%d: %x",i , lut_buf[i]);
        k_msleep(100);
    }

    return err;
}

static int w25m02gw_bbm_manage(const struct device *dev)
{
    // TODO: Implement this function
    return ;
}
static int _hard_reset(const struct device *dev)
{
    int err = 0;
    uint8_t spi_tx_data[1];
    spi_tx_data[0] = 0xff;
    err = w25m02gw_spi_transceive(dev, spi_tx_data, 1, NULL, 0);
    if (err < 0) {
        printk("Error hard reset");
        return err;
    }
    spi_tx_data[0] = 0x66;
    err = w25m02gw_spi_transceive(dev, spi_tx_data, 1, NULL, 0);
    if (err < 0) {
        printk("Error hard reset");
        return err;
    }

    spi_tx_data[0] = 0x99;
    err = w25m02gw_spi_transceive(dev, spi_tx_data, 1, NULL, 0);
    if (err < 0) {
        printk("Error hard reset");
        return err;
    }
    return err;

}
static int write_enable(const struct device *dev)
{
    int err = 0;
    if(_is_write_enabled(dev)){
        LOG_DBG("Write already enabled\n");
    }else{
        LOG_DBG("Write not enabled\n");
    }
    if(_is_busy(dev)){
        LOG_DBG("DISK Busy\n");
        k_yield();
    }
    uint8_t spi_tx_data[1];
    spi_tx_data[0] = INSTRU_WRITE_ENABLE;
    err = w25m02gw_spi_transceive(dev, spi_tx_data, 1, NULL, 0);
    if (err < 0) {
        printk("Error writing enable");
        return err;
    }

        if(_is_write_enabled(dev)){
        LOG_DBG("Write already enabled\n");
    }else{
        LOG_DBG("Write not enabled\n"); 
    }
    return 0;
}

static int _form_query_addr(uint16_t block_addr, uint8_t page_addr, uint8_t *spi_tx_addr)
{
    uint16_t addr = 0;

    // Block address is in the upper 10 bits, page address is in the lower 6 bits
    addr = (block_addr << 6) | (page_addr & 0x3F);

    spi_tx_addr[0] = (addr >> 8) & 0xFF; // MSB of the address
    spi_tx_addr[1] = addr & 0xFF;        // LSB of the address

    return addr;
}
// static int _form_query_addr(uint16_t block_addr, uint8_t page_addr, uint8_t *spi_tx_addr)
// {
//     uint16_t addr = (page_addr << 10) | (block_addr & 0x03FF);

//     spi_tx_addr[0] = (addr >> 8) & 0xFF; // MSB of the address
//     spi_tx_addr[1] = addr & 0xFF;        // LSB of the address

//     return addr;
// }

static int read_buffer(const struct device *dev)
{
    while (_is_busy(dev)){
        LOG_DBG("DISK Busy\n");
        k_yield();
    }
    int err = 0;
    uint8_t instruction_buffer[4];
    uint8_t rx_buf[2048+4];
    instruction_buffer[0] = INSTRU_READ_BUF;
    instruction_buffer[1] = 0x00;
    instruction_buffer[2] = 0x00;
    instruction_buffer[3] = 0x00;

    err = w25m02gw_spi_transceive(dev, instruction_buffer, 4, rx_buf, 2052);
    if (err < 0) {
        printk("Error reading buffer");
        return err;
    }
    return  err;

}
static int _load_data_to_write(const struct device *dev, uint16_t column_addr, uint8_t * buffer, size_t buffer_size)
{
    int err = 0;
    while(_is_busy(dev)){
        LOG_DBG("DISK Busy\n");
        k_yield();

    }

    uint8_t spi_tx_data[buffer_size + 3];
    spi_tx_data[0] = INSTRU_PROGRAM_BUF;
    _uint16to8(column_addr, &spi_tx_data[1]);

    memcpy(&spi_tx_data[3], buffer, buffer_size);

    // for (int i = 0; i < buffer_size + 3; i++){
    //     printk("buffer idx: %d, 0x%02x \n", i, spi_tx_data[i]);
    //     k_msleep(3);
    // }

    
    err = w25m02gw_spi_transceive(dev, spi_tx_data, buffer_size + 2, NULL, 0);
    
    if (err < 0) {
        printk("Error loading data to write");
        return err;
    }
    return err;
}

static int _program_execute(const struct device *dev, uint16_t block_addr, uint8_t page_addr)
{
    int err = 0;
    uint8_t spi_tx_data[4];
    spi_tx_data[0] = INSTRU_PROGRAM_EXECUTE;
    spi_tx_data[1] = 0x00;
    _form_query_addr(block_addr, page_addr, &spi_tx_data[2]);

    for (int i = 2; i < 4; i++){
        printk("write spi_tx_data[%d]: 0x%02x\n", i, spi_tx_data[i]);
    }
    err = w25m02gw_spi_transceive(dev, spi_tx_data, 4, NULL, 0);
    if (err < 0) {
        LOG_DBG("Error executing program");
        return err;
    }

    return err;
}

static int _w25m02gw_write(const struct device *dev, uint16_t block_addr, uint8_t page_addr, uint16_t column_addr, uint8_t * buffer, size_t buffer_size)
{

    while(_is_busy(dev)){
        LOG_DBG("DISK Busy\n");
        k_yield();

    }

    LOG_DBG("status of _is_pfail: %d\n", _is_pfail(dev));

    int err = 0;
    err = write_enable(dev);
    err = _load_data_to_write(dev, column_addr, buffer, buffer_size);
    err = write_enable(dev);
    err = _program_execute(dev, block_addr, page_addr);
    LOG_DBG("status of _is_pfail: %d\n", _is_pfail(dev));
    return err;
}
static int w25m02gw_read_buffer(const struct device * dev, uint16_t column_addr, uint8_t * output, size_t output_size)
{
    while (_is_busy(dev)){
        LOG_DBG("DISK Busy\n");
        k_yield();
    }
    int err = 0;
    uint8_t instruction_buffer[4];
    uint8_t rx_buf[output_size+4];
    instruction_buffer[0] = INSTRU_READ_BUF;
    instruction_buffer[1] = 0x00;
    _uint16to8(column_addr, &instruction_buffer[2]);

    err = w25m02gw_spi_transceive(dev, instruction_buffer, 4, rx_buf, output_size + 4);
    if (err < 0) {
        printk("Error reading buffer");
        return err;
    }

    memcpy(output, &rx_buf[4], output_size);
    return  err;
}

static int _w25m02gw_read(const struct device *dev, uint16_t block_addr, uint8_t page_addr, uint16_t column_addr, uint8_t * output, size_t output_size)
{
    int err = 0;
    while(_is_busy(dev)){
        LOG_DBG("DISK Busy\n");
        k_yield();

    }
    uint8_t spi_tx_data[4];
    spi_tx_data[0] = INSTRU_PAGE_DATA_READ;
    spi_tx_data[1] = 0x00;
    _form_query_addr(block_addr, page_addr, &spi_tx_data[2]);

    err = w25m02gw_spi_transceive(dev, spi_tx_data, 4, NULL, 0);
    if(err < 0){
        printk("Error reading data");
        return err;
    }

    err = w25m02gw_read_buffer(dev, column_addr, output, output_size);
    return err;
}


static int _load_data_to_buffer(const struct device *dev, uint16_t block_addr, uint8_t page_addr)
{
    int err = 0;
    while(_is_busy(dev)){
        LOG_DBG("DISK Busy\n");
        k_yield();

    }
    uint8_t spi_tx_data[4];
    spi_tx_data[0] = INSTRU_PAGE_DATA_READ;
    spi_tx_data[1] = 0x00;
    _form_query_addr(block_addr, page_addr, &spi_tx_data[2]);

    for (int i = 2; i < 4; i++){
        printk("read spi_tx_data[%d]: 0x%02x\n", i, spi_tx_data[i]);
    }
    err = w25m02gw_spi_transceive(dev, spi_tx_data, 4, NULL, 0);
    if(err < 0){
        printk("Error loading data to buffer");
        return err;
    }
    return err;
}

static int _read_first_byte_of_all(const struct device *dev)
{
    int err = 0;
    

    for (int ba = 0; ba < 1024; ba++){
        for (int pa = 0; pa < 64; pa++){
            uint8_t spi_tx_data[4];
            spi_tx_data[0] = INSTRU_PAGE_DATA_READ;
            spi_tx_data[1] = 0x00;
            _form_query_addr(ba, pa, &spi_tx_data[2]);
            
            err = w25m02gw_spi_transceive(dev, spi_tx_data, 4, NULL, NULL);
            


            // for (int byte = 0; byte < 2112; byte++){
            //     uint8_t spi_rx_data=0;
            //     spi_tx_data[0] = INSTRU_READ_BUF;
            //     _uint16to8(byte, &spi_tx_data[1]);
            //     spi_tx_data[3] = 0x00;

            //     err = w25m02gw_spi_transceive(dev, NULL, 0, &spi_rx_data, 1);

            //     if (err < 0) {
            //         printk("Error reading first byte of all");
            //         return err;
            //     }
            //     if(spi_rx_data != 0xFF){
            //         LOG_INF("Block: %d, Page: %d, byte: %d, Data: %x\n",ba, pa, byte, spi_rx_data);
            //     }
                

            // }
            

            uint8_t spi_rx_data=0;
            spi_tx_data[0] = INSTRU_READ_BUF;
            _uint16to8(0, &spi_tx_data[1]);
            spi_tx_data[3] = 0x00;

            err = w25m02gw_spi_transceive(dev, NULL, 0, &spi_rx_data, 1);

            if (err < 0) {
                printk("Error reading first byte of all");
                return err;
            }
            if(spi_rx_data != 0xFF){
                LOG_INF("Block: %d, Page: %d, byte: %d, Data: %x\n",ba, pa, 0, spi_rx_data);
            }
            printk("Block: %d, Page: %d, byte: %d, Data: %x\n",ba, pa, 0, spi_rx_data);

        }
    }

    LOG_INF("Read first byte of all done\n");

    return err;
}


static int init_gpios(const struct device *dev)
{
    int err;
    const struct w25m02gw_config *cfg = dev->config;


    
    err = gpio_pin_configure_dt(&cfg->hold, GPIO_OUTPUT_INACTIVE | NRF_GPIO_DRIVE_H0H1);
    if (err) {
        LOG_ERR("Error configuring HOLD GPIO");
        return err;
    }

    err = gpio_pin_configure_dt(&cfg->wp, GPIO_OUTPUT_INACTIVE | NRF_GPIO_DRIVE_H0H1);
    if (err) {
        LOG_ERR("Error configuring WP GPIO");
        return err;
    }

    return 0;
}


static int _erase_block(const struct device *dev, uint16_t block_addr)
{   
    while(_is_busy(dev)){
        LOG_DBG("DISK Busy\n");
        k_yield();

    }
    write_enable(dev);

    int err = 0;
    uint8_t spi_tx_data[4];
    spi_tx_data[0] = INSTRU_ERASE_BLOCK;
    spi_tx_data[1] = 0x00;
    _form_query_addr(block_addr, 0, &spi_tx_data[2]);

    err = w25m02gw_spi_transceive(dev, spi_tx_data, 4, NULL, 0);
    if (err < 0) {
        printk("Error erasing block");
        return err;
    }
    return 0;

}



static int init_device_data(const struct device *dev)
{
    struct w25m02gw_data *data = dev->data;
    data->current_block = 0;
    data->current_page = 0;
    return 0;
}

static int init_w25m02gw(const struct device *dev)
{

    int err = 0;
    err = init_device_data(dev);
    err = _hard_reset(dev);
    err = init_gpios(dev);
    err = _enable_otp(dev);
    err = _unlock_all_blocks(dev);


    w25m02gw_erase(dev, 0);
    uint8_t data_to_write[24] = "Hello World";

    w25m02gw_write(dev, 0, 2, 0, data_to_write, 11);
    uint8_t output[11];
    w25m02gw_read(dev, 0, 2, 0, output, 11);
    printk("Output: %s\n", output);










    return err;

}




static const struct w25m02gw_driver_api_funcs w25m02gw_api = {
    .read= _w25m02gw_read,
    .write = _w25m02gw_write,
    .erase = _erase_block,

};




#define INST_HOLD_GPIO_SPEC(idx)                              \
    static const struct gpio_dt_spec hold_##idx = \
        GPIO_DT_SPEC_GET(DT_DRV_INST(idx), hold_gpios);\


#define INST_WP_GPIO_SPEC(idx)                              \
    static const struct gpio_dt_spec wp_##idx = \
        GPIO_DT_SPEC_GET(DT_DRV_INST(idx), wp_gpios);\


#define W25M02GW_CONFIG(inst) \
    INST_HOLD_GPIO_SPEC(inst) \
    INST_WP_GPIO_SPEC(inst) \
    static const struct w25m02gw_config w25m02gw_config_##inst = { \
    .spi = SPI_DT_SPEC_INST_GET(inst, SPIOP, 0), \
    .hold = hold_##inst, \
    .wp = wp_##inst, \
    }	\
    


#define W25M02GW_DEFINE(inst)						\
    static struct w25m02gw_data w25m02gw_data_##inst;			\
    W25M02GW_CONFIG(inst);						\
    DEVICE_DT_INST_DEFINE(inst,			\
                init_w25m02gw,							\
                NULL,							\
                &w25m02gw_data_##inst,	\
                &w25m02gw_config_##inst,\
                POST_KERNEL, \
                CONFIG_W25M02GW_INIT_PRIORITY, \
                &w25m02gw_api);

/* STEP 5.2 - Create the struct device for every status "okay" node in the devicetree */
DT_INST_FOREACH_STATUS_OKAY(W25M02GW_DEFINE)
