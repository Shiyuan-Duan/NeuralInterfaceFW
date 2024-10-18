

// ble.c
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/reboot.h>

#include "ble.h"
#include "sensor_data_manager.h"
#include "max86141.h"
#include "state.h"

// #define PRIO K_HIGHEST_APPLICATION_THREAD_PRIO + 1
#define PRIO 7

LOG_MODULE_REGISTER(BLE);


static bool sensor_state = false;



// const SystemStatus* sys_stat_ptr; 

static bool stream_sensor_channel1_enabled;
static bool stream_sensor_channel2_enabled;
static bool stream_sensor_channel3_enabled;
static bool stream_sensor_channel4_enabled;
static bool stream_sensor_channel5_enabled;
static bool stream_sensor_channel6_enabled;
static bool stream_sensor_channel7_enabled;
static bool stream_sensor_channel8_enabled;
static bool stream_sensor_hr_enabled;
static bool stream_sensor_spo2_enabled;
static bool stream_sensor_temp_enabled;
static bool stream_sensor_glucose_enabled;
static bool is_streaming = false;
static bool is_sensor_on = false;

static char dbg_message[1024]="Hello world\n\0";
static struct ble_cb cb;



static void ble_ccc_stream_channel1_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel1_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel2_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel2_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel3_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel3_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel4_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel4_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel5_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel5_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel6_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel6_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel7_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel7_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel8_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel8_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_hr_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_hr_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_spo2_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_spo2_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_temp_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_temp_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_glucose_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_glucose_enabled = (value == BT_GATT_CCC_NOTIFY);
}




static ssize_t write_sensor_sw(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t value = *((uint8_t *)buf);

	if(value == 1){
		cb.sensor_switch_cb(1);
	}else{
		cb.sensor_switch_cb(0);
	}
	is_sensor_on = value;



	return len;
}
static ssize_t read_sensor_sw(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    uint8_t sensor_sw_value = is_sensor_on; 
	// if(fw_get_state() == STATE_OPERATING){
	// 	sensor_sw_value = 1;
	// }else{
	// 	sensor_sw_value = 0;
	// }


    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_sw_value, sizeof(sensor_sw_value));
}

static ssize_t bt_write_reset(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t value = *((uint8_t *)buf);

	if (!value) {
		sys_reboot(SYS_REBOOT_COLD);
		printk("Resetting\n");
	}

	return len;
}


static ssize_t write_sensor_stream_sw(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t value = *((uint8_t *)buf);

	is_streaming = value;



	return len;
}

static ssize_t read_sensor_stream_sw(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    // bool is_streaming;
	// is_streaming = fw_get_is_streaming();
	bool _is_streaming = is_streaming;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &_is_streaming, sizeof(_is_streaming));
}


BT_GATT_SERVICE_DEFINE(
	exg_service,
	BT_GATT_PRIMARY_SERVICE(BT_UUID),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL1, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel1_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL2, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel2_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL3, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel3_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL4, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel4_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL5, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel5_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL6, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel6_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL7, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel7_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL8, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel8_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_HR, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_hr_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_SPO2, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_spo2_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_TEMP, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_temp_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_GLUCOSE, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_glucose_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),


	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_SW, BT_GATT_CHRC_WRITE|BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE|BT_GATT_PERM_READ, read_sensor_sw, write_sensor_sw, &is_sensor_on),
	BT_GATT_CHARACTERISTIC(BT_UUID_RESET, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, bt_write_reset, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_SW, BT_GATT_CHRC_WRITE|BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE|BT_GATT_PERM_READ, read_sensor_stream_sw, write_sensor_stream_sw, &is_streaming)

);

int init_ble_service(struct ble_cb *callbacks){
	cb.sensor_data_download_cb = callbacks->sensor_data_download_cb;
	cb.sensor_switch_cb = callbacks->sensor_switch_cb;
	return 0;
}

static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONNECTABLE |
	 BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
        BT_GAP_ADV_SLOW_INT_MIN,
        BT_GAP_ADV_SLOW_INT_MAX,
	NULL); /* Set to NULL for undirected advertising */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_VAL),
};

static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

	printk("Connected\n");
	ssize_t mtu = bt_gatt_get_mtu(conn);
	printk("MTU: %d\n", mtu);


}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);


}
struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};


int stream_sensor_data(enum SensorType sensor_type, uint32_t *sensor_value, ssize_t size)
{
	struct bt_gatt_attr *attr;
	switch (sensor_type) {
		case CHANNEL1:
			attr = &exg_service.attrs[1];
			if (!stream_sensor_channel1_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL2:
			attr = &exg_service.attrs[4];
			if (!stream_sensor_channel2_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL3:
			attr = &exg_service.attrs[7];
			if (!stream_sensor_channel3_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL4:
			attr = &exg_service.attrs[10];
			if (!stream_sensor_channel1_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL5:
			attr = &exg_service.attrs[13];
			if (!stream_sensor_channel2_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL6:
			attr = &exg_service.attrs[16];
			if (!stream_sensor_channel3_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL7:
			attr = &exg_service.attrs[19];
			if (!stream_sensor_channel1_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL8:
			attr = &exg_service.attrs[22];
			if (!stream_sensor_channel2_enabled) {
				return -EACCES;
			}
			break;
		case HR:
			attr = &exg_service.attrs[25];
			if (!stream_sensor_hr_enabled) {
				return -EACCES;
			}
			break;
		case SPO2:
			attr = &exg_service.attrs[28];
			if (!stream_sensor_spo2_enabled) {
				return -EACCES;
			}
			break;
		case TEMP:
			attr = &exg_service.attrs[31];
			if (!stream_sensor_temp_enabled) {
				return -EACCES;
			}
			break;
		case GLUCOSE:
			attr = &exg_service.attrs[34];
			if (!stream_sensor_glucose_enabled) {
				return -EACCES;
			}
			break;
		
		default:
			return -EINVAL;
	}
	
	return bt_gatt_notify(NULL, attr, sensor_value, size);
}



int register_ble_cb(struct ble_cb *callbacks){
	cb.sensor_switch_cb = callbacks->sensor_switch_cb;
	cb.sensor_data_download_cb = callbacks->sensor_data_download_cb;
	return 0;
};

int ble_main(void)
{	

	int err;
	// fw_initialize_system_status();
	// sys_stat_ptr = fw_get_system_status();
	init_ble_service(&cb);
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}

	bt_conn_cb_register(&connection_callbacks);

	LOG_INF("Bluetooth initialized\n");
	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return -1;
	}

	// stream_sensor_data(1);

	LOG_INF("Advertising successfully started with prioirty: %d\n", PRIO);
	return 0;
}

// K_THREAD_DEFINE(ble_t, 4096, ble_main, NULL, NULL, NULL, PRIO, 0, 0);