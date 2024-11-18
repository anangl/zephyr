#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/mspi.h>

#define MSPI_NODE DT_NODELABEL(exmif)
#define FLASH_NODE DT_NODELABEL(mx25uw63)

static int test3(const struct device *mspi_dev)
{
	const struct mspi_dev_id dev_id = MSPI_DEVICE_ID_DT(FLASH_NODE);
	uint8_t buf[128] = {0};
	int rc;

	struct mspi_dev_cfg dev_cfg = MSPI_DEVICE_CONFIG_DT(FLASH_NODE);
	struct mspi_xfer_packet packet = {
		.data_buf = buf,
	};
	struct mspi_xfer xfer = {
		.xfer_mode   = MSPI_PIO,
		.packets     = &packet,
		.num_packet  = 1,
		.timeout     = 10,
	};

	rc = mspi_dev_config(mspi_dev, &dev_id,
			     MSPI_DEVICE_CONFIG_ALL, &dev_cfg);
	if (rc < 0) {
		printk("mspi_dev_config() failed: %d\n", rc);
		return rc;
	}

	for (int i = 0; i < sizeof(buf); ++i) {
		buf[i] = (uint8_t)i;
	}

	packet.dir = MSPI_TX;
	packet.cmd = 0x00FF;
	packet.num_bytes = sizeof(buf);
	xfer.cmd_length = 2;
	xfer.addr_length = 0;
	xfer.tx_dummy = 0;
	rc = mspi_transceive(mspi_dev, &dev_id, &xfer);
	if (rc < 0) {
		printk("mspi_transceive() failed: %d\n", rc);
		return rc;
	}

	return 0;
}

int main(void)
{
	const struct device *mspi_dev = DEVICE_DT_GET(MSPI_NODE);
	const struct device *flash_dev = DEVICE_DT_GET(FLASH_NODE);
	int rc;

	printk("EXMIF test on %s\n", CONFIG_BOARD_TARGET);

	if (!device_is_ready(mspi_dev)) {
		printk("%s: device not ready\n", mspi_dev->name);
		return -1;
	}

	if (!device_is_ready(flash_dev)) {
		printk("%s: device not ready\n", flash_dev->name);
		return -1;
	}

	if (0) {
		rc = test3(mspi_dev);
		if (rc < 0) {
			return -1;
		}
	}

	// uint8_t jedec_id[3] = {0};
	// rc = flash_read_jedec_id(flash_dev, jedec_id);
	// if (rc == 0) {
	// 	printk("jedec-id = [%02x %02x %02x];\n",
	// 	       jedec_id[0], jedec_id[1], jedec_id[2]);
	// } else {
	// 	printk("flash_read_jedec_id() failed: %d\n", rc);
	// }

	uint8_t buf[256];

	rc = flash_read(flash_dev, 0, buf, sizeof(buf));
	if (rc < 0) {
		printk("flash_read() failed: %d\n", rc);
		return -1;
	}

	// for (int i = 0; i < sizeof(buf); ++i) {
	// 	buf[i] = (uint8_t)i;
	// }

	// rc = flash_write(flash_dev, 0, buf, sizeof(buf));
	// if (rc < 0) {
	// 	printk("flash_write() failed: %d\n", rc);
	// 	return -1;
	// }

	extern void test_xip(const struct device *dev);
	test_xip(mspi_dev);

	// rc = flash_read(flash_dev, 0, buf, sizeof(buf));
	// if (rc < 0) {
	// 	printk("flash_read() failed: %d\n", rc);
	// 	return -1;
	// }

	return 0;
}
