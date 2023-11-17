#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

void main(void)
{
    const struct device *dev = device_get_binding("LSM9DS1_MAGN");

    k_sleep(K_SECONDS(5));

    if (!dev)
    {
        printk("Failed to get device binding for LSM9DS1_MAGN");
        return;
    }

    struct sensor_value magn[3];
    while (1)
    {
        // Fetching sensor samples
        if (sensor_sample_fetch(dev) < 0)
        {
            printk("Sensor sample update problem\n");
            continue;
        }

        // Get sensor values
        struct sensor_value magn_x, magn_y, magn_z;
        if (sensor_channel_get(dev, SENSOR_CHAN_MAGN_X, &magn_x) < 0 ||
            sensor_channel_get(dev, SENSOR_CHAN_MAGN_Y, &magn_y) < 0 ||
            sensor_channel_get(dev, SENSOR_CHAN_MAGN_Z, &magn_z) < 0)
        {
            printk("Failed to get sensor values");
            return;
        }

        // Print the sensor values
        printk("Magnetometer X: %d.%06d, Y: %d.%06d, Z: %d.%06d\n",
               magn_x.val1, magn_x.val2, magn_y.val1, magn_y.val2, magn_z.val1, magn_z.val2);

        k_sleep(K_MSEC(1000));
    }

    return;
}
