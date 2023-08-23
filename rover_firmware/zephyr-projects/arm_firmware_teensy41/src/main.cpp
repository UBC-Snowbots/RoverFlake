

#include <armFirmware.h>

LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);

#define RING_BUF_SIZE 1024
uint8_t ring_buffer[RING_BUF_SIZE];


struct ring_buf ringbuf;
Axis axes[NUM_AXES];
struct k_timer Axis::stepper_timer;



#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
USBD_CONFIGURATION_DEFINE(config_1,
			  USB_SCD_SELF_POWERED,
			  200);

USBD_DESC_LANG_DEFINE(sample_lang);
USBD_DESC_MANUFACTURER_DEFINE(sample_mfr, "UBC ROVER");
USBD_DESC_PRODUCT_DEFINE(sample_product, "Z USBD CDC ACM");
USBD_DESC_SERIAL_NUMBER_DEFINE(sample_sn, "0123456789ABCDEF");

USBD_DEVICE_DEFINE(sample_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   0x2fe3, 0x0001);




static int enable_usb_device_next(void)
{
	int err;

	err = usbd_add_descriptor(&sample_usbd, &sample_lang);
	if (err) {
		LOG_ERR("Failed to initialize language descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_mfr);
	if (err) {
		LOG_ERR("Failed to initialize manufacturer descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_product);
	if (err) {
		LOG_ERR("Failed to initialize product descriptor (%d)", err);
		return err;
	}

	err = usbd_add_descriptor(&sample_usbd, &sample_sn);
	if (err) {
		LOG_ERR("Failed to initialize SN descriptor (%d)", err);
		return err;
	}

	err = usbd_add_configuration(&sample_usbd, &config_1);
	if (err) {
		LOG_ERR("Failed to add configuration (%d)", err);
		return err;
	}

	err = usbd_register_class(&sample_usbd, "cdc_acm_0", 1);
	if (err) {
		LOG_ERR("Failed to register CDC ACM class (%d)", err);
		return err;
	}

	err = usbd_init(&sample_usbd);
	if (err) {
		LOG_ERR("Failed to initialize device support");
		return err;
	}

	err = usbd_enable(&sample_usbd);
	if (err) {
		LOG_ERR("Failed to enable device support");
		return err;
	}

	LOG_DBG("USB device support enabled");

	return 0;
}
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK_NEXT) */

static void interrupt_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			int recv_len, rb_len;
			uint8_t rx_buffer[RX_BUF_SIZE];

			size_t len = MIN(ring_buf_space_get(&ringbuf),
					 sizeof(rx_buffer));
			recv_len = uart_fifo_read(dev, rx_buffer, len);

			//uint8_t tx_buffer[] = "Msg Recieved";

			if (recv_len < 0) {
				// LOG_ERR("Failed to read UART FIFO");
				recv_len = 0;
			};

			// rb_len = ring_buf_put(&ringbuf, tx_buffer, sizeof(tx_buffer));
			// if (rb_len < recv_len) {
			// 	// LOG_ERR("Drop %u bytes", recv_len - rb_len);
			// }

			// LOG_DBG("tty fifo -> ringbuf %d bytes", rb_len);
			// if (rb_len) {
			// 	uart_irq_tx_enable(dev);
			// }
			parseCmd(rx_buffer);
			
		}

		if (uart_irq_tx_ready(dev)) {
			uint8_t buffer[TX_BUF_SIZE];
			int rb_len, send_len;

			rb_len = ring_buf_get(&ringbuf, buffer, sizeof(buffer));
			if (!rb_len) {
				// LOG_DBG("Ring buffer empty, disable TX IRQ");
				uart_irq_tx_disable(dev);
				continue;
			}

			send_len = uart_fifo_fill(dev, buffer, rb_len);
			if (send_len < rb_len) {
				// LOG_ERR("Drop %d bytes", rb_len - send_len);
			}

			// LOG_DBG("ringbuf -> tty fifo %d bytes", send_len);
		}
	}
}
void sendMsg(const char tx_msg[TX_BUF_SIZE]){

	ring_buf_put(&ringbuf, (uint8_t *)tx_msg, strlen(tx_msg));
	uart_irq_tx_enable(dev);
}


void parseCmd(uint8_t cmd[RX_BUF_SIZE]){
	int arpo = 2;
		 if(cmd[0] == 'q'){

	char temp_msg[TX_BUF_SIZE] = "up\n\r\0";
	axes[arpo].speed = 1500;
	axes[arpo].macrostep = 8;
	axes[arpo].step_des_pos = axes[arpo].step_pos + axes[arpo].macrostep;
	stepAxis(arpo);
	sendMsg(temp_msg);
	//	k_sleep(K_MSEC(50));
	//stepAxis(1);

	}

	 if(cmd[0] == 'w'){

	char temp_msg[TX_BUF_SIZE] = "up\n\r\0";
	axes[arpo].speed = 800;
	axes[arpo].macrostep = 15;
	axes[arpo].step_des_pos = axes[arpo].step_pos + axes[arpo].macrostep;
//		set_gpio(4, 8, 0);

	stepAxis(arpo);

	sendMsg(temp_msg);
	//	k_sleep(K_MSEC(50));
	//stepAxis(1);

	}
		 if(cmd[0] == 'f'){

	char temp_msg[TX_BUF_SIZE] = "up\n\r\0";
	axes[arpo].speed = 500;
	axes[arpo].macrostep = 25;
	axes[arpo].step_des_pos = axes[arpo].step_pos + axes[arpo].macrostep;
	stepAxis(arpo);
	sendMsg(temp_msg);
	//	k_sleep(K_MSEC(50));
	//stepAxis(1);

	}

	if(cmd[0] == 'r'){
	char temp_msg[TX_BUF_SIZE] = "down\n\r\0";
		axes[arpo].speed = 1000;
	axes[arpo].macrostep = 12;
	axes[arpo].step_des_pos = axes[arpo].step_pos - axes[arpo].macrostep;
//    set_gpio(4, 8, 1);

	stepAxis(arpo);

	sendMsg(temp_msg);
	//	k_sleep(K_MSEC(50));

	//stepAxis(1);


	}
	// uint8_t tx_buffer[] = "msgrecievs\n";
	// ring_buf_put(&ringbuf, tx_buffer, sizeof(tx_buffer));
	// uart_irq_tx_enable(dev);

	


}

void stepAxis(int axis) {

	if(axes[axis].step_des_pos > axes[axis].step_pos){
		axes[axis].dir = 0;
	} else{
		axes[axis].dir = 1;

	}
    set_gpio(axes[axis].DIR_PIN[0], axes[axis].DIR_PIN[1], axes[axis].dir);


	axes[axis].steps_remaining = axes[axis].macrostep;//abs(axes[axis].step_des_pos - axes[axis].step_pos);



  if(!axes[axis].moving){
	k_timer_start(&axes[axis].stepper_timer, K_NO_WAIT, K_NO_WAIT);
  }
  //axes[axis].step_des_pos += steps;
  //axes[axis].steps_remaining = 100;//abs(axes[axis].step_pos - axes[axis].step_des_pos);

  // Code to move the motor steps
  



  
}

void set_gpio(int dev, int pin, int value){
	const struct device *tempdev;
	switch (dev)
	{
	case 1:
		tempdev = gpio1_dev;
		break;
	
	case 2:
		tempdev = gpio2_dev;

		break;
	case 4:
		tempdev = gpio2_dev;
		break;
	default:
		break;
	}

	gpio_pin_set(tempdev, pin, value);
	
}

void stepper_timer_callback(struct k_timer *timer_id) {
	// 	char debug[TX_BUF_SIZE];
	// 	sprintf(debug, "Axis timer triggered \n\r\0");

		
	//  	ring_buf_put(&ringbuf, (uint8_t *)debug, TX_BUF_SIZE);

	// // //memcpy((uint8_t)axmsg, tx_bufferr, 64);
	
	//  uart_irq_tx_enable(dev); // Enable the TX interrupt to start sending
     
	for (int i = 0; i < NUM_AXES; i++){
	if(timer_id == &axes[i].stepper_timer){
		// axes[i].pulse_state = false;
		axes[i].moving = true;
    if (axes[i].steps_remaining > 0) {
        if (axes[i].pulse_state) {
            // End of pulse, prepare for the next one

            set_gpio(axes[i].STEP_PIN[0], axes[i].STEP_PIN[1], 0);
            axes[i].steps_remaining--;   
			if(axes[i].step_des_pos > axes[i].step_pos){
				axes[i].step_pos++;
			} else{
				axes[i].step_pos--;
			}
	 		char movemsg[TX_BUF_SIZE];
		sprintf(movemsg, "Axis %d moving from: %d, to: %d \n\r\0", i + 1, axes[i].step_pos, axes[i].step_des_pos);

		
	 	ring_buf_put(&ringbuf, (uint8_t *)movemsg, TX_BUF_SIZE);

	
	 uart_irq_tx_enable(dev); // Enable the TX interrupt to start sending
     
  
        } else {
            // Start of pulse
            set_gpio(axes[i].STEP_PIN[0], axes[i].STEP_PIN[1], 1);
        }

        // Toggle pulse state
        axes[i].pulse_state = !axes[i].pulse_state;

        // Reset the timer
    k_timer_start(&axes[i].stepper_timer, K_USEC(axes[i].speed), K_NO_WAIT);
    } else {
        // No steps remaining, stop the timer
				char movemsg[TX_BUF_SIZE];
			sprintf(movemsg, "Axis %d Done moving to: %d \n\r\0", i + 1, axes[i].step_pos);

		
		ring_buf_put(&ringbuf, (uint8_t *)movemsg, TX_BUF_SIZE);

	//memcpy((uint8_t)axmsg, tx_bufferr, 64);
	
	uart_irq_tx_enable(dev); // Enable the TX interrupt to start sending
     
		axes[i].moving = false;
        k_timer_stop(&axes[i].stepper_timer);
       // printk("destination approached");

    }
	}

	}
}




int main(void)
{
	uint32_t baudrate, dtr = 0U;
	int ret;
	gpio1_dev = DEVICE_DT_GET(DT_NODELABEL(gpio1));
 	gpio2_dev = DEVICE_DT_GET(DT_NODELABEL(gpio2));
 	gpio4_dev = DEVICE_DT_GET(DT_NODELABEL(gpio4));
	dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(dev)) {
		// LOG_ERR("CDC ACM device not ready");
		return 0;
	}

#if defined(CONFIG_USB_DEVICE_STACK_NEXT)
		ret = enable_usb_device_next();
#else
		ret = usb_enable(NULL);
#endif

	if (ret != 0) {
		// LOG_ERR("Failed to enable USB");
		return 0;
	}

	ring_buf_init(&ringbuf, sizeof(ring_buffer), ring_buffer);

	// LOG_INF("Wait for DTR");

	while (true) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		if (dtr) {
			break;
		} else {
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
	}

	// LOG_INF("DTR set");

	/* They are optional, we use them to test the interrupt endpoint */
	ret = uart_line_ctrl_set(dev, UART_LINE_CTRL_DCD, 1);
	if (ret) {
		// LOG_WRN("Failed to set DCD, ret code %d", ret);
	}

	ret = uart_line_ctrl_set(dev, UART_LINE_CTRL_DSR, 1);
	if (ret) {
		// LOG_WRN("Failed to set DSR, ret code %d", ret);
	}

	/* Wait 100ms for the host to do all settings */
	k_msleep(100);

	ret = uart_line_ctrl_get(dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
	if (ret) {
		// LOG_WRN("Failed to get baudrate, ret code %d", ret);
	} else {
		// LOG_INF("Baudrate detected: %d", baudrate);
	}

	uart_irq_callback_set(dev, interrupt_handler);

	/* Enable rx interrupts */
	uart_irq_rx_enable(dev);

	k_sleep(K_MSEC(50));


// //initiate axes, im sure there is a better way to orginize all this data
for (int i = 0; i < NUM_AXES; i++){
    axes[i].STEP_PIN[0] = stepGPIO_PIN[i][0];
	axes[i].STEP_PIN[1] = stepGPIO_PIN[i][1];
	axes[i].DIR_PIN[0] = dirGPIO_PIN[i][0];
	axes[i].DIR_PIN[1] = dirGPIO_PIN[i][1];

    axes[i].ENC_PIN_A = encPinA[i];
    axes[i].ENC_PIN_B = encPinB[i];

	axes[i].LIMIT_PIN[0] = limGPIO_PIN[i][0];
	axes[i].LIMIT_PIN[1] = limGPIO_PIN[i][1];


    axes[i].PPR = ppr[i];
    axes[i].REDUCTION = red[i];
    axes[i].steps_remaining = 0;
    axes[i].angle = 0.0;
	axes[i].steps_remaining = 0;
	axes[i].step_pos = 0;
	axes[i].pulse_state = false;
	axes[i].moving = false;
	axes[i].speed = 800;
	axes[i].macrostep = 10;
	axes[i].dir = 0;
	axes[i].attach(); //must be called after pins declared
    // gpio_pin_configure(gpio_dev, axes[i].STEP_PIN, GPIO_OUTPUT);
	// gpio_pin_configure(gpio_dev, axes[i].DIR_PIN, GPIO_OUTPUT);

	k_timer_init(&axes[i].stepper_timer, stepper_timer_callback, NULL);

	k_sleep(K_MSEC(50));

	char axmsg[TX_BUF_SIZE];

	sprintf(axmsg, "Axis %d attached at STEP: %d.%d, DIR: %d.%d, \n\r\0", i + 1, axes[i].STEP_PIN[0], axes[i].STEP_PIN[1], axes[i].DIR_PIN[0], axes[i].DIR_PIN[1]);


	//memcpy((uint8_t)axmsg, tx_bufferr, 64);
	
	ring_buf_put(&ringbuf, (uint8_t *)axmsg, strlen(axmsg));

	uart_irq_tx_enable(dev); // Enable the TX interrupt to start sending
	k_sleep(K_MSEC(50));

    //char buffer[100];
    // sprintf(buffer, "Axis %d attached at STEP: %d, DIR: %d\n", i, axes[i].STEP_PIN, axes[i].DIR_PIN);
 //   print_uart(buffer);
}
 


// while (true) {
// 	//check serial for msgs
//     // uint8_t rx_buffer[64];
//     // int rb_len = ring_buf_get(&ringbuf, rx_buffer, sizeof(rx_buffer));
//     // if (rb_len) {
//     //     // Process the received data here
// 	// 	// uint8_t data_to_send[] = "MSG RECIEVED";
// 	// 	// ring_buf_put(&ringbuf, data_to_send, sizeof(data_to_send) - 1);
// 	// 	// uart_irq_tx_enable(dev); // Enable the TX interrupt to start sending

//     //     // buffer contains rb_len bytes of data
//     // }
//     // Sleep or yield if needed
// 		//k_sleep(K_MSEC(500));

// 		//stepAxis(0);

// }





	return 0;
}


