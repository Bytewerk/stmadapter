#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include "systime.h"

#define USB_TX_BUF_SIZE 128
uint16_t usb_tx_buf[USB_TX_BUF_SIZE];
int usb_tx_buf_used = 0;


static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
} };

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
} };

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 }
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
} };

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
} };

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
} };

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char * usb_strings[] = {
	"bytewerk Technologies",
	"MDB USB Adapter",
	"42",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static int cdcacm_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		return 1;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding)) {
			return 0;
		}

		return 1;
	}
	return 0;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	uint8_t buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

	for (int i=0; i<(len/2); i++) {
		uint16_t val9bit = buf[2*i];
		val9bit <<= 8;
		val9bit |= (uint16_t)buf[2*i+1];
		usart_send_blocking(USART2, val9bit);
		while ((USART_SR(USART2) & USART_SR_TC) == 0);
	}

}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);

	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

void usart2_isr(void)
{

	while (usart_get_flag(USART2, USART_SR_RXNE)) {

		if (usb_tx_buf_used < USB_TX_BUF_SIZE) { // txbuf has space left?
			uint16_t ch = usart_recv(USART2) & 0x1FF;
			usb_tx_buf[usb_tx_buf_used++] = ((ch & 0xFF)<<8) | ((ch>>8) & 0xFF);
		} else {
			usart_recv(USART2);
		}
	}

}

int main(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_120MHZ]);

	systime_setup(120000);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_USART2);

	/* Setup GPIO pin GPIO12 on GPIO port D for LED. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);

	// UART2 TX
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);

	// UART2 RX
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO3);

	// UART3 TX
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	gpio_set_af(GPIOB, GPIO_AF7, GPIO10);

	// UART3 RX
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11);
	gpio_set_af(GPIOB, GPIO_AF7, GPIO11);



	/* Setup USART2 parameters. */
	usart_set_baudrate(USART2, 9600);
	usart_set_databits(USART2, 9);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	nvic_enable_irq(NVIC_USART2_IRQ);
	usart_enable_rx_interrupt(USART2);
	usart_enable(USART2);

	/* init usb */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);

	usbd_device *usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 3,usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

	while (1) {

		usart_disable_rx_interrupt(USART2);
		if (usb_tx_buf_used>0) {
			usbd_ep_write_packet(usbd_dev, 0x82, usb_tx_buf, usb_tx_buf_used*2);
			usb_tx_buf_used = 0;
		}
		usart_enable_rx_interrupt(USART2);

		usbd_poll(usbd_dev);
	}

}
