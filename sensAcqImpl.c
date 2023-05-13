#include "usb_hal.h"
#include "sensAcqImpl.h"

// Global variables for USB device state
static bool should_set_address = false;
static uint8_t dev_addr = 0;
static volatile bool configured = false;

// EP0 IN and OUT
static const struct usb_endpoint_descriptor ep0_out = {
    .bLength = sizeof(struct usb_endpoint_descriptor),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
    .bEndpointAddress = EP0_OUT_ADDR, // EP number 0, OUT from host (rx to device)
    .bmAttributes = USB_TRANSFER_TYPE_CONTROL,
    .wMaxPacketSize = 64,
    .bInterval = 0};

static const struct usb_endpoint_descriptor ep0_in = {
    .bLength = sizeof(struct usb_endpoint_descriptor),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
    .bEndpointAddress = EP0_IN_ADDR, // EP number 0, OUT from host (rx to device)
    .bmAttributes = USB_TRANSFER_TYPE_CONTROL,
    .wMaxPacketSize = 64,
    .bInterval = 0};

// Descriptors
static const struct usb_device_descriptor device_descriptor = {
    .bLength = sizeof(struct usb_device_descriptor),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_DEVICE,
    .bcdUSB = 0x0110,       // USB 1.1 device
    .bDeviceClass = 0,      // Specified in interface descriptor
    .bDeviceSubClass = 0,   // No subclass
    .bDeviceProtocol = 0,   // No protocol
    .bMaxPacketSize0 = 64,  // Max packet size for ep0
    .idVendor = 0x0000,     // Your vendor id
    .idProduct = 0x0001,    // Your product ID
    .bcdDevice = 0,         // No device revision number
    .iManufacturer = 1,     // Manufacturer string index
    .iProduct = 2,          // Product string index
    .iSerialNumber = 0,     // No serial number
    .bNumConfigurations = 1 // One configuration
};

static const struct usb_interface_descriptor interface_descriptor = {
    .bLength = sizeof(struct usb_interface_descriptor),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,      // Interface has 2 endpoints
    .bInterfaceClass = 0xff, // Vendor specific endpoint
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0};

static struct usb_endpoint_descriptor ep1_in = {
    .bLength = sizeof(struct usb_endpoint_descriptor),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
    .bEndpointAddress = EP1_IN_ADDR, // EP number 1, IN to host (tx to device)
    .bmAttributes = USB_TRANSFER_TYPE_ISOCHRONOUS,
    .wMaxPacketSize = 0,    // Change this in initDataAcq function
    .bInterval = 1};

static const struct usb_configuration_descriptor config_descriptor = {
    .bLength = sizeof(struct usb_configuration_descriptor),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_CONFIG,
    .wTotalLength = (sizeof(struct usb_configuration_descriptor) +
                     sizeof(struct usb_interface_descriptor) +
                     sizeof(struct usb_endpoint_descriptor)),
    .bNumInterfaces = 1,
    .bConfigurationValue = 1, // Configuration 1
    .iConfiguration = 0,      // No string
    .bmAttributes = 0x80,     // attributes: self powered, no remote wakeup
    .bMaxPower = 0x32         // 100ma
};

static const unsigned char lang_descriptor[] = {
    4,         // bLength
    0x03,      // bDescriptorType == String Descriptor
    0x09, 0x04 // language id = us english
};

static const unsigned char *descriptor_strings[] = {
    (unsigned char *)"Raspberry Pi",                  // Vendor
    (unsigned char *)"Pico Sensor Acquisition Device" // Product
};

// Bus reset handler
void handleBusReset(void);
// Setup packet handler
void handleSetupPkt(struct usb_setup_packet *pkt);
// EP handler declarations
void ep0_in_handler(uint8_t *buf, uint16_t len);
void ep0_out_handler(uint8_t *buf, uint16_t len);
void ep1_in_handler(uint8_t *buf, uint16_t len);

// Struct defining the device configuration
struct usb_device_configuration dev_config =
    {
        .device_descriptor = &device_descriptor,
        .interface_descriptor = &interface_descriptor,
        .config_descriptor = &config_descriptor,
        .lang_descriptor = lang_descriptor,
        .descriptor_strings = descriptor_strings,
        .busResetHandler = handleBusReset,
        .setupPktHandler = handleSetupPkt,
        .endpoints =
            {
                {
                    .descriptor = &ep0_out,
                    .handler = &ep0_out_handler,
                    .endpoint_control = NULL, // NA for EP0
                    .buffer_control = &usb_dpram->ep_buf_ctrl[0].out,
                    // EP0 in and out share a data buffer
                    .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                    .descriptor = &ep0_in,
                    .handler = &ep0_in_handler,
                    .endpoint_control = NULL, // NA for EP0,
                    .buffer_control = &usb_dpram->ep_buf_ctrl[0].in,
                    // EP0 in and out share a data buffer
                    .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                    .descriptor = &ep1_in,
                    .handler = &ep1_in_handler,
                    // EP1 starts at offset 0 for endpoint control
                    .endpoint_control = &usb_dpram->ep_ctrl[0].in,
                    .buffer_control = &usb_dpram->ep_buf_ctrl[1].in,
                    // First free EPX buffer
                    .data_buffer = &usb_dpram->epx_data[0 * 64],
                }}};

// Type definitions for dma control blocks
struct
{
    uint32_t nBytes;
    uint8_t *readAddr;
} __attribute__((packed)) sens_data_control_blocks[128];

struct
{
    bool *writeAddr;
    uint32_t nBytes;
} __attribute__((packed)) sens_drdy_control_blocks[128];

// Global variables for dma operations
static int usb_ep1_in_ctrl_chan, usb_ep1_in_data_chan;
static dma_channel_config usb_ep1_in_ctrl_chan_cfg, usb_ep1_in_data_chan_cfg;

// Functions related to dma operations
void usb_ep1_in_dma_setup()
{
    // ctrl_chan loads control blocks into data_chan, which executes them.
    usb_ep1_in_ctrl_chan = dma_claim_unused_channel(true);
    usb_ep1_in_data_chan = dma_claim_unused_channel(true);

    // The control channel transfers two words into the data channel's control
    // registers, then halts. The write address wraps on a two-word
    // (eight-byte) boundary, so that the control channel writes the same two
    // registers when it is next triggered. The alias to be written to for data_chan
    // will change based on what is happening. There are two types of data transfers.
    // 1. Sensor data to USB EP1 IN buffer
    // 2. 0 to sensor DRDY variable
    // For transfers to USB EP1 IN buffer, the read address will vary while the write
    // address will be incremented automatically by the data_chan. So, alias 3 will be
    // used for this purpose.
    // For transfers to DRDY variable, the write address will vary while the read address
    // will stay the same. So, alias 1 will be used for this purpose.

    usb_ep1_in_ctrl_chan_cfg = dma_channel_get_default_config(usb_ep1_in_ctrl_chan);
    channel_config_set_transfer_data_size(&usb_ep1_in_ctrl_chan_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&usb_ep1_in_ctrl_chan_cfg, true);
    channel_config_set_write_increment(&usb_ep1_in_ctrl_chan_cfg, true);
    channel_config_set_ring(&usb_ep1_in_ctrl_chan_cfg, true, 3); // 1 << 3 byte boundary on write ptr
    dma_channel_set_config(usb_ep1_in_ctrl_chan, &usb_ep1_in_ctrl_chan_cfg, false);
    dma_channel_set_trans_count(usb_ep1_in_ctrl_chan, 2, false); // Halt after each control block

    // The data channel is set up to write to the USB DRAM or to DRDY variables and then chain
    // to the control channel once it completes. The control channel programs a new read/write
    // address and data length, and retriggers the data channel. The data channel should increment
    // the write address automatically for transfers to USB EP1 IN buffer. For transfers to DRDY
    // variables, it doesn't matter since the write address is updated every time and there is only
    // one transfer that needs to be performed.

    usb_ep1_in_data_chan_cfg = dma_channel_get_default_config(usb_ep1_in_data_chan);
    channel_config_set_transfer_data_size(&usb_ep1_in_data_chan_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&usb_ep1_in_data_chan_cfg, true);
    channel_config_set_write_increment(&usb_ep1_in_data_chan_cfg, true);
    channel_config_set_chain_to(&usb_ep1_in_data_chan_cfg, usb_ep1_in_ctrl_chan); // Trigger ctrl_chan when data_chan completes
    channel_config_set_irq_quiet(&usb_ep1_in_data_chan_cfg, true);                // Raise the IRQ flag when 0 is written to a trigger register (end of chain):
}

void transfer_data_to_ep1_in_buf(volatile void *ep1_in_buff_add)
{
    channel_config_set_read_increment(&usb_ep1_in_data_chan_cfg, true);                                            // Increment the read address automatically in every transfer for data chan
    dma_channel_set_config(usb_ep1_in_data_chan, &usb_ep1_in_data_chan_cfg, false);                                // Update the configuration of data chan
    dma_channel_set_write_addr(usb_ep1_in_data_chan, ep1_in_buff_add, false);                                      // Initial write address for data chan
    dma_channel_set_write_addr(usb_ep1_in_ctrl_chan, &dma_hw->ch[usb_ep1_in_data_chan].al3_transfer_count, false); // Initial write address for ctrl chan
    dma_channel_set_read_addr(usb_ep1_in_ctrl_chan, sens_data_control_blocks, true);                               // Initial read address for ctrl chan
}

void transfer_data_to_drdy_vars()
{
    bool drdyVal = 0;
    channel_config_set_read_increment(&usb_ep1_in_data_chan_cfg, false);                                       // Don't increment read address for data chan
    dma_channel_set_config(usb_ep1_in_data_chan, &usb_ep1_in_data_chan_cfg, false);                            // Update the configuration of data chan
    dma_channel_set_read_addr(usb_ep1_in_data_chan, &drdyVal, false);                                          // Initial read address for data chan
    dma_channel_set_write_addr(usb_ep1_in_ctrl_chan, &dma_hw->ch[usb_ep1_in_data_chan].al1_write_addr, false); // Initial write address for ctrl chan
    dma_channel_set_read_addr(usb_ep1_in_ctrl_chan, sens_drdy_control_blocks, true);                           // Initial read address for ctrl chan
}

void wait_for_transfer_completion()
{
    // The data channel will assert its IRQ flag when it gets a null trigger,
    // indicating the end of the control block list. We're just going to wait
    // for the IRQ flag instead of setting up an interrupt handler.
    while (!(dma_hw->intr & 1u << usb_ep1_in_data_chan))
        tight_loop_contents();
    dma_hw->ints0 = 1u << usb_ep1_in_data_chan;
}

// Structures for holding sensor implementation information
data_cfg_t sensorList[128];

static struct
{
    uint8_t *dataAddr;
    bool *drdyAddr;
} dataPointers[128];

static uint8_t numSensors = 0;
static uint16_t usbPktSize = 0;
static uint16_t sensDataSize = 0;
static uint8_t nDRDYbytes = 0;

// Function to register a sensor so that the data can be sent
bool register_sensor(uint8_t sensType, uint8_t dataType, uint8_t nDataVals, uint16_t samplingRate, uint8_t *dataAddr, bool *drdyAddr)
{
    // Make sure that the dataType width is power of 2.
    if (((dataType & DTYPE_WIDTH_MASK) == 1) || ((dataType & DTYPE_WIDTH_MASK) == 2) || ((dataType & DTYPE_WIDTH_MASK) == 4) || ((dataType & DTYPE_WIDTH_MASK) == 8))
        // Make sure that the size of the usb packet doesn't become larger than 1023 bytes.
        // The sum below accounts for
        //   size of the total sensor data(including the one being added)
        // + 8 bytes for timestamp
        // + ceil(numSensors / 8) for drdy bits that will  be sent in the packet
        if ((sensDataSize + 8 + (dataType & DTYPE_WIDTH_MASK) * nDataVals + (numSensors + 8 - 1) / 8) < 1024 && numSensors < 128)
        {
            sensorList[numSensors].sensType = sensType;
            sensorList[numSensors].dataType = dataType;
            sensorList[numSensors].nDataVal = nDataVals;
            sensorList[numSensors].samplingRate = samplingRate;
            dataPointers[numSensors].dataAddr = dataAddr;
            dataPointers[numSensors].drdyAddr = drdyAddr;
            sensDataSize += (dataType & DTYPE_WIDTH_MASK) * nDataVals;
            ++numSensors;
            return true;
        }
        else
            return false;
    else
        return false;
}

// Function to initialize the USB flight controller communication functionality
bool initDataAcq()
{
    // Although this should not happen, make sure that the size of the usb packet doesn't become larger than 1023 bytes.
    // The sum below accounts for
    //   size of the total sensor data
    // + 8 bytes for timestamp
    // + ceil(numSensors / 8) for drdy bits that will  be sent in the packet
    if ((sensDataSize + 8 + (numSensors + 8 - 1) / 8) < 1024)
    {
        // Define the wMaxPacketSize for ep1_in
        ep1_in.wMaxPacketSize = (sensDataSize + 8 + (numSensors + 8 - 1) / 8);

        // Update the number of data ready bytes required so that other data can be arranged accordingly
        nDRDYbytes = (numSensors + 8 - 1) / 8;

        // Update the data_buffer for other endpoints to acommodate ep1_in's max packet
        // will be done later for ep1_out

        // Setup dma channels for data transfers to USB ram
        usb_ep1_in_dma_setup();

        // Setup USB peripheral and get going
        usb_device_init();

        // Wait until USB is configured
        while (!configured)
        {
            tight_loop_contents();
        }

        // Setup an initial transfer so that the process starts
        struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP1_IN_ADDR);
        ep1_in_handler((uint8_t *)ep->data_buffer, ep1_in.wMaxPacketSize);

        return true;
    }
    else
        return false;
}

void handleBusReset(void)
{
    // Set address back to 0
    dev_addr = 0;
    should_set_address = false;
    usb_set_dev_addr(dev_addr);
    configured = false;
}

void handleSetupPkt(struct usb_setup_packet *pkt)
{
    // Reset PID to 1 for EP0 IN
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR); // We'll probably need EP0 in
    ep->next_pid = 1u;

    if (pkt->bmRequestType == (USB_REQ_TYPE_DIR_OUT | USB_REQ_TYPE_TYPE_STANDARD | USB_REQ_TYPE_RECIPIENT_DEVICE))
    {
        if (pkt->bRequest == USB_REQUEST_SET_ADDRESS)
        {
            // Set address is a bit of a strange case because we have to send a 0 length status packet first with
            // address 0
            dev_addr = (pkt->wValue & 0xff);
            // printf("Set address %d\r\n", dev_addr);
            // Will set address in the callback phase
            should_set_address = true;
            usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0); // Acknowledge OUT request
        }
        else if (pkt->bRequest == USB_REQUEST_SET_CONFIGURATION)
        {
            // Only one configuration so just acknowledge the request
            usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0); // USB enumeration finishes here
            configured = true;
        }
        else
        {
            // Unknown OUT request
            usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0); // Acknowledge OUT request
        }
    }
    else if (pkt->bmRequestType == (USB_REQ_TYPE_DIR_IN | USB_REQ_TYPE_TYPE_STANDARD | USB_REQ_TYPE_RECIPIENT_DEVICE))
    {
        if (pkt->bRequest == USB_REQUEST_GET_DESCRIPTOR)
        {
            uint16_t descriptor_type = pkt->wValue >> 8;
            if (descriptor_type == USB_DESCRIPTOR_TYPE_DEVICE)
            {
                ep->next_pid = 1;                                                       // Always respond with pid 1
                uint16_t len = MIN(sizeof(struct usb_device_descriptor), pkt->wLength); // Calculate the amount of data
                memcpy((void *)ep->data_buffer, (void *)dev_config.device_descriptor, len); // Copy the data to the ep0 in data buffer
                usb_start_transfer(ep, NULL, len); // Make ep0 in ready for transfer
            }
            else if (descriptor_type == USB_DESCRIPTOR_TYPE_CONFIG)
            {
                volatile uint8_t *buf = ep->data_buffer;

                // First request will want just the config descriptor
                memcpy((void *)buf, dev_config.config_descriptor, sizeof(struct usb_configuration_descriptor));
                buf += sizeof(struct usb_configuration_descriptor);

                // If we need more than just the config descriptor copy it all
                if (pkt->wLength >= dev_config.config_descriptor->wTotalLength)
                {
                    memcpy((void *)buf, dev_config.interface_descriptor, sizeof(struct usb_interface_descriptor));
                    buf += sizeof(struct usb_interface_descriptor);

                    // Copy all the endpoint descriptors starting from EP1
                    for (uint i = 2; i < USB_NUM_ENDPOINTS; i++)
                    {
                        if (dev_config.endpoints[i].descriptor)
                        {
                            memcpy((void *)buf, dev_config.endpoints[i].descriptor, sizeof(struct usb_endpoint_descriptor));
                            buf += sizeof(struct usb_endpoint_descriptor);
                        }
                    }
                }

                // Send data
                // Get len by working out end of buffer subtract start of buffer
                uint32_t len = (uint32_t)buf - (uint32_t)ep->data_buffer;
                usb_start_transfer(ep, NULL, MIN(len, pkt->wLength));
            }
            else if (descriptor_type == USB_DESCRIPTOR_TYPE_STRING)
            {
                uint8_t i = pkt->wValue & 0xff;
                uint8_t len = 0;

                if (i == 0)
                {
                    len = 4;
                    memcpy((void *)ep->data_buffer, dev_config.lang_descriptor, len);
                }
                else
                {
                    // Prepare fills in ep0_buf
                    const unsigned char *str = dev_config.descriptor_strings[i - 1];
                    // 2 for bLength + bDescriptorType + strlen * 2 because string is unicode. i.e. other byte will be 0
                    len = 2 + (strlen((const char *)str) * 2);
                    static const uint8_t bDescriptorType = 0x03;

                    volatile uint8_t *buf = ep->data_buffer;
                    *buf++ = len;
                    *buf++ = bDescriptorType;

                    uint8_t c;

                    do
                    {
                        c = *str++;
                        *buf++ = c;
                        *buf++ = 0;
                    } while (c != '\0');
                }

                usb_start_transfer(ep, NULL, MIN(len, pkt->wLength));
            }
            else
            {
                // printf("Unhandled GET_DESCRIPTOR type 0x%x\r\n", descriptor_type);
            }
        }
        else
        {
            // printf("Other IN request (0x%x)\r\n", pkt->bRequest);
        }
    }
}

void ep0_in_handler(uint8_t *buf, uint16_t len)
{
    if (should_set_address)
    {
        // Set actual device address in hardware
        usb_set_dev_addr(dev_addr);
        should_set_address = false;
    }
    else
    {
        // Receive a zero length status packet from the host on EP0 OUT
        struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_OUT_ADDR);
        usb_start_transfer(ep, NULL, 0);
    }
}

void ep0_out_handler(uint8_t *buf, uint16_t len)
{
    ;
}

// Device specific functions
void ep1_in_handler(uint8_t *buf, uint16_t len)
{
    // Send data back to host
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP1_IN_ADDR);

    // Set length of the packet to zero here and keep track of the amount of data to be sent
    len = 0;

    // Update the drdy bits in the ep1_in buffer
    for (uint8_t i = 0; i < numSensors; ++i)
        if (*dataPointers[i].drdyAddr)
            *(buf + i / 8) |= (1 << (i % 8));
        else
            *(buf + i / 8) &= ~(1 << (i % 8));

    // Update buf to point to the time value, and add nDRDYbytes to len
    buf += nDRDYbytes;
    len += nDRDYbytes;

    // Copy current time to the USB ram
    uint64_t time = time_us_64();
    memcpy((void *)buf, (void *)&time, sizeof(time));

    // Update buf to point to the start of sensor data region, and add 8 bytes to len
    buf += 8;
    len += 8;

    // Setup the sensor data control blocks for the new sensor data that has not been sent yet.
    uint8_t ctrlBlckLoc = 0;
    for (uint8_t i = 0; i < numSensors; ++i)
    {
        // Check, from the ep1_in buffer, if the sensor data was ready
        if (ep->data_buffer[i / 8] & (1 << (i % 8)))
        {
            // Add an entry for this sensor in the sensor data control block array
            sens_data_control_blocks[ctrlBlckLoc].nBytes = (sensorList[i].dataType & DTYPE_WIDTH_MASK) * sensorList[i].nDataVal;
            sens_data_control_blocks[ctrlBlckLoc++].readAddr = dataPointers[i].dataAddr;
            // Add number of bytes being sent for this sensor to len
            len += sens_data_control_blocks[ctrlBlckLoc - 1].nBytes;
        }
    }
    // Add a null control block at the end to terminate the dma transfers
    sens_data_control_blocks[ctrlBlckLoc].nBytes = 0;
    sens_data_control_blocks[ctrlBlckLoc].readAddr = NULL;
    // Initiate dma transfers for sensor data at the location after the 64-bit time value
    transfer_data_to_ep1_in_buf(buf);

    // Setup the sensor data ready control blocks to indicate that the sensor data has been read
    ctrlBlckLoc = 0;
    for (uint8_t i = 0; i < numSensors; ++i)
    {
        // Check, from the ep1_in buffer, if the sensor data was ready
        if (ep->data_buffer[i / 8] & (1 << (i % 8)))
        {
            sens_drdy_control_blocks[ctrlBlckLoc].writeAddr = dataPointers[i].drdyAddr;
            sens_drdy_control_blocks[ctrlBlckLoc++].nBytes = 1;
        }
    }
    // Add a null control block to terminate the dma transfers
    sens_drdy_control_blocks[ctrlBlckLoc].writeAddr = NULL;
    sens_drdy_control_blocks[ctrlBlckLoc].nBytes = 0;
    // Wait for sensor data to be copied to the USB ram
    wait_for_transfer_completion();
    // Initiate dma transfers for drdy variables
    transfer_data_to_drdy_vars();

    // Let USB peripheral know that data is ready for being sent to the host.
    // This probably, guessing, can be done right after the dma transfers for sensor data is initiated
    // assuming that the dma controller can copy data faster than the USB peripehral
    // can read the ram and transmit the data to the host.
    usb_start_transfer(ep, NULL, len);
}