#include <stm32f1xx.h>
const uint8_t DeviceDescriptor[] = {
    0x12,       // Length
    0x01,       // DescriptorType (Device)
    0x00, 0x02, // bcdUSB (2.0)
    0x02,       // DeviceClass (CDC)
    0x00,       // DeviceSubClass
    0x00,       // DeviceProtocol
    0x40,       // bMaxPacketSize0
    0x83, 0x04, // idVendor (STMicroelectronics)
    0x40, 0x57, // idProduct
    0x00, 0x02, // bcdDevice
    0x01,       // iManufacturer
    0x02,       // iProduct
    0x03,       // iSerialNumber
    0x01        // bNumConfigurations
};

const uint8_t ConfigurationDescriptor[] = {
    0x09,       // Length
    0x02,       // DescriptorType (Configuration)
    0x43, 0x00, // wTotalLength
    0x02,       // bNumInterfaces
    0x01,       // bConfigurationValue
    0x00,       // iConfiguration
    0xC0,       // bmAttributes
    0x32,       // bMaxPower (100mA)

    // Interface Descriptor (CDC Control)
    0x09, // Length
    0x04, // DescriptorType (Interface)
    0x00, // bInterfaceNumber
    0x00, // bAlternateSetting
    0x01, // bNumEndpoints
    0x02, // bInterfaceClass (CDC)
    0x02, // bInterfaceSubClass (ACM)
    0x01, // bInterfaceProtocol (AT commands)
    0x00, // iInterface

    // CDC Header Functional Descriptor
    0x05,       // Length
    0x24,       // DescriptorType (CS_INTERFACE)
    0x00,       // Descriptor SubType (Header Functional Descriptor)
    0x10, 0x01, // bcdCDC (1.10)

    // CDC Call Management Functional Descriptor
    0x05, // Length
    0x24, // DescriptorType (CS_INTERFACE)
    0x01, // Descriptor SubType (Call Management Functional Descriptor)
    0x00, // bmCapabilities
    0x01, // bDataInterface

    // CDC ACM Functional Descriptor
    0x04, // Length
    0x24, // DescriptorType (CS_INTERFACE)
    0x02, // Descriptor SubType (Abstract Control Management Descriptor)
    0x02, // bmCapabilities

    // CDC Union Functional Descriptor
    0x05, // Length
    0x24, // DescriptorType (CS_INTERFACE)
    0x06, // Descriptor SubType (Union Functional Descriptor)
    0x00, // bMasterInterface
    0x01, // bSlaveInterface0

    // Endpoint Descriptor (Interrupt)
    0x07,       // Length
    0x05,       // DescriptorType (Endpoint)
    0x81,       // bEndpointAddress (IN endpoint 1)
    0x03,       // bmAttributes (Interrupt)
    0x08, 0x00, // wMaxPacketSize
    0xFF,       // bInterval

    // Interface Descriptor (CDC Data)
    0x09, // Length
    0x04, // DescriptorType (Interface)
    0x01, // bInterfaceNumber
    0x00, // bAlternateSetting
    0x02, // bNumEndpoints
    0x0A, // bInterfaceClass (CDC Data)
    0x00, // bInterfaceSubClass
    0x00, // bInterfaceProtocol
    0x00, // iInterface

    // Endpoint Descriptor (Bulk IN)
    0x07,       // Length
    0x05,       // DescriptorType (Endpoint)
    0x82,       // bEndpointAddress (IN endpoint 2)
    0x02,       // bmAttributes (Bulk)
    0x40, 0x00, // wMaxPacketSize
    0x00,       // bInterval

    // Endpoint Descriptor (Bulk OUT)
    0x07,       // Length
    0x05,       // DescriptorType (Endpoint)
    0x02,       // bEndpointAddress (OUT endpoint 2)
    0x02,       // bmAttributes (Bulk)
    0x40, 0x00, // wMaxPacketSize
    0x00        // bInterval
};

const uint8_t StringDescriptor[] = {
    0x04,      // Length
    0x03,      // DescriptorType (String)
    0x09, 0x04 // LANGID (English)
};

const uint8_t StringDescriptorManufacturer[] = {
    0x1A, // Length
    0x03, // DescriptorType (String)
    'S',  0x00, 'T', 0x00, 'M', 0x00, 'i', 0x00, 'c', 0x00, 'r', 0x00,
    'o',  0x00, 'e', 0x00, 'l', 0x00, 'e', 0x00, 'c', 0x00, 't', 0x00,
    'r',  0x00, 'o', 0x00, 'n', 0x00, 'i', 0x00, 'c', 0x00, 's', 0x00};

const uint8_t StringDescriptorProduct[] = {
    0x16, // Length
    0x03, // DescriptorType (String)
    'V',  0x00, 'i',  0x00, 'r',  0x00, 't',  0x00, 'u',  0x00, 'a',
    0x00, 'l',  0x00, ' ',  0x00, 'C',  0x00, 'O',  0x00, 'M',  0x00,
    ' ',  0x00, 'P',  0x00, 'o',  0x00, 'r',  0x00, 't',  0x00};

const uint8_t StringDescriptorSerial[] = {
    0x1A, // Length
    0x03, // DescriptorType (String)
    '0',  0x00, '0', 0x00, '1', 0x00, '2', 0x00, '3', 0x00, '4', 0x00,
    '5',  0x00, '6', 0x00, '7', 0x00, '8', 0x00, '9', 0x00, 'A', 0x00,
    'B',  0x00, 'C', 0x00, 'D', 0x00, 'E', 0x00, 'F', 0x00};
