// RocketComm.c

#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>

#include "lusb0_usb.h"


// Define DEBUG_BYTES to dump raw bytes received from the device
#define DEBUG_BYTES


#define USB_STATUS				0
#define USB_PAGE_DUMP			1
#define USB_READ_ALTIMETER		2
#define USB_FLASH_STATUS		3
#define USB_CLEAR_MEMORY		4
#define USB_FLASH_WRITE_TEST	5
#define USB_FLASH_READ_TEST		6
#define USB_IS_FLASH_BUSY		7
#define USB_TEST				8
#define USB_DATA_SIZE			9


#define DEV_GOOD_MPLX115	0
#define DEV_GOOD_FLASH		1
#define DEV_GOOD_MPU6050	2


#define BIT_TRUE(x, n) ((x>>n)&1)

#define USB_TIMEOUT  5000



const char *FindFileNamePtr(const char *sz)
{
    const char *p;
    p = strrchr(sz, '\\');
    if (p == NULL) p = sz;
    else p++;
    return p;
}


const char *FindExtensionPtr(const char *sz)
{
    const char *p;
    p = strrchr(sz, '.');
    if (p == NULL) p = sz + strlen(sz);
    return p;
}




/* Used to get descriptor strings for device identification */
static int usbGetDescriptorString(usb_dev_handle *dev, int index, int langid, 
                                  char *buf, int buflen) {
    char buffer[256];
    int rval, i;

    // make standard request GET_DESCRIPTOR, type string and given index 
    // (e.g. dev->iProduct)
    rval = usb_control_msg(dev, 
        USB_TYPE_STANDARD | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
        USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8) + index, langid, 
        buffer, sizeof(buffer), 1000);

    if (rval < 0) // error
        return rval;

    // rval should be bytes read, but buffer[0] contains the actual response size
    if ((unsigned char)buffer[0] < rval)
        rval = (unsigned char)buffer[0]; // string is shorter than bytes read

    if (buffer[1] != USB_DT_STRING) // second byte is the data type
        return 0; // invalid return type

    // we're dealing with UTF-16LE here so actual chars is half of rval,
    // and index 0 doesn't count
    rval /= 2;

    /* lossy conversion to ISO Latin1 */
    for (i = 1; i < rval && i < buflen; i++) {
        if(buffer[2 * i + 1] == 0)
            buf[i-1] = buffer[2 * i];
        else
            buf[i-1] = '?'; /* outside of ISO Latin1 range */
    }
    buf[i-1] = 0;

    return i-1;
}


static usb_dev_handle * usbOpenDevice(int vendor, char *vendorName, 
                                      int product, char *productName) {
    struct usb_bus *bus;
    struct usb_device *dev;
    char devVendor[256], devProduct[256];

    usb_dev_handle * handle = NULL;

    usb_init();
    usb_find_busses();
    usb_find_devices();

    for (bus=usb_get_busses(); bus; bus=bus->next) {
        for (dev=bus->devices; dev; dev=dev->next) {                     
            if (dev->descriptor.idVendor != vendor ||
               dev->descriptor.idProduct != product)
                continue;

            /* we need to open the device in order to query strings */
            if (!(handle = usb_open(dev))) {
                fprintf(stderr, "Warning: cannot open USB device: %s\n",
                        usb_strerror());
                continue;
            }

            /* get vendor name */
            if (usbGetDescriptorString(handle, dev->descriptor.iManufacturer, 
                                      0x0409, devVendor, sizeof(devVendor)) < 0) {
                fprintf(stderr, 
                        "Warning: cannot query manufacturer for device: %s\n", 
                        usb_strerror());
                usb_close(handle);
                continue;
            }

            /* get product name */
            if (usbGetDescriptorString(handle, dev->descriptor.iProduct, 
                                      0x0409, devProduct, sizeof(devProduct)) < 0) {
                fprintf(stderr, 
                        "Warning: cannot query product for device: %s\n", 
                        usb_strerror());
                usb_close(handle);
                continue;
            }
			//printf("Got mfg = \"%s\" and product = \"%s\"\n", devVendor, devProduct);

            if (strcmp(devVendor, vendorName) == 0 && 
                strcmp(devProduct, productName) == 0)
                return handle;
            else
                usb_close(handle);
        }
    }

    return NULL;
}



void debug_byte_dump(int nBytes, unsigned char *buffer)
{
#ifdef DEBUG_BYTES
	int i;
    printf("Got %d bytes:\n", nBytes);
	for (i=0; i<nBytes; i++)
		printf("  %2d: %02x\n", i, buffer[i]);
#endif
}



int usage(char *sz, usb_dev_handle *handle)
{
    printf("Usage:\n");
    printf("%s status|read|test|clear|dump [filename]|write \"string\"|retrieve\n", sz);
	puts("   status    Print device status");
	puts("   read      Print current sensor conditions");
	puts("   test      Run test routine and print results");
	puts("   clear     Clear device memory");
	puts("   dump      Receive contents of device memory and");
	puts("              write to file <filename>");
	puts("   write     Writes first four bytes of \"string\"");
	puts("              to a test area in flash RAM");
	puts("   retrieve  Reads and displays four bytes from test");
	puts("              area in flash RAM");

	if (handle)
		usb_close(handle);

    return 1;
}



int main(int argc, char **argv)
{
	FILE *outfile;
    usb_dev_handle *handle = NULL;
    int nBytes = -1;
	int i, pages, pages_used;
    unsigned char buffer[256];
	double degC, degF;

    if (argc < 2) return usage(argv[0], handle);

	//usb_set_debug(USB_LOG_LEVEL_WARNING);

// Find our device
    handle = usbOpenDevice(0x16C0, "AviRoc.com", 0x05DC, "T-010");
    if (handle == NULL) {
        fprintf(stderr, "Could not find USB device!\n");
        exit(1);
    }
    if (usb_set_configuration(handle, 1) < 0) {
        printf("error setting config #1: %s\n", usb_strerror());
        usb_close(handle);
        return 0;
    }
    if (usb_claim_interface(handle, 0) < 0) {
		printf("Error claiming interface #1: %s\n", usb_strerror());
        usb_close(handle);
        return 0;
    }

// First, check whether RAM is busy (if it will be used by this call)
	if ((strcmp(argv[1], "clear") == 0) || (strcmp(argv[1], "dump") == 0) ||
		(strcmp(argv[1], "write") == 0) || (strcmp(argv[1], "retrieve") == 0)) {
		nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
			USB_IS_FLASH_BUSY, 0, 0, (char *)buffer, sizeof(buffer), USB_TIMEOUT);
		if (nBytes != 1) puts("Unexpected buffer size return for flash RAM busy check!");
		if (buffer[0]) {
			puts("Device flash RAM is currently busy, please stand by!");
			usb_close(handle);
			return 0;
		}
	}

// Check command line argument
    if (strcmp(argv[1], "status") == 0) {
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
            USB_STATUS, 0, 0, (char *)buffer, sizeof(buffer), USB_TIMEOUT);
		debug_byte_dump(nBytes, buffer);
		if (nBytes != 1) puts("Unexpected buffer size return!");
		else {
			//printf("Status = 0x%02x\n", buffer[0]);
			puts("=== Device connected to USB");
			printf("      Altimeter (MPL3115)        %s\n", BIT_TRUE(buffer[0],DEV_GOOD_MPLX115)?"OK":"BAD");
			printf("      Flash RAM (S2FL1%dK)       %s\n", (buffer[0] >> 5) * 16, BIT_TRUE(buffer[0],DEV_GOOD_FLASH)?"OK":"BAD");
			printf("      Accelerometer (MPU-6050)   %s\n", BIT_TRUE(buffer[0],DEV_GOOD_MPU6050)?"OK":"BAD");
		}
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
            USB_FLASH_STATUS, 0, 0, (char *)buffer, sizeof(buffer), USB_TIMEOUT);
		debug_byte_dump(nBytes, buffer);
		pages = (buffer[1] << 8) + buffer[0] - 1;
		pages_used = (buffer[5] << 8) + buffer[4] - 1;
		printf("=== Memory is %d%% full (%d KB used)\n", (pages_used * 100) / pages, (pages_used + 3) / 4);
    }
    else if (strcmp(argv[1], "read") == 0) {
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
            USB_READ_ALTIMETER, 0, 0, (char *)buffer, sizeof(buffer), USB_TIMEOUT);
		debug_byte_dump(nBytes, buffer);
		if (nBytes != 5) puts("Unexpected buffer size return!");
		else {
			printf("Altitude = %d.%04d meters\n", (buffer[0] << 8) + buffer[1], BIT_TRUE(buffer[2],7)*5000 +
				BIT_TRUE(buffer[2],6)*2500 + BIT_TRUE(buffer[2],5)*1250 + BIT_TRUE(buffer[2],4)*625);
			degC = buffer[3] + BIT_TRUE(buffer[4],7)*0.5 + BIT_TRUE(buffer[4],6)*0.25 +
				BIT_TRUE(buffer[4],5)*0.125 + BIT_TRUE(buffer[4],4)*0.0625;
			degF = 32. + 9 * degC / 5;
			printf("Temperature = %.4f degC (%.4f degF)\n", degC, degF);
		}
    }
	else if (strcmp(argv[1], "test") == 0) {
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
            USB_TEST, 0, 0, (char *)buffer, sizeof(buffer), USB_TIMEOUT);
        printf("Got %d bytes:\n", nBytes);
		for (i=0; i<nBytes; i++)
			printf("  %2d: %02x\n", i, buffer[i]);
    }
	else if (strcmp(argv[1], "clear") == 0) {
		nBytes = 0;
		printf("Clearing memory...");
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
            USB_CLEAR_MEMORY, 0, 0, (char *)buffer, sizeof(buffer), USB_TIMEOUT);
		if (nBytes == 1) puts("OK.");
		else puts("FAILURE.");
		debug_byte_dump(nBytes, buffer);
    }
	else if (strcmp(argv[1], "dump") == 0) {
		if (argc != 3) {
			puts("Must specify filename on the command line!\n");
		}
		else {
			fopen_s(&outfile, argv[2], "wb");
			if (!outfile) {
				printf("Error creating file \"%s\"!\n", argv[2]);
			}
			else {
				nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
					USB_DATA_SIZE, 0, 0, (char *)buffer, sizeof(buffer), USB_TIMEOUT);
				pages = *((short *)buffer);
				printf("Receiving %d.%02d KB of data", pages / 4, 25 * (pages % 4));
				for (i=0; i<pages; i++) {
					nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
						USB_PAGE_DUMP, 0, i, (char *)buffer, sizeof(buffer), USB_TIMEOUT);
					if (nBytes != 256) printf("\nWARNING: Short page received (len=%d)!", nBytes);
					fwrite(buffer, 1, nBytes, outfile);
					printf(".");
				}
				puts("\nDone!");
				fclose(outfile);
			}
		}
    }
	else if (strcmp(argv[1], "write") == 0) {
		if (argc != 3) {
			puts("Must specify string to write on the command line!\n");
		}
		else if (strlen(argv[2]) != 4) {
			puts("Must provide four-character string!\n");
		}
		else {
			nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
				USB_FLASH_WRITE_TEST, argv[2][0] + (argv[2][1] << 8), argv[2][2] + (argv[2][3] << 8),
				(char *)buffer, sizeof(buffer), USB_TIMEOUT);
			printf("Wrote \"%s\" to flash RAM!\n", argv[2]);
		}
    }
	else if (strcmp(argv[1], "retrieve") == 0) {
        nBytes = usb_control_msg(handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
            USB_FLASH_READ_TEST, 0, 0, (char *)buffer, sizeof(buffer), USB_TIMEOUT);
        printf("Got %d bytes:\n", nBytes);
		for (i=0; i<nBytes; i++)
			printf("  %2d: %02x '%c'\n", i, buffer[i], buffer[i]);
    }
	else return usage(argv[0], handle);

    if (nBytes < 0)
        fprintf(stderr, "USB error: %s\n", usb_strerror());

    usb_close(handle);

    return 0;
}