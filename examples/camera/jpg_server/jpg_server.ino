/* This example requires the QNEthernet library and ethernet connection.
 * This example requires at least one PSRAM chip.
 *
 * It is hardcoded to work with a specific USB webcam;
 * to test a different device requires changed the vendor ID
 * and product ID in USB_Cam::offer(), and the parameters
 * set in USB_Cam::attach() to match the streaming endpoint
 * of the new device. You can find these parameters by examining
 * the full configuration descriptor.
 * When the network is up and the camera is connected, point
 * a web browser at http://$(TEENSY'S IP)/stream to view the streamed
 * jpeg frames.
 */

#include <time.h>
#include <QNEthernet.h>
#include <teensy4_usbhost.h>

using namespace qindesign::network;

static DMAMEM TeensyUSBHost2 usb;

#define QUEUE_LENGTH 3
#define JPEGBUF_MAX (640*480*2)

typedef struct {
  uint8_t *jpegbuf;
  int len;
} jpeg_frame;

class USB_Cam : public USB_Driver, public USB_Driver::Factory {
  uint8_t ep_in;
  uint8_t iface_streaming;
  uint8_t alt_setting;
  int16_t wMaxPacketSize;
  volatile bool attached = 0;

  uint8_t *sample_buf = NULL;
  isolength iso_lengths[QUEUE_LENGTH];

  jpeg_frame current_frame = {NULL, 0};

  typedef struct {
    uint8_t *sample;
    isolength *length;
  } iso_transfer;
  ATOM_QUEUE transferq;
  iso_transfer transferq_msgs[QUEUE_LENGTH];

  ATOM_QUEUE inputq;
  jpeg_frame inputq_msgs[5];
  ATOM_QUEUE outputq;
  jpeg_frame outputq_msgs[5];

  bool offer(const usb_device_descriptor *d, const usb_configuration_descriptor*) override {
    if (getDevice() != NULL) return false; // already attached to a device
    if (d->idVendor == 0x057E && d->idProduct == 0x030A) return true;
    return false;
  }
  USB_Driver *attach(const usb_device_descriptor* d, const usb_configuration_descriptor*, USB_Device *dev) override {
    setDevice(dev);

    iface_streaming = 1;
    ep_in = 0x81;
    alt_setting = 6;
    /* manufacturer string in the device descriptor for this webcam is different between full/high speed,
     * makes it easy to differentiate without actually parsing the endpoint descriptor
     */
    wMaxPacketSize = (d->iManufacturer == 48 ? 3 : 1) * 0x3FC;
    sample_buf = (uint8_t*)aligned_alloc(32, wMaxPacketSize*8*QUEUE_LENGTH);

    // activate alt streaming interface
    ControlMessage(USB_REQTYPE_INTERFACE_SET, USB_REQ_SET_INTERFACE, alt_setting, iface_streaming, 0, NULL, [=](int r) {
      if (r >= 0) {
        for (int i=0; i < QUEUE_LENGTH; i++) {
          iso_transfer t = {sample_buf+i*8*wMaxPacketSize, &iso_lengths[i]};
          atomQueuePut(&transferq, 0, &t);
        }
        printf("Streaming interface is active\n");
        attached = true;
      }
    });

    return this;
  }
  void detach(void) {
    jpeg_frame f;
    iso_transfer t;
    attached = false;

    current_frame.jpegbuf = NULL;
    current_frame.len = 0;
    // empty the transfer queue
    while (atomQueueGet(&transferq, -1, &t) == ATOM_OK);
    // empty the input queue
    while (atomQueueGet(&inputq, -1, &f) == ATOM_OK);
    free(sample_buf);
  }

  void run_iso(uint8_t *dst, isolength& lengths) {
    auto iso_cb = [=, &lengths](int r) {
      const uint8_t *p = dst;
      if (current_frame.jpegbuf == NULL) {
        if (atomQueueGet(&inputq, -1, &current_frame) == ATOM_OK)
          current_frame.len = -1; // wait for start of a new frame
      }
      for (int i=0; i < 8; i++, p+= wMaxPacketSize) {
        int16_t packet_length = lengths[i];
        const uint8_t *packet = p;
        if (p[0] <= packet_length) {
          ++packet;
          while ((*packet++ & 0x80) == 0); // skip until the end-of-header flag is seen
          if (p[1] & 4) // header has PTS (skip over)
            packet += 4;
          if (p[1] & 8) // header has SCR (skip over)
            packet += 6;
          packet_length -= packet - p;
          if (packet_length>0 && current_frame.jpegbuf && current_frame.len >= 0) {
            memcpy(current_frame.jpegbuf+current_frame.len, packet, packet_length);
            current_frame.len += packet_length;
          }
          if (p[1] & 2) { // header specifies end of frame - send it out
            if (current_frame.len < 0)
              current_frame.len = 0; // a new frame is starting - start collecting packets
            else {
              if (current_frame.jpegbuf) {
                atomQueuePut(&outputq, -1, &current_frame);
                current_frame.jpegbuf = NULL;
              }
              if (atomQueueGet(&inputq, -1, &current_frame) == ATOM_OK)
                current_frame.len = 0;
            }
          }
        }
      }
      // run transfer again if there is still work to do
      if (r >= 0 && current_frame.jpegbuf) {
        run_iso(dst, lengths);
        return;
      }
      // otherwise put this transfer in the idle queue
      iso_transfer t = {dst, &lengths};
      atomQueuePut(&transferq, -1, &t);
    };

    for (int i=0; i < 8; i++)
      lengths[i] = wMaxPacketSize;
    IsochronousMessage(ep_in, lengths, dst, iso_cb);
  }

public:
  USB_Cam() {
    atomQueueCreate(&transferq, transferq_msgs, sizeof(transferq_msgs[0]), sizeof(transferq_msgs)/sizeof(transferq_msgs[0]));
    atomQueueCreate(&inputq, inputq_msgs, sizeof(inputq_msgs[0]), sizeof(inputq_msgs)/sizeof(inputq_msgs[0]));
    atomQueueCreate(&outputq, outputq_msgs, sizeof(outputq_msgs[0]), sizeof(outputq_msgs)/sizeof(outputq_msgs[0]));
  }
  ~USB_Cam() {
    atomQueueDelete(&outputq);
    atomQueueDelete(&inputq);
    atomQueueDelete(&transferq);
  }
  operator bool() {
    return attached;
  }

  void submitBuffer(uint8_t *pic, int length) {
    if (attached) {
      jpeg_frame f = {pic, length};
      if (atomQueuePut(&inputq, -1, &f) == ATOM_OK) {
        iso_transfer t;
        // start any/all idle transfers
        while (atomQueueGet(&transferq, -1, &t) == ATOM_OK) {
          run_iso(t.sample, *t.length);
        }
      }
    }
  }

  uint8_t* getBuffer(int &length) {
    if (attached) {
      jpeg_frame f;
      if (atomQueueGet(&outputq, 100, &f) == ATOM_OK) {
        if (f.jpegbuf) {
          length = f.len;
          return f.jpegbuf;
        }
      }
    }
    return NULL;
  }
};

static DMAMEM USB_Cam camera;
static EthernetServer server(80);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (CrashReport) Serial.print(CrashReport);

  usb.begin();

  Ethernet.onAddressChanged([]() {
    IPAddress ip = Ethernet.localIP();
    bool hasIP = (ip != INADDR_NONE);
    if (hasIP) {
      printf("[Ethernet] Address changed:\r\n");

      printf("    Local IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.subnetMask();
      printf("    Subnet   = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.gatewayIP();
      printf("    Gateway  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      ip = Ethernet.dnsServerIP();
      if (ip != INADDR_NONE) {  // May happen with static IP
        printf("    DNS      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
      }
    } else {
      printf("[Ethernet] Address changed: No IP address\r\n");
    }
  });

  Ethernet.begin();
  server.beginWithReuse();
}

static void http_date(EthernetClient &client) {
  char datetime[128];
  time_t t;

  time(&t);
  struct tm* l = localtime(&t);
  size_t len = strftime(datetime, sizeof(datetime), "%a, %d %b %Y %X GMT\r\n", l);
  client.writeFully(datetime, len);
}

#define CHUNK_BOUNDARY "31415926535897932384626434"

static void serve_start(EthernetClient &client) {
  static EXTMEM uint8_t jpegbuf[2][JPEGBUF_MAX];

  char request[512] = {0};
  int response = 0;

  if (client.read((uint8_t*)request, sizeof(request)-1) > 0) {
    response = 1;
    char *r, *e;
    for (r = request; (e = strstr(r, "\r\n")) || (e = strstr(r, "\n")); r = e)
    {
      if (e == r) {
        break;
      }
      if (*e == '\r')
        *e++ = '\0';
      *e++ = '\0';

      if (strncmp(r, "GET ", 4)==0) {
        r += 4;
        response = 2;
        if (camera && strncasecmp(r, "/stream ", 8)==0) {
          r += 8;
          response = 3;
        } else {
          // skip over unknown name
          while (*r && *r++ != ' ');
        }

        if (strncmp(r, "HTTP/1.", 7)==0)
            continue;
        response = 1;
      }
    }
  }

  switch (response) {
    //case 0: // no request - just close it
    default:
      break;
    case 1: // unknown request
      client.writeFully("HTTP/1.0 501 Not Implemented\r\n\r\n");
      client.flush();
      break;
    case 2: // not found
      client.writeFully("HTTP/1.0 404 Not found\r\n"
      "Date: ");
      http_date(client);
      client.writeFully("Content-Type: text/html; charset=UTF-8\r\n\r\n"
      "<!DOCTYPE html>\n<html>"
      "<body>"
      "<h1>Not Found</h1>"
      "<p>The requested URL was not found.</p>"
      "</body></html>");
      client.flush();
      break;
    case 3: // request matched stream URL
      client.writeFully("HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace;boundary=" CHUNK_BOUNDARY "\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "X-Framerate: 60\r\n\r\n");
      // get rid of any old output frames
      {
        int l;
        while (camera.getBuffer(l) != NULL);
      }
      camera.submitBuffer(jpegbuf[0], JPEGBUF_MAX);
      camera.submitBuffer(jpegbuf[1], JPEGBUF_MAX);
      return;
  }

  client.close();
}

static void serve_continue(EthernetClient &client) {
  struct timeval tv;
  char cheader[128];
  uint8_t *jpg_buf;
  int jpg_len;

  jpg_buf = camera.getBuffer(jpg_len);
  if (jpg_buf == NULL || jpg_len <= 0) {
    printf("Request timed out, closing connection\n");
    client.close();
    return;
  }

  gettimeofday(&tv, NULL);
  snprintf(cheader, sizeof(cheader), "Content-Length %d\r\nX-Timestamp: %lld.%06ld\r\n\r\n", jpg_len, tv.tv_sec, tv.tv_usec);
  client.writeFully("--" CHUNK_BOUNDARY "\r\nContent-Type: image/jpeg\r\n");
  client.writeFully(cheader);
  client.writeFully(jpg_buf, jpg_len);
  camera.submitBuffer(jpg_buf, JPEGBUF_MAX);
  client.writeFully("\r\n");
}

void loop() {
  static EthernetClient client;

  if (!client) {
    client = server.available();
    if (client) serve_start(client);
  }

  if (camera) {
    if (client) serve_continue(client);
  } else {
    if (client) client.close();
  }
}

