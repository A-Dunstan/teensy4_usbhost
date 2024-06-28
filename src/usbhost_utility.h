#ifndef _USB_UTIL_H
#define _USB_UTIL_H

#include "Arduino.h"
#include "FS.h"

// wrapper to print DateTimeFields
class PrintableDateTimeFields : public DateTimeFields, public Printable {
public:
  virtual size_t printTo(Print& p) const {
    static const char* mlabels[] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec","???"};
    size_t mindex = mon < 12 ? mon : 12;
    return p.printf("%02u:%02u:%02u %s %u %04u", hour, min, sec, mlabels[mindex], mday, year+1900);
  }
  PrintableDateTimeFields() {
    sec=0, min=0, hour=0, wday=0, mday=1, mon=0, year=70;
  }
};

// recursively list all files in the given directory and all sub-directories
static void recurse_dir_print(String base, File &f, Print &p) {
  p.print("DIRECTORY: ");p.println(base);
  // display all files in the directory
  File n;
  while ((n = f.openNextFile())) {
    if (!n.isDirectory()) {
      PrintableDateTimeFields dt;
      n.getModifyTime(dt);
      // print fields in this order because it keeps things better aligned
      p.print("Modified: "); p.print(dt); p.print('\t');
      p.print("Size: "); p.print(n.size()); p.print('\t');
      p.print("Name: "); p.print(n.name());
      p.println();
    }
  }
  // now parse into each sub-directory
  f.rewindDirectory();
  while ((n = f.openNextFile())) {
    if (n.isDirectory())
      recurse_dir_print(base + n.name() + '/', n, p);
  }
}

// check if a pointer holds a valid USB descriptor of the specified type
template <class desc>
const desc* get_desc_type(const void* p, uint8_t type, const uint8_t* end, size_t min_length = sizeof(desc)) {
  const uint8_t *b = (const uint8_t*)p;
  // no space left for descriptor
  if (b >= end) return NULL;
  // is this descriptor long enough for a match?
  if (b[0] < min_length) return NULL;
  min_length = b[0];
  // is there enough room here for the entire descriptor?
  if (b + min_length > end) return NULL;
  // is this descriptor the wanted type?
  if (b[1] != type) return NULL;

  return (const desc*)b;
}

#endif // _USB_UTIL_H
