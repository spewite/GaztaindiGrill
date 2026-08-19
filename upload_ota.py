#!/usr/bin/env python3
# OTA upload over HTTP POST, invoked by PlatformIO as the "custom" upload_command
# (see platformio.ini). Replaces a curl.exe invocation: on this machine curl.exe
# resolves to a Cygwin build (x86_64-pc-cygwin) that fails with "URL rejected:
# Malformed input to a URL function" whenever --data-binary points at a file
# inside .pio, regardless of which file, its size or the path syntax used;
# sibling folders with an identical layout are unaffected. Root cause not
# isolated (not path length, quoting, slashes, hidden/reparse/ADS attributes
# or ACLs) despite testing all of them. Using Python's stdlib http client
# instead sidesteps the whole system-curl dependency, which also makes the
# build reproducible regardless of which curl happens to be first on PATH.
import sys
import urllib.error
import urllib.request

OTA_URL = "http://192.168.1.100:3232/update"


def main() -> int:
    firmware_path = sys.argv[1]

    with open(firmware_path, "rb") as f:
        data = f.read()

    print(f"Uploading {firmware_path} ({len(data)} bytes) to {OTA_URL}")
    request = urllib.request.Request(OTA_URL, data=data, method="POST")

    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            print(response.read().decode(errors="replace"))
    except urllib.error.HTTPError as e:
        print(f"OTA upload rejected by the grill: {e.code} {e.read().decode(errors='replace')}")
        return 1
    except urllib.error.URLError as e:
        print(f"OTA upload failed: {e.reason}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
