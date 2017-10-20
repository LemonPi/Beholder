# simple inquiry example
# requires PyBluez; a pain to set up on 64bit windows
# see http://www.lfd.uci.edu/~gohlke/pythonlibs/#pybluez for 64bit wheels
import bluetooth as bl
import sys

if sys.version < '3':
    input = raw_input

SPP_UID = "00001101-0000-1000-8000-00805f9b34fb"
# for this module only
MAC_ADDR = "00:14:03:05:D1:AE"

service_matches = bl.find_service(uuid=SPP_UID, address=MAC_ADDR)

if len(service_matches) != 1:
    print(service_matches)
    raise RuntimeError("Cannot find a bluetooth service; is it paired?")

bl_service = service_matches[0]

port = bl_service["port"]
name = bl_service["name"]
host = bl_service["host"]

print("connecting to \"%s\" on %s" % (name, host))

# Create the client socket
sock = bl.BluetoothSocket(bl.RFCOMM)
sock.connect((host, port))

print("connected.  type stuff")
while True:
    data = input()
    if len(data) == 0:
        break
    sock.send(data)

sock.close()


# use when using a new BL module
def print_nearby_devices():
    nearby_devices = bl.discover_devices(lookup_names=True)
    print("found %d devices" % len(nearby_devices))
    for addr, name in nearby_devices:
        print("  %s - %s" % (addr, name))
