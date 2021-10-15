# libtransport-socket - transport socket library

libtransport-socket is a C library to handle sending and receiving packets using
an IP socket.

## Dependencies

The library depends on the following Alchemy modules:

* libfutils
* libpomp
* libtransport-packet
* libulog

## Building

Building is activated by enabling _libtransport-socket_ in the Alchemy build
configuration.

## Operation

### Threading model

The library is designed to run on a _libpomp_ event loop (_pomp_loop_, see
_libpomp_ documentation). All API functions must be called from the _pomp_loop_
thread. All callback functions are called from the _pomp_loop_ thread.
