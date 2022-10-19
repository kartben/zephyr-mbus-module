Zephyr Port of libmbus
======================

To use this Zephyr module in an application, see [zephyr-mbus-app][].
Make sure to use matching versions of the two.

When you've set up your `west.yml` and enabled the Zephyr shell, your
main function should look something like this:

```C
#include <mbus/mbus.h>

int main(void)
{
    return mbus_init();
}
```

> **Note:** This is the Zephyr port of the M-bus Library from [Raditex
> Control][1].  For the original, see https://github.com/rscada/libmbus

libmbus is an open source library for the M-bus (Meter-Bus) protocol available
under the 3-clause [BSD license][2].

The Meter-Bus is a standard for reading out meter data from electricity meters,
heat meters, gas meters, etc. The M-bus standard deals with both the electrical
signals on the M-Bus, and the protocol and data format used in transmissions on
the M-Bus. The role of libmbus is to decode/encode M-bus data, and to handle
the communication with M-Bus devices.

For more information see http://www.rscada.se/libmbus

[0]: https://zephyrproject.org
[1]: http://www.rscada.se
[2]: https://en.wikipedia.org/wiki/BSD_licenses
[zephyr-mbus-app]: addiva-elektronik/zephyr-mbus-app@9250145
