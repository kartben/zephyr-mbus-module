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

If you are using the nRF Common Application Framework (CAF) you need a
slightly more complex `main()` function, but otoh you don't need to call
`mbus_init()` anymore.

```C
#define MODULE main

#include <zephyr/logging/log.h>
#include <caf/events/module_state_event.h>
#include <zephyr/settings/settings.h>

LOG_MODULE_REGISTER(MODULE);

int main(void)
{
    /* Start CAF and signal that main have completed system setup */
    if (app_event_manager_init()) {
        LOG_ERR("Application Event Manager not initialized!");
        return 1;
    }

    settings_subsys_init();
    settings_load();

    LOG_DBG("Event manager initialized.");
    module_set_state(MODULE_STATE_READY);

   return 0;
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
