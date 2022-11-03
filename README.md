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

Origin & References
-------------------

This is the Zephyr port of the *Addiva Elektronik fork* of the original
M-bus Library.  For more information, see [libmbus][].

[libmbus]: https://github.com/addiva-elektronik/libmbus
[zephyr-mbus-app]: https://github.com/addiva-elektronik/zephyr-mbus-app
