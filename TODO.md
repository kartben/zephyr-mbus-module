TODOs
=====

 - [x] Add CLI settings for baudrate and parity
 - [x] Add support for saving line settings across boots
 - [x] Add CLI diag for serial line
 - [x] Add ping command (ping primary addr)
 - [x] Let shell commands run in foreground, separate out logic for
       running in background thread
 - [x] Migrate to new secondary scanner, store probed addresses
       in a local array for now
 - [x] Add support for querying primary address on secondary probe
 - [x] Add support for requesting secondary address on primary scan
 - [ ] Add support for probing over all supported baud rates.
	   When starting up the first time, an M-Bus master needs
	   to find all connected devices.  Most go up with 2400 8E1
	   but some may run at other baud rates.
 - [ ] Store probed devices in a device registry:
       - primary address (from set address or scan op)
       - secondary address (from probe op)
	   - baud rate (from baud rate used during scan/probe)
	   - timestamp of last access?
