#ifndef SRC_WATCHDOG_HANDLER_H_
#define SRC_WATCHDOG_HANDLER_H_

extern RestrictedTask watchdog_handler;
extern Semaphore watchdog_activator;
extern bool user_initiated_reset;
extern const char * user_reset_reason;

//!< Intentional reset (firmware update, SD card inserted, ...): the calling
//!< task spins until the watchdog resets the system. The report written
//!< after the reset is named *.REBOOT and names the reason.
void reset_by_watchdog( const char *reason) __attribute__((noreturn));

#endif /* SRC_WATCHDOG_HANDLER_H_ */
