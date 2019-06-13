
typedef enum driver_event_type
{
	EVENT_NONE,
	EVENT_FRAMESERVER_GOTFRAME,
	EVENT_TRACKER_RECONFIGURED
} driver_event_type_t;

typedef struct driver_event
{
	driver_event_type_t type;
	// extra data to go along with events
	// can be added here
} driver_event_t;

typedef void (*event_consumer_callback_func)(void* instance,
                                             driver_event_t event);
