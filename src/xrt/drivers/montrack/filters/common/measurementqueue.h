#ifndef VECTOR_H
#define VECTOR_H

#include <pthread.h>
#include <optical_tracking/common/tracker.h>

#define MEASUREMENTQUEUE_INITIAL_CAPACITY 8
#define MAX_MEASUREMENT_SOURCES 32


typedef struct measurement_ring {
    tracker_measurement_t* items;
	uint32_t size;
	uint32_t capacity;
	uint32_t head_index;
	uint32_t tail_index;
	uint32_t last_optical_index; //optimisation to avoid search
} measurement_ring_t;

typedef struct measurment_queue {
	measurement_ring_t measurements;
	uint64_t source_id_counter;
    uint64_t timestamp_inc;
    pthread_mutex_t queue_lock;
} measurement_queue_t;

measurement_queue_t* measurement_queue_create();
void measurement_queue_destroy(measurement_queue_t* mq);

//bool measurement_queue_get_latest_n(measurement_queue_t* mq,uint32_t source_id, uint32_t* count,tracker_measurement_t* cm); //used by consumers
uint32_t measurement_queue_get_since_last_frame(measurement_queue_t* mq,uint32_t source_id,tracker_measurement_t** cm);
uint32_t measurement_queue_get_since_timestamp(measurement_queue_t* mq,uint32_t source_id,int64_t timestamp_us,tracker_measurement_t** cm);
void measurement_queue_add(measurement_queue_t* mq, tracker_measurement_t* m); //used by producers
uint64_t measurement_queue_uniq_source_id(measurement_queue_t* mq); //used by producers




#endif
