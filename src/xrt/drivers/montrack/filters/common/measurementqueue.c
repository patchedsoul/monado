#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/u_misc.h>
#include "measurementqueue.h"

measurement_queue_t* measurement_queue_create()
{
	measurement_queue_t* mq = U_TYPED_CALLOC(measurement_queue_t);
	if (! mq) {
		printf("ERROR: could not malloc!\n");
		exit(0);
	}
	mq->measurements.items=U_TYPED_ARRAY_CALLOC(tracker_measurement_t,MEASUREMENTQUEUE_INITIAL_CAPACITY);
	mq->measurements.capacity = MEASUREMENTQUEUE_INITIAL_CAPACITY;

	mq->measurements.size = 0;
	mq->measurements.head_index = 0;
	mq->measurements.tail_index = 0;

	mq->measurements.last_optical_index = 0;
	pthread_mutex_init(&mq->queue_lock,NULL);
	mq->source_id_counter = 0;
    mq->timestamp_inc=0;
	return mq;
}

void measurement_queue_destroy(measurement_queue_t* mq)
{
	free(mq->measurements.items);
}

uint64_t measurement_queue_uniq_source_id(measurement_queue_t* mq) {
	//pthread_mutex_lock(&mq->queue_lock);
	return mq->source_id_counter++;
	//pthread_mutex_unlock(&mq->queue_lock);
}
void measurement_queue_add(measurement_queue_t* mq,tracker_measurement_t* tm)
{
	if (!mq) {
		return;
	}
	bool full = false;
	pthread_mutex_lock(&mq->queue_lock);

	mq->measurements.items[mq->measurements.tail_index] = *tm;

	if (tm->flags & MEASUREMENT_OPTICAL) {
		mq->measurements.last_optical_index = mq->measurements.tail_index;
    }

    //printf("added data at index %d head %d \n",mq->measurements.tail_index, mq->measurements.head_index);
	mq->measurements.tail_index++;
	if (mq->measurements.tail_index == mq->measurements.capacity) {
		mq->measurements.tail_index = 0;
	}

    if (mq->measurements.size >= mq->measurements.capacity) {
		full = true;
	}

	if (full) {
		if ((mq->measurements.head_index + 1) == mq->measurements.capacity) {
			mq->measurements.head_index = 0;
		} else {
			mq->measurements.head_index++;}
	} else {
		mq->measurements.size++;
	}
    //printf("measurements size %d\n",mq->measurements.size);

	pthread_mutex_unlock(&mq->queue_lock);
}

//NOTE: we alloc here, caller must free.
uint32_t measurement_queue_get_since_last_frame(measurement_queue_t* mq,uint32_t source_id,tracker_measurement_t** cm){
	pthread_mutex_lock(&mq->queue_lock);
	//iterate over our measurements, count the number of records we will copy
	//our starting timestamp is the one on our last frame
	uint64_t ts = mq->measurements.items[mq->measurements.last_optical_index].source_timestamp;
	uint32_t count=0;
	for (uint32_t i=0;i<mq->measurements.size;i++)
        if (mq->measurements.items[i].source_timestamp > ts) {
			count++;
		}
	*cm = U_TYPED_ARRAY_CALLOC(tracker_measurement_t,count);
	count =0;
	for (uint32_t i=0;i<mq->measurements.size;i++) {
        if (mq->measurements.items[i].source_timestamp > ts) {
			memcpy(*cm+count,&mq->measurements.items[i],sizeof(tracker_measurement_t));
			count++;
		}
	}
	pthread_mutex_unlock(&mq->queue_lock);
	return count;
}

uint32_t measurement_queue_get_since_timestamp(measurement_queue_t* mq,uint32_t source_id,uint64_t timestamp_ns,tracker_measurement_t** cm) {
    pthread_mutex_lock(&mq->queue_lock);
    //iterate over our measurements, count the number of records we will copy
    //our starting timestamp is the one on our last frame
    uint32_t count=0;
    for (uint32_t i=mq->measurements.head_index;i<mq->measurements.size;i++) {
        if (mq->measurements.items[i].source_timestamp > timestamp_ns) {
           printf("H %lld\n",mq->measurements.items[i].source_timestamp);
            count++;
        }
    }
    for (uint32_t i=0;i<mq->measurements.tail_index;i++) {
        if (mq->measurements.items[i].source_timestamp > timestamp_ns) {
            printf("0 %lld\n",mq->measurements.items[i].source_timestamp);
            count++;
        }
    }
    *cm = U_TYPED_ARRAY_CALLOC(tracker_measurement_t,count);
    count =0;
    for (uint32_t i=mq->measurements.head_index;i<mq->measurements.size;i++) {
        if (mq->measurements.items[i].source_timestamp > timestamp_ns) {
            memcpy(*cm+count,&mq->measurements.items[i],sizeof(tracker_measurement_t));
            count++;
        }
    }
    for (uint32_t i=0;i<mq->measurements.tail_index;i++) {
        if (mq->measurements.items[i].source_timestamp > timestamp_ns) {
            memcpy(*cm+count,&mq->measurements.items[i],sizeof(tracker_measurement_t));
            count++;
        }
    }

    pthread_mutex_unlock(&mq->queue_lock);
    return count;
}


