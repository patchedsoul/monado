#ifndef VECTOR_H
#define VECTOR_H

#include "frameservers/common/frameserver.h"
#define FRAMEQUEUE_INITIAL_CAPACITY 32;

typedef struct frame_data
{
	uint32_t refcount;
	uint8_t* buffer;
} framedata_t;

typedef struct frame_array
{
	frame_t* items;
	framedata_t* refdata;
	uint32_t size;
	uint32_t capacity;
} frame_array_t;

typedef struct frame_queue
{
	frame_array_t frames;
	uint64_t source_id_counter;
} frame_queue_t;

frame_queue_t*
frame_queue_instance(); //'singleton' initialisation

void
frame_queue_init();
frame_t*
frame_queue_ref_latest(frame_queue_t* fq,
                       uint32_t source_id); // used by consumers
void
frame_queue_unref(frame_queue_t* fq, frame_t* f); // used by consumers
void
frame_queue_add(frame_queue_t* fq, frame_t* f); // used by producers
uint64_t
frame_queue_uniq_source_id(frame_queue_t* fq); // used by producers

void
frame_array_init(frame_array_t* fa);
uint32_t
frame_array_total(frame_array_t* fa);
static void
frame_array_resize(frame_array_t* fa, uint32_t s);
void
frame_array_add(frame_array_t* fa, frame_t* f);
void
frame_array_set(frame_array_t* fa, uint32_t index, frame_t* f);
frame_t*
frame_array_get(frame_array_t* fa, uint32_t index);
void
frame_array_delete(frame_array_t* fa, uint32_t index);
void
frame_array_free(frame_array_t* fa);



#endif
