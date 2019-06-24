#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mt_framequeue.h"

frame_queue_t* frame_queue_instance()
{
	static frame_queue_t* fq = NULL;

	// TODO: locking
	if(fq == NULL) {
		fq = malloc(sizeof(frame_queue_t));
		if (! fq) {
			printf("ERROR: could not malloc!\n");
			exit(0);
		}
		frame_array_init(&fq->frames);
		fq->source_id_counter=0;
        //make sure our source_frames is initialised to NULLs;
        memset(fq->source_frames,0,sizeof(frame_t)* MAX_FRAME_SOURCES);
	}
	return fq;
}

uint64_t frame_queue_uniq_source_id(frame_queue_t* fq) {
	//TODO: locking
    return fq->source_id_counter++;
}


frame_t* frame_queue_ref_latest(frame_queue_t* fq,uint32_t source_id){
	//find the latest frame from this source, increment its refcount and return it
	//TODO: locking
	uint64_t highest_seq =0;
	uint32_t selected_index=0;
	frame_t* ret=NULL;
	for (uint32_t i =0;i<fq->frames.size;i++) {
		frame_t* f =&fq->frames.items[i];
		if (f->source_id == source_id && f->source_sequence > highest_seq) {
			highest_seq = f->source_sequence;
			ret = f;
			selected_index=i;
		}
	}
	if (ret){
		framedata_t* fd = &fq->frames.refdata[selected_index];
		fd->refcount++;
		return ret;
	}
	return NULL;
}

void frame_queue_unref(frame_queue_t* fq,frame_t* f) {
	//find the frame index, based on the source id and sequence id and decrement the corresponding index
	//TODO: locking
	uint32_t selected_index=0;
	bool found =false;
	for (uint32_t i =0;i<fq->frames.size;i++) {
		frame_t* qf =&fq->frames.items[i];
		if (qf->source_id == f->source_id && qf->source_sequence == f->source_sequence) {
			selected_index=i;
			found =true;
		}
	}
	if (found){
		framedata_t* fd = &fq->frames.refdata[selected_index];
		fd->refcount--;
	}

}
void frame_queue_add(frame_queue_t* fq,frame_t* f) {
	//delete any unrefed frames for this source, then add this new one
	//TODO: locking

    //update our frame data for this source
    if (f->source_id < MAX_FRAME_SOURCES) {
        fq->source_frames[f->source_id] = *f;
        fq->source_frames[f->source_id].data = NULL;
    }
	printf("queue add: existing size: %d\n",fq->frames.size);
	uint32_t* indices_to_remove = malloc(sizeof(uint32_t) * fq->frames.size);
	uint32_t c_index=0;
	for (uint32_t i =0;i<fq->frames.size;i++) {
		framedata_t* fd = &fq->frames.refdata[i];
		printf("checking frame - refcount %d\n",fd->refcount);
		if (fd->refcount == 0) {
			indices_to_remove[c_index] = i;
			c_index++;
		}
	}
	printf("queue marking %d indices for removal\n",c_index);

	for (uint32_t i=0;i<c_index;i++)
	{
		frame_array_delete(&fq->frames,indices_to_remove[i]);
		printf("queue deleting frame new size %d\n",fq->frames.size);
	}
	free(indices_to_remove);
	frame_array_add(&fq->frames,f);
	printf("queue adding frame new size %d\n",fq->frames.size);

}

void frame_array_init(frame_array_t* fa)
{
	fa->capacity = FRAMEQUEUE_INITIAL_CAPACITY;
	fa->size = 0;
	fa->items = malloc(sizeof(frame_t) * fa->capacity);
	fa->refdata=malloc(sizeof(framedata_t) * fa->capacity);
}

uint32_t frame_array_size(frame_array_t* fa)
{
	return fa->size;
}

void frame_array_resize(frame_array_t* fa, uint32_t capacity)
{
    #ifdef DEBUG_ON
	printf("resize: %d to %d\n", fa->capacity, capacity);
    #endif

	void **new_items = realloc(fa->items, sizeof(frame_t) * capacity);
	void **new_refdata = realloc(fa->refdata, sizeof(framedata_t) * capacity);
	if (new_items && new_refdata) {
		fa->items = (frame_t*)new_items;
		fa->refdata = (framedata_t*)new_refdata;
		fa->capacity = capacity;
	}


}

void frame_array_add(frame_array_t* fa, frame_t* f)
{
	if (fa->capacity == fa->size) {
		frame_array_resize(fa, fa->capacity * 2); //alloc double the size on mem exhaustion
	}
	frame_t* nf = fa->items+fa->size ;
	memcpy(nf,f,sizeof(frame_t));
	framedata_t* fd = fa->refdata+fa->size;
	fd->refcount =0;
	fd->buffer = malloc(f->size_bytes);
	memcpy(fd->buffer,f->data,f->size_bytes);
	nf->data = fd->buffer; //point the queued frames data ptr at our refdata
	fa->size++;
	//printf("adding - new size: %d\n",fa->size);

}

/*void frame_array_set(frame_array_t* fa, uint32_t index, frame_t* f)
{
	if (index >= 0 && index < fa->size) {
		memcpy( fa->items+index,f,sizeof(frame_t));
	}
}*/

frame_t* frame_array_get(frame_array_t* fa, uint32_t index)
{
	if (index < fa->size) {
		return &(fa->items[index]);
	}
	return NULL;
}

void frame_array_delete(frame_array_t* fa, uint32_t index)
{
	if (index >= fa->size) {
		return;
	}

	framedata_t* fd = &fa->refdata[index];
	free(fd->buffer);
	fd->buffer = NULL;
	for (uint32_t i = index; i < fa->size - 1; i++) {
		fa->items[i] = fa->items[i+1];
		fa->refdata[i] = fa->refdata[i+1];
	}

	fa->size--;
	//printf("deleting - new size: %d\n",fa->size);

	if (fa->size > 0 && fa->size == fa->capacity / 4) //reclaim memory on major size reduction
		frame_array_resize(fa, fa->capacity / 2);
}

void frame_array_free(frame_array_t* fa)
{
	//free(v->items); //free all the data malloced
}
