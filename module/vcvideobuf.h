#ifndef VCVIDEOBUF_H
#define VCVIDEOBUF_H

#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include "vcdevice.h"

void swap_in_queue_buffers( struct vc_in_queue * q );

int vc_in_queue_setup( struct vc_in_queue * q , size_t size );

void vc_in_queue_destroy( struct vc_in_queue * q );

int vc_out_videobuf2_setup( struct vc_device * dev );

int vc_out_queue_setup( struct vb2_queue * vq,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,0)
                         const struct v4l2_format * fmt,
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0)
                         const void *parg,
#endif
                         unsigned int *nbuffers, unsigned int *nplanes,
                         unsigned int sizes[],
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,8,0)
						 void *alloc_ctxs[]
#else
						 struct device* alloc_ctxs[]
#endif
						 );

int vc_out_buffer_prepare( struct vb2_buffer * vb );

void vc_out_buffer_queue( struct vb2_buffer * vb );

int vc_start_streaming( struct vb2_queue * q, unsigned int count );

void vc_stop_streaming( struct vb2_queue * q );

void vc_outbuf_lock( struct vb2_queue * vq );

void vc_outbuf_unlock( struct vb2_queue * vq );

//int direct_submit_user_buffer( const char * __user data, size_t length );

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)

//TODO ioctl and fop helper functions for kernel < 3.6.0

#endif /* Linux version < 3.6.0 */

#endif
