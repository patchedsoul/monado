#include "tracker.h"
#include "tracker3D_sphere_mono.h"
#include "tracker3D_sphere_stereo.h"
#include "tracker3D_uvbi.h"

#include <string.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <errno.h>

tracker_instance_t*
tracker_create(tracker_type_t t)
{
	tracker_instance_t* i = calloc(1, sizeof(tracker_instance_t));
	if (i) {
		switch (t) {
		case TRACKER_TYPE_SPHERE_MONO:
			i->tracker_type = t;
			i->internal_instance = tracker3D_sphere_mono_create(i);
			i->tracker_get_capture_params =
			    tracker3D_sphere_mono_get_capture_params;
			i->tracker_get_poses = tracker3D_sphere_mono_get_poses;
			i->tracker_get_debug_frame =
			    tracker3D_sphere_mono_get_debug_frame;
			i->tracker_queue = tracker3D_sphere_mono_queue;
			i->tracker_register_measurement_callback =
			    tracker3D_sphere_mono_register_measurement_callback;
			i->tracker_register_event_callback =
			    tracker3D_sphere_mono_register_event_callback;
			i->tracker_has_new_poses =
			    tracker3D_sphere_mono_new_poses;
			i->tracker_configure = tracker3D_sphere_mono_configure;
			break;
		case TRACKER_TYPE_SPHERE_STEREO:
			i->tracker_type = t;
			i->internal_instance =
			    tracker3D_sphere_stereo_create(i);
			i->tracker_get_capture_params =
			    tracker3D_sphere_stereo_get_capture_params;
			i->tracker_get_poses =
			    tracker3D_sphere_stereo_get_poses;
			i->tracker_get_debug_frame =
			    tracker3D_sphere_stereo_get_debug_frame;
			i->tracker_queue = tracker3D_sphere_stereo_queue;
			i->tracker_register_measurement_callback =
			    tracker3D_sphere_stereo_register_measurement_callback;
			i->tracker_register_event_callback =
			    tracker3D_sphere_stereo_register_event_callback;
			i->tracker_has_new_poses =
			    tracker3D_sphere_stereo_new_poses;
			i->tracker_configure =
			    tracker3D_sphere_stereo_configure;
			break;
		case TRACKER_TYPE_UVBI:
			i->tracker_type = t;
			i->internal_instance = tracker3D_uvbi_create(i);
			i->tracker_get_capture_params =
			    tracker3D_uvbi_get_capture_params;
			i->tracker_get_poses = tracker3D_uvbi_get_poses;
			i->tracker_get_debug_frame =
			    tracker3D_uvbi_get_debug_frame;
			i->tracker_queue = tracker3D_uvbi_queue;
			i->tracker_register_measurement_callback =
			    tracker3D_uvbi_register_measurement_callback;
			i->tracker_register_event_callback =
			    tracker3D_uvbi_register_event_callback;
			i->tracker_has_new_poses = tracker3D_uvbi_new_poses;
			i->tracker_configure = tracker3D_uvbi_configure;
			break;
		case TRACKER_TYPE_NONE:
		default:
			free(i);
			return NULL;
			break;
		}
		// TODO: make this optional - we should use pipewire or similar
		// here
		// Create debug socket file descriptor
		if ((i->debug_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
			printf("ERROR: socket creation failed\n");
			return NULL;
		}
		int opt = 1;
		if (setsockopt(i->debug_fd, SOL_SOCKET,
		               SO_REUSEADDR | SO_REUSEPORT, &opt,
		               sizeof(opt))) {
			printf("ERROR: socket option setting failed\n");
			return NULL;
		}
		i->debug_address.sin_family = AF_INET;
		i->debug_address.sin_addr.s_addr = INADDR_ANY;
		i->debug_address.sin_port = htons(6666);
		if (bind(i->debug_fd, (struct sockaddr*)&i->debug_address,
		         sizeof(i->debug_address)) < 0) {
			printf("ERROR: socket option setting failed\n");
			return NULL;
		}
		if (listen(i->debug_fd, 3) < 0) {
			printf("ERROR: socket listen failed\n");
			return NULL;
		}

		if (ioctl(i->debug_fd, FIONBIO, (char*)&opt) < 0) {
			printf("ERROR: non-blocking ioctl failed");
			close(i->debug_fd);
			return NULL;
		}
		signal(SIGPIPE, SIG_IGN); // ignore sigpipe.
		i->client_connected = false;
		return i;
	}
	return NULL;
}

bool
tracker_send_debug_frame(tracker_instance_t* inst)
{
	frame_t f = {};
	if (inst->tracker_get_debug_frame(inst, &f)) {
		if (!inst->client_connected) {
			inst->debug_client_fd =
			    accept(inst->debug_fd, NULL, NULL);

			if (inst->debug_client_fd == -1) {
				if (errno == EWOULDBLOCK) {
					return false;
				} else {
					// some other socket problem, assume we
					// cannot continue
					close(inst->debug_client_fd);
					inst->client_connected = false;
					return false;
				}
			}
			inst->client_connected = true;
		}

		if (inst->client_connected) {
			int ret =
			    recv(inst->debug_client_fd, NULL, 0, MSG_DONTWAIT);
			if (ret == 0) {
				// we are disconnected
				close(inst->debug_client_fd);
				inst->client_connected = false;
				return false;
			}
			// just firehose the data at the client, and hope it can
			// handle it.
			send(inst->debug_client_fd, f.data, f.size_bytes, 0);
			return true;
		}
	}
	return false;
}
