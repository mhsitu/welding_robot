// This file, and generally the shared memory part of the remote API
// are courtesy of Benjamin Navarro

#pragma once

#include <stddef.h>
#include <stdbool.h>

#ifdef _WIN32
typedef void* shm_handle_t;
#elif defined (__linux) || defined (__APPLE__)
typedef int shm_handle_t;
#endif /* _WIN32 */

typedef struct
{
	char name[28];
	shm_handle_t handle;
	size_t header_size;
	size_t size;
	unsigned char* buffer;
} shared_memory_info_t;

#ifdef __cplusplus
extern "C" {
#endif

bool create_shared_memory(shared_memory_info_t* info);
bool destroy_shared_memory(shared_memory_info_t* info);
bool open_shared_memory(shared_memory_info_t* info);
bool close_shared_memory(shared_memory_info_t* info);
bool map_shared_memory(shared_memory_info_t* info);
bool unmap_shared_memory(shared_memory_info_t* info);
void set_shared_memory_name(shared_memory_info_t* info, int identifier);
void set_shared_memory_size(shared_memory_info_t* info, size_t size);
bool is_valid_shared_memory_handle(shared_memory_info_t* info);

#ifdef __cplusplus
}
#endif