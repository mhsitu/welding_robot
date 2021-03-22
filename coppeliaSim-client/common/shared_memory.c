// This file, and generally the shared memory part of the remote API
// are courtesy of Benjamin Navarro

#include "shared_memory.h"

#include <stdio.h>

#ifdef _WIN32
	#include <windows.h>
	#include <memory.h> 
#elif defined (__linux) || defined (__APPLE__)
	#include <fcntl.h>      /* Defines O_ * constants */
	#include <sys/stat.h>   /* Defines mode constants */
	#include <sys/mman.h>
	#include <unistd.h>
	#include <sys/types.h>
#endif

void reset_shared_memory_pointer(shared_memory_info_t* info) {
	info->buffer = NULL;
#ifdef _WIN32
	info->handle = NULL;
#elif defined (__linux) || defined (__APPLE__)
	info->handle = -1;
#endif
}

bool create_shared_memory(shared_memory_info_t* info)
{
	info->header_size = 20;
#ifdef _WIN32
	info->handle = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, info->size + info->header_size, info->name);
	return info->handle != NULL;
#elif defined (__linux) || defined (__APPLE__)
	info->handle = shm_open(info->name, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
	return (info->handle != -1) && (ftruncate(info->handle, info->size + info->header_size) == 0);
#endif /* _WIN32 */
}

bool destroy_shared_memory(shared_memory_info_t* info)
{
	bool ok = false;
#ifdef _WIN32
	ok = CloseHandle(info->handle);
#elif defined (__linux) || defined (__APPLE__)
	ok = (shm_unlink(info->name) == 0) && (close(info->handle) == 0);
#endif /* _WIN32 */
	reset_shared_memory_pointer(info);
	return ok;
}

bool open_shared_memory(shared_memory_info_t* info)
{
	info->header_size = 20;
#ifdef _WIN32
	info->handle = OpenFileMapping(FILE_MAP_ALL_ACCESS, FALSE, info->name);
	return info->handle != NULL;
#elif defined (__linux) || defined (__APPLE__)
	info->handle = shm_open(info->name, O_RDWR, S_IRUSR | S_IWUSR);
	return (info->handle != -1);
#endif /* _WIN32 */
}

bool close_shared_memory(shared_memory_info_t* info)
{
	bool ok = false;
#ifdef _WIN32
	ok = CloseHandle(info->handle);
#elif defined (__linux) || defined (__APPLE__)
	ok = (close(info->handle) == 0);
#endif /* _WIN32 */
	reset_shared_memory_pointer(info);
	return ok;
}

bool map_shared_memory(shared_memory_info_t* info)
{
#ifdef _WIN32
	info->buffer = (unsigned char*) MapViewOfFile(info->handle, FILE_MAP_ALL_ACCESS, 0,0, info->size + info->header_size);
	return info->buffer != NULL;
#elif defined (__linux) || defined (__APPLE__)
	info->buffer = (unsigned char*) mmap(NULL, info->size + info->header_size, PROT_READ | PROT_WRITE, MAP_SHARED, info->handle, 0);
	return info->buffer != MAP_FAILED;
#endif /* _WIN32 */
}

bool unmap_shared_memory(shared_memory_info_t* info)
{
#ifdef _WIN32
	return UnmapViewOfFile(info->handle);
#elif defined (__linux) || defined (__APPLE__)
	bool ok = true;
	if(info->buffer != NULL)
	{
		ok = munmap(info->buffer, info->size + info->header_size) == 0;
		info->buffer = NULL;
	}
	return ok;
#endif /* _WIN32 */
}

void set_shared_memory_name(shared_memory_info_t* info, int identifier)
{
	if(identifier < 0) {
		identifier = -identifier;
	}
#ifdef _WIN32
	sprintf(info->name, "Local\\SIM_REMOTE_API%05d", identifier);
#elif defined (__linux) || defined (__APPLE__)
	sprintf(info->name, "/LocalSIM_REMOTE_API%05d", identifier);
#endif /* _WIN32 */
}

void set_shared_memory_size(shared_memory_info_t* info, size_t size)
{
	info->size = size;
}

bool is_valid_shared_memory_handle(shared_memory_info_t* info) {
#ifdef _WIN32
	return info->handle != NULL;
#elif defined (__linux) || defined (__APPLE__)
	return info->handle != -1;
#endif /* _WIN32 */
}
