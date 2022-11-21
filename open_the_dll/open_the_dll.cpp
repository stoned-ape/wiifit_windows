#include <stdio.h>
#include <assert.h>
#ifdef UNICODE
static_assert(1 == 0, "");
#endif
//#undef _UNICODE
#include <windows.h>

void (*start_recording)(const char *) = NULL;

void (*stop_recording)() = NULL;

void (*start_replay)(const char *) = NULL;

void (*stop_replay)() = NULL;

void *(*run_render_thread)() = NULL;

int main(){
	char buf[1024];
	GetCurrentDirectory(sizeof(buf), buf);
	puts(buf);
	HMODULE hm = LoadLibrary("../x64/debug/dll1.dll");
	assert(hm);

	assert(start_recording = (void(*)(const char *))GetProcAddress(hm, "start_recording"));

	assert(start_replay = (void(*)(const char *))GetProcAddress(hm, "start_replay"));


	assert(stop_recording = (void(*)())GetProcAddress(hm, "stop_recording"));

	assert(stop_replay = (void(*)())GetProcAddress(hm, "stop_replay"));

	assert(run_render_thread = (void *(*)())GetProcAddress(hm, "run_render_thread"));
	
	WaitForSingleObject((HANDLE*)run_render_thread(), INT_MAX);

	//start_replay("record.bin");

}
