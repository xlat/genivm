#include <windows.h>
#include <stdio.h>

int main(int argv, char** argc){
	
	HMODULE hmod;
	void* func_ptr;
	hmod = LoadLibrary("kernel32.dll");
	func_ptr = GetProcAddress( hmod, "LoadLibraryA" );
	printf("LoadLibraryA: hmodule=0x%08x - func ptr=0x%08x\n", hmod, func_ptr );
	hmod = LoadLibrary("user32.dll");
	func_ptr = GetProcAddress( hmod, "MessageBoxA" );
	printf("MessageBoxA: hmodule=0x%08x - func ptr=0x%08x\n", hmod, func_ptr );
	
	return 0;
}