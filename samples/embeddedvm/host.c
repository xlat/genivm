/***
 * Author: GeNi
 * Date: Mars 2011
 ***/
#include <stdio.h>
#include <windows.h>
#include "..\..\geni_vm_lib.h"

//Some initialisation fakes 
int gb_init_1 = 0;
int gb_init_2 = 0;
int gb_init_3 = 0;

DWORD geni_vm_api_DoInit1(GENI_VM* vm, DWORD arg){
	gb_init_1 = TRUE; 
	return ~arg;
}
DWORD geni_vm_api_DoInit2(GENI_VM* vm, DWORD arg){
	static int counter = 0;
	if(counter++>9){
		gb_init_2 = TRUE;
	}
	return ~arg;
}
DWORD geni_vm_api_DoInit3(GENI_VM* vm, DWORD arg){
	gb_init_3 = TRUE;
	return ~arg;
}

DWORD geni_vm_api_GetInitState(GENI_VM* vm, DWORD arg){
	return gb_init_1 + gb_init_2 + gb_init_3;
}

int main(int argc, char** argv){
	int return_code = 0;
	char* binaryfile = "nagscreen.gvm";
	GENI_VM* vm = NULL;
	int i;

	vm = geni_vm_init();
	if(!vm){
		return_code = 4;
		goto safe_exit;
	}

	//Register some API
#pragma warning(push)
#pragma warning(disable:4028)
	geni_vm_register_api(vm, geni_vm_api_DoInit1, 0xcaca, "doinit1");
	geni_vm_register_api(vm, geni_vm_api_DoInit2, 0xdada, "doinit2");
	geni_vm_register_api(vm, geni_vm_api_DoInit3, 0xcafe, "doinit3");
	geni_vm_register_api(vm, geni_vm_api_GetInitState, 0xfeed, "getinitstate");
#pragma warning(pop)

	load_gvm_file(vm, binaryfile);
	geni_vm_run( vm );
	
safe_exit:
	geni_vm_delete( vm );	
	return return_code;
}
