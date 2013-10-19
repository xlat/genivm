#ifndef GENI_VM_LIB_H
#define GENI_VM_LIB_H

#pragma comment(lib, "geni_vm.lib")
typedef struct _GENI_VM { int reserved; } GENI_VM;
typedef DWORD (fnAPIHandler) (GENI_VM* , DWORD );

//#define prototype of exported functions
#ifdef CPP
extern "C"{
#endif
	void geni_vm_set_userdata( GENI_VM* vm, void* userdata);
	void* geni_vm_get_userdata( GENI_VM* vm);
	void geni_vm_register_api(GENI_VM* vm, fnAPIHandler* apihandler, DWORD id, char* name);
	BOOL load_gvm_data(GENI_VM* vm, BYTE* binarydata);
	void geni_vm_run( GENI_VM* vm );
	void geni_vm_delete( GENI_VM* vm );
	GENI_VM* geni_vm_init();
#ifdef CPP
}
#endif
#endif // GENI_VM_LIB_H