/***
 * Author: GeNi
 * Date: Mars 2011
 ***/
#include "vm.h"

#ifdef HAS_CORE
//Global for the core app
BOOL verbose = FALSE;
#endif

void safe_semaphore_init(GENI_VM* vm){
	/*
	SECURITY_ATTRIBUTES sa;
	sa.nLength = sizeof(SECURITY_ATTRIBUTES);
	sa.bInheritHandle = TRUE;
	sa.lpSecurityDescriptor = NULL;	
	//InitializeCriticalSectionEx(&vm->criticalsection, 0x400, 0 );	//InitializeCriticalSection(&vm->criticalsection);
	vm->semaphore = CreateSemaphore(&sa, 1, 1, NULL);
	vm->flags.critical = FALSE;
	*/
}
void safe_semaphore_deinit(GENI_VM* vm){
	/*
	CloseHandle(vm->semaphore);
	//DeleteCriticalSection(&vm->criticalsection);
	vm->flags.critical = FALSE;
	*/
}

void safe_semaphore_begin(GENI_VM* vm){
	/*
	DWORD dwWaitResult;
	BOOL bContinue=TRUE;
	//vm->flags.critical = TRUE;
	//EnterCriticalSection(&vm->criticalsection);
	while(bContinue){
		dwWaitResult = WaitForSingleObject( vm->semaphore, 0);   // handle to semaphore, zero-second time-out interval
		switch (dwWaitResult){ 
			// The semaphore object was signaled.
			case WAIT_OBJECT_0: 
				bContinue=FALSE;
				//printf("Thread %d: wait succeeded\n", GetCurrentThreadId());
				break;
			// The semaphore was nonsignaled, so a time-out occurred.
			case WAIT_TIMEOUT:
				//printf("Thread %d: wait timed out\n", GetCurrentThreadId());
				break; 
		}
	}
	*/
}

void safe_semaphore_end(GENI_VM* vm){
	/*
	DWORD count;
	if (!ReleaseSemaphore( vm->semaphore, 1, &count)){ //handle to semaphore,increase count by one,not interested in previous count
		printf("ReleaseSemaphore error: %d\n", GetLastError());
    }	
	//LeaveCriticalSection(&vm->criticalsection);
	//vm->flags.critical = FALSE;
	*/
}

void* safe_alloc( DWORD size ){
	void* ptr;
	HANDLE heap=GetProcessHeap();
	ptr = HeapAlloc( heap, HEAP_GENERATE_EXCEPTIONS | HEAP_ZERO_MEMORY, (SIZE_T)size);
#ifdef VM_HEAP_TRACE 
	printf("[heap=%08X] safe_alloc( %08X ) = %08X\n", heap, size, ptr);
#endif
	if(!ptr){
		DebugBreak();
	}	
	return ptr;
	/*void* ret = malloc( (size_t)size ); 
	if(ret){
		memset( ret, 0, size);
	}
	return ret;
	*/
}

void* safe_realloc( void** address, DWORD newsize){
	void* newaddress = NULL;
	HANDLE heap = NULL;
	heap=GetProcessHeap();
	if(!*address){
		newaddress = safe_alloc(newsize);
	}
	else{
		newaddress = HeapReAlloc(heap,HEAP_GENERATE_EXCEPTIONS | HEAP_ZERO_MEMORY, *address, (SIZE_T)newsize);
	}
	//void* newaddress = realloc( *address, newsize );
#ifdef VM_HEAP_TRACE 
	printf("[heap=%08X] safe_realloc( %08X, %08X ) = %08X\n", heap, *address, newsize, newaddress);
#endif
	if(newaddress){
		*address = newaddress;
	}
	else{
		DebugBreak();
	}
	return newaddress;
}

void safe_free( void* ptr ){ 
	HANDLE heap=GetProcessHeap();
#ifdef VM_HEAP_TRACE 
	printf("[heap=%08X] safe_free( %08X )\n", heap, ptr);
#endif
	if(ptr) HeapFree(heap, 0, ptr);
	//if(ptr) free(ptr); 
}

void geni_vm_backup_stack(GENI_STACK* source, GENI_STACK* backup){
	//create a clone of the given stack
	DWORD* ptr = (DWORD*)safe_alloc(  source->size * sizeof( DWORD ) );
#ifdef VM_DEBUG_STACK
	printf("** geni_vm_backup_stack ** sp=%08X, size=%08X ptr=%08X\n", source->sp, source->size, source->ptr);
#endif
	backup->size = source->size;
	backup->sp = source->sp;
	memcpy(ptr, source->ptr, source->size * sizeof( DWORD ));
	backup->ptr = source->ptr;
	source->ptr = ptr;
}

void geni_vm_restore_stack(GENI_STACK* target, GENI_STACK* backup){
	DWORD* ptr = target->ptr;
#ifdef VM_DEBUG_STACK
	printf("** geni_vm_restore_stack ** sp=%08X, size=%08X ptr=%08X\n", target->sp, target->size, target->ptr);
	printf("\tusing backup: sp=%08X, size=%08X ptr=%08X\n", backup->sp, backup->size, backup->ptr);
#endif
	target->size = backup->size;
	target->sp = backup->sp;
	target->ptr = backup->ptr;
	safe_free(ptr);
}



DWORD geni_vm_do_sub(GENI_CALLBACK* cb, DWORD* host_stackpointer){
//	GENI_OPCODE opcode_call;
//	OPCODE32(&opcode_call, OC_CALL, CST32(cb->vm_ip), UNDEF32 );
//	geni_vm_opcode_call32(vm, &opcode_call);
	GENI_REGISTER bak_register;
	GENI_STACK bak_stack;
	GENI_STACK bak_callstack;
	GENI_VM* vm;
	DWORD current_ip, returnvalue, subreturnval, i, *stackptr;
/*	DWORD bk1[1000], bk2[1000];	//Hack @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	bak_stack.ptr = bk1;
	bak_callstack.ptr = bk2;
*/
	vm = (GENI_VM*)cb->vmptr;
	/*
	if(vm->flags.critical){
		DebugBreak();	//ARG !!!
	}*/
	//safe_semaphore_begin(vm);
	//vm->flags.critical = TRUE;
	memcpy( &bak_register, &vm->registers, sizeof(GENI_REGISTER) );
	geni_vm_backup_stack(&vm->stack, &bak_stack);
	geni_vm_backup_stack(&vm->callstack, &bak_callstack );
#ifdef VM_DEBUG
	printf("** geni_vm_do_sub Entering: IP=%08X => %08X\n", vm->code.ip, cb->vm_ip);
#endif
	current_ip = vm->code.ip;
	returnvalue = vm->return_value;

	//copy HOST stack into VM stack (en copie inversée ?)
	//	stackptr = &vm->stack.ptr[ vm->stack.sp ];
	vm->stack.sp += cb->arg_count;
	i = vm->stack.sp;
	if(vm->stack.size < i){
#ifdef VM_DEBUG_STACK
		printf("** geni_vm_do_sub: reallocating stack (%08X) from %08X to %08X\n", vm->stack.ptr, vm->stack.size , i + DEFAULT_STACK_SIZE);
#endif
		vm->stack.size = i + DEFAULT_STACK_SIZE;
		safe_realloc( &vm->stack.ptr, vm->stack.size );
	}
	stackptr = &vm->stack.ptr[ vm->stack.sp ];
#ifdef VM_DEBUG_STACK 
#ifdef CAN_DUMP
	geni_vm_dump_stack("Args Stack", &vm->stack);
	geni_vm_dump_stack("Ori Args Stack", &bak_stack );
#endif
#endif
	for(i=0;i<=cb->arg_count;i++){
		*stackptr = *host_stackpointer;
		stackptr--;
		host_stackpointer++;
	}
	//vm->stack.sp++;
	vm->code.ip = cb->vm_ip;
	vm->callstack.sp = 0;
	geni_vm_run(vm);
	subreturnval = vm->return_value;

	memcpy( &bak_register, &vm->registers, sizeof(GENI_REGISTER) );
	geni_vm_restore_stack(&vm->stack, &bak_stack);
	geni_vm_restore_stack(&vm->callstack, &bak_callstack );
	vm->code.ip = current_ip;
#ifdef VM_DEBUG
	printf("** geni_vm_do_sub Outgoing: IP=%08X => %08X\n", cb->vm_ip, vm->code.ip);
#endif
	vm->return_value = returnvalue;
	//vm->flags.critical = FALSE;
	//
	cb->return_value = subreturnval;
	return subreturnval;
}
/*
DWORD WINAPI DoSub_ThreadProc( GENI_CALLBACK* cb ){
	safe_semaphore_begin((GENI_VM*)cb->vmptr);
	geni_vm_do_sub( cb, cb->host_stack );
	safe_semaphore_end((GENI_VM*)cb->vmptr);
	return TRUE;
}

DWORD geni_vm_create_sub_instance( GENI_CALLBACK* cb, DWORD* host_stackpointer){
	DWORD ThreadID;
	HANDLE threadHandle;
	cb->host_stack = host_stackpointer;
	threadHandle = CreateThread( 
		 NULL,       // default security attributes
		 0,          // default stack size
		 (LPTHREAD_START_ROUTINE) DoSub_ThreadProc, 
		 cb,       // no thread function arguments
		 0,          // default creation flags
		 &ThreadID); // receive thread identifier
	//wait for end of the tread;
	if(threadHandle){
		//WaitForSingleObject(ThreadID, INFINITE);
		WaitForMultipleObjects(1, &threadHandle, TRUE, INFINITE);
		CloseHandle(threadHandle);
	}
	cb->host_stack = NULL;
	return cb->return_value;
}

*/
DWORD geni_vm_make_callback( GENI_VM* vm, DWORD ip, DWORD stack_in ){
	//create a callback
	DWORD i, cbptr, wrappersize, cbstr, tmplptr, *topatch;
	i = vm->callback.count++;
	safe_realloc( &vm->callback.items, sizeof( GENI_CALLBACK ) * vm->callback.count );
	vm->callback.items[i].vm_ip = ip;
	vm->callback.items[i].vmptr = (DWORD)vm;
	vm->callback.items[i].arg_count = stack_in;
	__asm{
		lea eax, start_block
		mov tmplptr, eax
		lea eax, end_block
		sub eax, tmplptr
		mov wrappersize, eax
	}
//	cbptr = (DWORD) HACKWinProc;	//Hack @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	cbptr = (DWORD) safe_alloc(  wrappersize );	//calculer la taille de la default callback.
	memcpy((void*)cbptr, (void*)tmplptr, wrappersize);
	//Now patch 0x12345678 marked bytes by our CALLBACK struct pointer
	cbstr = (DWORD)&vm->callback.items[i];
	for(topatch=(DWORD*)cbptr;	//topatch = (DWORD*) memchr((void*)cbptr, 0x12345678, 1);
		*topatch!=0x12345678 && ((DWORD)topatch) < cbptr+wrappersize;
		((BYTE*)topatch)++){
	}
	if(topatch && *topatch == 0x12345678){
		*topatch = cbstr;
	}

	vm->callback.items[i].host_address = cbptr;
	//replace 0x12345678, &vm->callback.items[i]
	return cbptr;
start_block:
	__asm{
/*		;push eax
		push ecx
		push edx
		push ebx
		;push esp
		push ebp
		push esi
		push edi
*/
		push esp			; stack ptr
		push 0x12345678		; GENI_CALLBACK*
		;lea eax, geni_vm_create_sub_instance	;geni_vm_do_sub
		lea eax, geni_vm_do_sub
		call eax
		add esp, 8
		;eax contain our wanted value
		;ret 0x10						; THIS may be variable, depending on the callback arguments ?
/*		pop edi
		pop esi
		pop ebp
		;pop esp
		pop ebx
		pop edx
		pop ecx*/
		ret 4 * 4
	}
	/*
		//;push ebp
		//;mov ebp, esp
		//;mov eax, ebp
		//;add eax, 2
		//;push eax			; stack ptr
		push esp
		push 0x12345678		; GENI_CALLBACK*
		lea eax, geni_vm_do_sub
		call eax
		add esp, 8
		;eax contain our wanted value
		ret
		//;mov esp, ebp
		//;pop ebp
		//;ret 0x10
	*/
end_block: __asm{ nop }
	
}

void geni_vm_remove_callback( GENI_VM* vm, DWORD ip){
	DWORD i;
	for(i=0;i<vm->callback.count;i++){
		if(vm->callback.items[i].vm_ip == ip){
			//1) destroy the given callback
			safe_free( (void*) vm->callback.items[i].host_address );
			//2) move callbacks if needed
			if(vm->callback.count - i>0){
				memcpy(&vm->callback.items[i], &vm->callback.items[i+1], sizeof( GENI_CALLBACK ) * ( vm->callback.count - i )) ;
			}
			//3) realloc the memory
			vm->callback.count--;
			safe_realloc( &vm->callback.items, sizeof( GENI_CALLBACK ) * vm->callback.count );
			break;
		}
	}
}


DWORD* geni_vm_operand_ptrget32( GENI_VM* vm, GENI_OPERAND* op){
	//retrieve a pointer to the value of the given operand
	DWORD* valueptr;
	valueptr = &op->dword;
	switch( op->type ){
		case OK_CONSTANT:	//do nothing more
			break;
		case OK_REGISTER:	//get a pointer to the register case.
			valueptr = &vm->registers.r[ (*valueptr) & 0xFF ];
			break;
		case OK_VM_MEMORY:{
				//we must assert (*valueptr) is < memory.size !!!
				valueptr = (DWORD*)&vm->memory.ptr[ (*valueptr) ];
			}
			break;
		case OK_HOST_MEMORY:
			//printf("** DEBUG OK_HOST_MEMORY **\n\tvalueptr=%08x, *valueptr=%08x\n\t=>", valueptr, *valueptr);
			valueptr = (DWORD*)&(*valueptr);
			//printf("** DEBUG OK_HOST_MEMORY **\n\tvalueptr=%08x, *valueptr=%08x\n", valueptr, *valueptr);
			break;
		default:
			printf("geni_vm_operand_ptrget32: Error - unknow operand type !\n");
			return NULL;
			break;
	}
	switch(op->mode){
		case OM_NORMAL:
			break;
		case OM_REG_POINTER:	//get a pointer to the register case.
			valueptr = &vm->registers.r[ (*valueptr) & 0xFF ];
			break;
		case OM_VM_POINTER:{
				//we must assert (*valueptr) is < memory.size !!!
				valueptr = (DWORD*)&vm->memory.ptr[ (*valueptr) ];
			}
			break;
		case OM_HOST_POINTER:
			//Became a host - marker, no more pointer !
			//printf("** DEBUG OM_HOST_POINTER at ip = %08X **\n\tvalueptr=%08x, *valueptr=%08x\n\t=>", vm->code.ip, valueptr, *valueptr);
			valueptr = (DWORD*)(*valueptr);
			//printf("** DEBUG OM_HOST_POINTER **\n\tvalueptr=%08x, *valueptr=%08x\n", valueptr, *valueptr);
			break;
		default:
			printf("geni_vm_operand_ptrget32: Error - unknow operand mode !\n");
			return NULL;
			break;
	}
	return valueptr;
}

DWORD geni_vm_operand_get32( GENI_VM* vm, GENI_OPERAND* op){
	//retrieve the value from the given operand
	DWORD * value_ptr = geni_vm_operand_ptrget32(vm, op );
	if(value_ptr){
		return *value_ptr;
	}
	return -1;
}

void geni_vm_operand_set32( GENI_VM* vm, GENI_OPERAND* op, DWORD value){
	//retrieve the value from the given operand
	DWORD * value_ptr = geni_vm_operand_ptrget32( vm, op);
	if(value_ptr){
		*value_ptr = value;
	}
	//~ else{
		//~ printf("geni_vm_operand_set32: Error - bad pointer !\n");
	//~ }
}

void geni_vm_opcode_mov32( GENI_VM* vm, GENI_OPCODE* opcode) {
	geni_vm_operand_set32( vm, 
		&opcode->leftop, 
		geni_vm_operand_get32( vm, &opcode->rightop) );
}

void geni_vm_opcode_movzb( GENI_VM* vm, GENI_OPCODE* opcode) {
	geni_vm_operand_set32( vm, 
		&opcode->leftop, 
		geni_vm_operand_get32( vm, &opcode->rightop) & 0x0FF );
}

void geni_vm_opcode_movzw( GENI_VM* vm, GENI_OPCODE* opcode) {
	geni_vm_operand_set32( vm, 
		&opcode->leftop, 
		geni_vm_operand_get32( vm, &opcode->rightop) & 0x0FFFF );
}

void geni_vm_opcode_add32( GENI_VM* vm, GENI_OPCODE* opcode) {
	geni_vm_operand_set32( vm, 
		&opcode->leftop, 
		geni_vm_operand_get32( vm, &opcode->leftop) +
		geni_vm_operand_get32( vm, &opcode->rightop) );
}

void geni_vm_opcode_sub32( GENI_VM* vm, GENI_OPCODE* opcode) {
	geni_vm_operand_set32( vm, 
		&opcode->leftop, 
		geni_vm_operand_get32( vm, &opcode->leftop) -
		geni_vm_operand_get32( vm, &opcode->rightop) );
}

void geni_vm_opcode_mul32( GENI_VM* vm, GENI_OPCODE* opcode) {
	geni_vm_operand_set32( vm, 
		&opcode->leftop, 
		geni_vm_operand_get32( vm, &opcode->leftop) *
		geni_vm_operand_get32( vm, &opcode->rightop) );
}

void geni_vm_opcode_div32( GENI_VM* vm, GENI_OPCODE* opcode) {
	geni_vm_operand_set32( vm, 
		&opcode->leftop, 
		geni_vm_operand_get32( vm, &opcode->leftop) /
		geni_vm_operand_get32( vm, &opcode->rightop) );
}

void geni_vm_opcode_and32( GENI_VM* vm, GENI_OPCODE* opcode) {
	geni_vm_operand_set32( vm, 
		&opcode->leftop, 
		geni_vm_operand_get32( vm, &opcode->leftop) &
		geni_vm_operand_get32( vm, &opcode->rightop) );
}

void geni_vm_opcode_or32( GENI_VM* vm, GENI_OPCODE* opcode) {
	geni_vm_operand_set32( vm, 
		&opcode->leftop, 
		geni_vm_operand_get32( vm, &opcode->leftop) |
		geni_vm_operand_get32( vm, &opcode->rightop) );
}

DWORD xor(DWORD a, DWORD b){ return (a & (~b)) | ((~a) & b); }
void geni_vm_opcode_xor32( GENI_VM* vm, GENI_OPCODE* opcode) {
	DWORD a, b;
	a = geni_vm_operand_get32( vm, &opcode->leftop);
	b = geni_vm_operand_get32( vm, &opcode->rightop);
	geni_vm_operand_set32( vm, &opcode->leftop, ( (a & (~b)) | ((~a) & b) ) );
}

void geni_vm_opcode_not32( GENI_VM* vm, GENI_OPCODE* opcode) {
	geni_vm_operand_set32( vm, 
		&opcode->leftop, 
		~(geni_vm_operand_get32( vm, &opcode->rightop)) );
}
void geni_vm_opcode_push32( GENI_VM* vm, GENI_OPCODE* opcode) {
	//THIS VERSION IGNORE RIGHT ARGUMENTS
	// A futur implementation can allow to work on a range of values (where Left is start and Right content the count - items)
	
	//ATTENTION, il nous faut gérer deux piles !
	// une pour simuler la HOST stack !
	// 
	
	vm->stack.ptr[ vm->stack.sp ] = geni_vm_operand_get32( vm, &opcode->leftop);
	vm->stack.sp++;
	if(vm->stack.sp<0 || (vm->stack.sp >= vm->stack.size) ){
		printf("ip:%08X - 'push' got a stack overflow (sp=%08X, size=%08X)\n", vm->code.ip, vm->stack.sp, vm->stack.size);
#ifdef CAN_DUMP
		geni_vm_dump_stack( "Args Stack", &vm->stack );
#endif
#ifdef Debug
		if(IsDebuggerPresent()){
			DebugBreak();//STACKOVERFLOW !
		}
#endif
	}
}
void geni_vm_opcode_pop32( GENI_VM* vm, GENI_OPCODE* opcode) {
	//THIS VERSION IGNORE RIGHT ARGUMENTS
	// A futur implementation can allow to work on a range of values (where Left is start and Right content the count - items)
	DWORD value;
	vm->stack.sp--;
	if(vm->stack.sp<0 || (vm->stack.sp >= vm->stack.size) ){
		printf("ip:%08X - 'pop' got a stack overflow (sp=%08X, size=%08X)\n", vm->code.ip, vm->stack.sp, vm->stack.size);
#ifdef CAN_DUMP
		geni_vm_dump_stack( "Args Stack", &vm->stack );
#endif
#ifdef Debug
		if(IsDebuggerPresent()){
			DebugBreak();//STACKOVERFLOW !
		}
#endif
	}
	value = vm->stack.ptr[ vm->stack.sp ];
	geni_vm_operand_set32( vm, &opcode->leftop, value );
}
void geni_vm_opcode_jmp32( GENI_VM* vm, GENI_OPCODE* opcode) {
	//On doit vérifier si le MODE de RIGHT est VM ou HOST
	//Le reste de RIGHT sert d'offest dans LEFT
	//eg: jmp32 r0, h[0x7c800000] aura pour effet de tenter d'executer du code dans HOST à l'addresse 0x7c800000 + r0
	DWORD local_addr;
	local_addr = geni_vm_operand_get32( vm, &opcode->leftop);
	vm->code.ip = local_addr -1;
	//~ if(vm->flags.debug){
		//~ printf("(jumping to %08x)",vm->code.ip);
	//~ }
}

void geni_vm_opcode_hcall32( GENI_VM* vm, GENI_OPCODE* opcode) {
	GENI_OPCODE next_ip;
	DWORD i, count;
	DWORD host_address;
	DWORD return_value;
	DWORD* localstack;	//DWORD localstack[200];
	BOOL jump;
	jump = opcode->instruction == OC_HJUMP;
	count = geni_vm_operand_get32(vm, &opcode->rightop);	
	host_address = geni_vm_operand_get32(vm, &opcode->leftop);	//now got the host pointer
#ifdef VM_DEBUG
	printf("geni_vm_opcode_hcall32: ADDR=%08X, stack-args = %08X\n", host_address, count);
#endif
	localstack = safe_alloc(sizeof(DWORD) * count);
	OPCODE32(&next_ip,OC_NOP, CST32(0), UNDEF32);
	for(i=0;i<count;i++){
		geni_vm_opcode_pop32( vm, &next_ip );
		localstack[i] = next_ip.leftop.dword;
#ifdef VM_DEBUG
	printf("\t%d: %08X\n", i, localstack[i]);
#endif
	}
	__asm{
		pushad
		mov esi, localstack		
//		lea esi, localstack		//to use when declared as DWORD localstack[100]
		mov edi, esp
		mov ecx, count
		or ecx,ecx
		jz no_args
push_again:	lodsd
		push eax
		dec ecx
		jnz push_again
no_args:
		mov eax, host_address
		mov ecx, jump
		or ecx,ecx
		jz docall
		jmp eax					; bye bye ! (stack corruption is surely there !)
docall:	call eax
		cmp esp, edi
		jz continue_normal
		mov esp, edi			;// Hack for some difference between Stdcall and Cdecl...
continue_normal:
		;did we have something to do with the stack at this point ?
		;depending on stdcall / decl.
		mov return_value, eax
		popad
	}
#ifdef VM_DEBUG
	printf("\t=> %08X\n", return_value );
#endif
	OPCODE32(&next_ip,OC_NOP, REG32(0), UNDEF32);
	geni_vm_operand_set32(vm, &next_ip.leftop, return_value);
	safe_free(localstack);
	return;
}



void geni_vm_opcode_call32( GENI_VM* vm, GENI_OPCODE* opcode) {
	//THIS VERSION IGNORE RIGHT ARGUMENTS
	GENI_OPCODE jmp_addr;	
#ifdef VM_DEBUG_STACK
	printf("stack trace: IP %08X : CALL (sp=%08X)\n", vm->code.ip, vm->stack.sp);
#endif
	//do a PUSH of IP+1 
	vm->callstack.ptr[ vm->callstack.sp ] = vm->code.ip + 1;
	vm->callstack.sp++;
	//then a JMP
	OPCODE32( &jmp_addr, OC_JMP, 
		opcode->leftop.size, opcode->leftop.type, opcode->leftop.mode, opcode->leftop.dword,
		opcode->rightop.size, opcode->rightop.type, opcode->rightop.mode, opcode->rightop.dword );
	geni_vm_opcode_jmp32( vm, &jmp_addr );
}
void geni_vm_opcode_ret32( GENI_VM* vm, GENI_OPCODE* opcode) {
	//THIS VERSION IGNORE RIGHT ARGUMENTS
	//GENI_OPCODE next_ip;
	//~ GENI_OPCODE jmp_addr;
	//do a POP of IP+1 
//	OPCODE32( &next_ip, OC_POP, CST32( 0 ) /* this is likely a local variable :) */, UNDEF32);
//	geni_vm_opcode_pop32( vm, &next_ip );
//	vm->code.ip = next_ip.leftop.dword;
#ifdef VM_DEBUG_STACK
	printf("stack trace: IP %08X : RET (sp=%08X)\n", vm->code.ip, vm->stack.sp);
#endif
	vm->return_value = geni_vm_operand_get32(vm, &opcode->leftop );
	vm->callstack.sp--;
	if(vm->callstack.sp>=0){
		vm->code.ip = vm->callstack.ptr[ vm->callstack.sp ] - 1;
	}
	else{
		//This will fake a "stop 0,0"
		vm->code.ip = vm->code.size +1;
	}
	//then a JMP to ip...
	//~ OPCODE32( &jmp_addr, OC_JMP, 
		//~ opcode->leftop.size, opcode->leftop.type, opcode->leftop.mode, opcode->leftop.dword,
		//~ opcode->rightop.size, opcode->rightop.type, opcode->rightop.mode, opcode->rightop.dword );
	//~ geni_vm_opcode_jmp32( vm, &jmp_addr );
}

void geni_vm_opcode_cmp32( GENI_VM* vm, GENI_OPCODE* opcode){ 
	//met à jour les flags zero, greater et lower.
	DWORD a, b;
	a = geni_vm_operand_get32( vm, &opcode->leftop);
	b = geni_vm_operand_get32( vm, &opcode->rightop);
	vm->flags.zero = ( a == b );
	vm->flags.greater = ( a > b );
	vm->flags.lower = ( a < b );
}
void geni_vm_opcode_jz32( GENI_VM* vm, GENI_OPCODE* opcode){
	GENI_OPCODE jmp_addr;
	GENI_OPERAND *a, *b;
	if(vm->flags.zero){
		a = &opcode->leftop;
		b = &opcode->rightop;
	}
	else{
		a = &opcode->rightop;
		b = &opcode->leftop;
	}
	OPCODE32( &jmp_addr, OC_JMP, 
		a->size, a->type, a->mode, a->dword,
		b->size, b->type, b->mode, b->dword );
	geni_vm_opcode_jmp32( vm, &jmp_addr );
}
void geni_vm_opcode_jnz32( GENI_VM* vm, GENI_OPCODE* opcode){ 
	GENI_OPCODE jmp_addr;
	GENI_OPERAND *a, *b;
	
	if(!vm->flags.zero){
		a = &opcode->leftop;
		b = &opcode->rightop;
	}
	else{
		a = &opcode->rightop;
		b = &opcode->leftop;
	}
	OPCODE32( &jmp_addr, OC_JMP, 
		a->size, a->type, a->mode, a->dword,
		b->size, b->type, b->mode, b->dword );
	geni_vm_opcode_jmp32( vm, &jmp_addr );
}
void geni_vm_opcode_jg32( GENI_VM* vm, GENI_OPCODE* opcode){ 
	GENI_OPCODE jmp_addr;
	GENI_OPERAND *a, *b;
	if(vm->flags.greater){
		a = &opcode->leftop;
		b = &opcode->rightop;
	}
	else{
		a = &opcode->rightop;
		b = &opcode->leftop;
	}
	OPCODE32( &jmp_addr, OC_JMP, 
		a->size, a->type, a->mode, a->dword,
		b->size, b->type, b->mode, b->dword );
	geni_vm_opcode_jmp32( vm, &jmp_addr );	
}
void geni_vm_opcode_jl32( GENI_VM* vm, GENI_OPCODE* opcode){
	GENI_OPCODE jmp_addr;
	GENI_OPERAND *a, *b;
	if(vm->flags.lower){
		a = &opcode->leftop;
		b = &opcode->rightop;
	}
	else{
		a = &opcode->rightop;
		b = &opcode->leftop;
	}
	OPCODE32( &jmp_addr, OC_JMP, 
		a->size, a->type, a->mode, a->dword,
		b->size, b->type, b->mode, b->dword );
	geni_vm_opcode_jmp32( vm, &jmp_addr );
}
void geni_vm_opcode_jge32( GENI_VM* vm, GENI_OPCODE* opcode){
	GENI_OPCODE jmp_addr;
	GENI_OPERAND *a, *b;
	if(vm->flags.zero || vm->flags.greater){
		a = &opcode->leftop;
		b = &opcode->rightop;
	}
	else{
		a = &opcode->rightop;
		b = &opcode->leftop;
	}
	OPCODE32( &jmp_addr, OC_JMP, 
		a->size, a->type, a->mode, a->dword,
		b->size, b->type, b->mode, b->dword );
	geni_vm_opcode_jmp32( vm, &jmp_addr );
}
void geni_vm_opcode_jle32( GENI_VM* vm, GENI_OPCODE* opcode){ 
	GENI_OPCODE jmp_addr;
	GENI_OPERAND *a, *b;
	if(vm->flags.zero || vm->flags.lower){
		a = &opcode->leftop;
		b = &opcode->rightop;
	}
	else{
		a = &opcode->rightop;
		b = &opcode->leftop;
	}
	OPCODE32( &jmp_addr, OC_JMP, 
		a->size, a->type, a->mode, a->dword,
		b->size, b->type, b->mode, b->dword );
	geni_vm_opcode_jmp32( vm, &jmp_addr );
}

void geni_vm_opcode_vm2h( GENI_VM* vm, GENI_OPCODE* opcode){ 
	//take operand b and convert the VM pointer to HOST pointer (MIT lic !) in op a
	DWORD a, b;
	b = geni_vm_operand_get32( vm, &opcode->rightop);
	if(vm->flags.debug && b>vm->memory.size){
		printf("ip: %08x, vm2h: got an vm pointer (%08x) outside of VM memory boundaries (%08x)!\n", vm->code.ip, b, vm->memory.size);
	}
	a = ((DWORD)(vm->memory.ptr)) + b;
	geni_vm_operand_set32( vm, &opcode->leftop, a);
}

void geni_vm_opcode_h2vm( GENI_VM* vm, GENI_OPCODE* opcode){ 
	//take operand b and convert the HOST pointer to VM pointer (MIT lic !) in op a
	DWORD a, b;
	b = geni_vm_operand_get32( vm, &opcode->rightop);
	a = b - ((DWORD)(vm->memory.ptr));
	if(vm->flags.debug && a>vm->memory.size){
		printf("ip: %08x, h2vm: got an vm pointer (%08x) outside of VM memory boundaries (%08x)!\n", vm->code.ip, a, vm->memory.size);
	}
	geni_vm_operand_set32( vm, &opcode->leftop, a);
}

void geni_vm_opcode_printl( GENI_VM* vm, GENI_OPCODE* opcode){
	//en fonction de la valeur dans rightop, on aura :
	// 's', 'c', 'd', 'x', ...
	DWORD a;
	char format_str[4] = { '%', '?', '\n', 0 };	
	format_str[1] = opcode->rightop.byte;
	a = geni_vm_operand_get32(vm, &opcode->leftop);
	printf(format_str, a);
}
void geni_vm_opcode_print( GENI_VM* vm, GENI_OPCODE* opcode){
	//en fonction de la valeur dans rightop, on aura :
	// 's', 'c', 'd', 'x', ...
	DWORD a;
	char format_str[3] = { '%', '?', 0 };
	format_str[1] = opcode->rightop.byte;
	a = geni_vm_operand_get32(vm, &opcode->leftop);
	printf(format_str, a);
}
void geni_vm_opcode_stop( GENI_VM* vm, GENI_OPCODE* opcode){
	vm->code.stop = TRUE;
	vm->return_value = geni_vm_operand_get32(vm, &opcode->leftop );
}
void geni_vm_opcode_shl( GENI_VM* vm, GENI_OPCODE* opcode){
	geni_vm_operand_set32( vm, 
		&opcode->leftop, 
		geni_vm_operand_get32( vm, &opcode->leftop) <<
		geni_vm_operand_get32( vm, &opcode->rightop) );
}
void geni_vm_opcode_shr( GENI_VM* vm, GENI_OPCODE* opcode){
	geni_vm_operand_set32( vm, 
	&opcode->leftop, 
	geni_vm_operand_get32( vm, &opcode->leftop) >>
	geni_vm_operand_get32( vm, &opcode->rightop) );
}
//void geni_vm_opcode_finddw( GENI_VM* vm, GENI_OPCODE* opcode){
//	//Find the dword matching leftop.dword from rightop (incrementing rightop value by 4), returning rightop modified.
//}
void geni_vm_opcode_nop( GENI_VM* vm, GENI_OPCODE* opcode){ /* DO NOTHING ! */ }

void geni_vm_opcode_mkcb( GENI_VM* vm, GENI_OPCODE* opcode){
	DWORD cbptr;
	cbptr = geni_vm_make_callback(vm, 
			geni_vm_operand_get32( vm, &opcode->leftop),
			geni_vm_operand_get32( vm, &opcode->rightop) );
	geni_vm_operand_set32( vm, &opcode->leftop, cbptr);
}

void geni_vm_opcode_rmcb( GENI_VM* vm, GENI_OPCODE* opcode){
	geni_vm_remove_callback(vm, 
			geni_vm_operand_get32( vm, &opcode->leftop) );
}

void geni_vm_opcode_dump( GENI_VM* vm, GENI_OPCODE* opcode){
#ifdef CAN_DUMP
	geni_vm_dump_memory( geni_vm_operand_get32( vm, &opcode->leftop), 
				0, 
				geni_vm_operand_get32( vm, &opcode->rightop ) );
#endif
}

void geni_vm_opcode_debugbreak( GENI_VM* vm, GENI_OPCODE* opcode){
#ifdef Debug
	if(IsDebuggerPresent()){
		DebugBreak();
	}
#endif
}

void geni_vm_opcode_sprintf( GENI_VM* vm, GENI_OPCODE* opcode){
	GENI_OPCODE arg_op;
	char *buffer, *pattern;
	buffer=(char*)geni_vm_operand_get32( vm, &opcode->leftop);
	pattern=(char*)geni_vm_operand_get32( vm, &opcode->rightop);
	OPCODE32(&arg_op,OC_POP,CST32(0),CST32(0));
	geni_vm_opcode_pop32(vm, &arg_op);
#pragma warning(push)
#pragma warning(disable:4996)
	sprintf(buffer,pattern,arg_op.leftop.dword);
#pragma warning(pop)
}

void geni_vm_opcode_api( GENI_VM* vm, GENI_OPCODE* opcode){
	DWORD i, arg, id;
	id = geni_vm_operand_get32( vm, &opcode->leftop);
	arg= geni_vm_operand_get32( vm, &opcode->rightop);
	for(i=0;i<vm->api.count;i++){
		if(vm->api.items[i].id==id){
			i = vm->api.items[i].handler((DWORD*)vm, arg);
			//geni_vm_operand_set32(vm, &opcode->rightop, i);
			vm->registers.r[0] = i;
			return;
		}
	}
#ifdef VM_DEBUG
	printf("Calling an undefined API(%08X) !\n", id);
#endif
	// leftop = api reference
	// rightop = direct argument (without pushing anything)
	// An API will be (typically) registered in HOST code,
	// but we could imagine another opcode that register/unregister an API
	//
	// API struct must keep trace of the following:
	// - name (in debug mode)
	// - origine ( HOST / VM )
	// - handler address (or IP in VM)
	// - number of arguments to automatically pop ?
	// proto of host_api_handlers:
	// DWORD geni_vm_api_APINAME( GENI_VM* vm )
	//	-> the return value will be put in the r0 register.
}

GENI_VM* geni_vm_init(){
  DWORD fs_30;
	GENI_VM* vm;
	HANDLE heap;
    
	heap = HeapCreate(/*HEAP_CREATE_ENABLE_EXECUTE|*/HEAP_GENERATE_EXCEPTIONS, 0, 0);
	vm = safe_alloc( sizeof( GENI_VM ) );
#ifdef VM_DEBUG
	OutputDebugString("-> geni_vm_init");
#endif
	if(vm){
		vm->heap = heap;
		vm->code.size = 0;
		vm->code.ip = 0;
		vm->code.opcodes = NULL;
		vm->code.stop = FALSE;
		vm->flags.critical = FALSE;
		vm->flags.zero = FALSE;
		vm->flags.lower = FALSE;
		vm->flags.greater = FALSE;		
//#ifdef VM_DEBUG
//		vm->flags.debug = TRUE;
//#else
		vm->flags.debug = FALSE;
//#endif

		vm->memory.size = DEFAULT_MEMORY_SIZE;
		vm->memory.ptr = (BYTE*)safe_alloc( vm->memory.size );
		vm->stack.size = DEFAULT_STACK_SIZE;
		vm->stack.ptr = (DWORD*)safe_alloc( sizeof(DWORD) * vm->stack.size );
		vm->stack.sp = 0;	//From 0 to (size / 4) !
		vm->callstack.size = DEFAULT_CALLSTACK_SIZE;
		vm->callstack.ptr = (DWORD*)safe_alloc( sizeof(DWORD) * vm->callstack.size );
		vm->callstack.sp = 0;	//From 0 to (size / 4) !
		vm->callback.count = 0;
		vm->callback.items = NULL;
    //A small bit of hack: Register 0 always start with the PEB* pointer :)
    __asm{
      mov eax, FS:[0x30]
      mov fs_30, eax
    }
    vm->registers.r[0] = fs_30;
    

		safe_semaphore_init(vm);
				
		//Attention, il faut penser à définir le mapping des instructions !
		// entre [ instruction code ] et [ handler ]
		MAP_OPCODE( vm, geni_vm_opcode_nop, OC_NOP, "nop");
		MAP_OPCODE( vm, geni_vm_opcode_mov32, OC_MOV, "mov");
		MAP_OPCODE( vm, geni_vm_opcode_add32, OC_ADD, "add");
		MAP_OPCODE( vm, geni_vm_opcode_sub32, OC_SUB, "sub");
		MAP_OPCODE( vm, geni_vm_opcode_mul32, OC_MUL, "mul");
		MAP_OPCODE( vm, geni_vm_opcode_div32, OC_DIV, "div");
		MAP_OPCODE( vm, geni_vm_opcode_and32, OC_AND, "and");
		MAP_OPCODE( vm, geni_vm_opcode_or32, OC_OR, "or");
		MAP_OPCODE( vm, geni_vm_opcode_xor32, OC_XOR, "xor");
		MAP_OPCODE( vm, geni_vm_opcode_not32, OC_NOT, "not");
		MAP_OPCODE( vm, geni_vm_opcode_not32, OC_NEG, "neg");

		MAP_OPCODE( vm, geni_vm_opcode_push32, OC_PUSH, "push");
		MAP_OPCODE( vm, geni_vm_opcode_pop32, OC_POP, "pop");
		MAP_OPCODE( vm, geni_vm_opcode_jmp32, OC_JMP, "jmp");
		MAP_OPCODE( vm, geni_vm_opcode_call32, OC_CALL, "call");
		MAP_OPCODE( vm, geni_vm_opcode_hcall32, OC_HCALL, "hcall");
		MAP_OPCODE( vm, geni_vm_opcode_ret32, OC_RET, "ret");
		
		MAP_OPCODE( vm, geni_vm_opcode_cmp32, OC_CMP, "cmp");
		MAP_OPCODE( vm, geni_vm_opcode_jz32, OC_JZ, "jz");
		MAP_OPCODE( vm, geni_vm_opcode_jnz32, OC_JNZ, "jnz");
		MAP_OPCODE( vm, geni_vm_opcode_jg32, OC_JG, "jg");
		MAP_OPCODE( vm, geni_vm_opcode_jl32, OC_JL, "jl");
		MAP_OPCODE( vm, geni_vm_opcode_jge32, OC_JGE, "jge");
		MAP_OPCODE( vm, geni_vm_opcode_jle32, OC_JLE, "jle");

		MAP_OPCODE( vm, geni_vm_opcode_vm2h, OC_VM2H, "vm2h");
		MAP_OPCODE( vm, geni_vm_opcode_h2vm, OC_H2VM, "h2vm");
		MAP_OPCODE( vm, geni_vm_opcode_printl, OC_PRINTL, "printl");
		MAP_OPCODE( vm, geni_vm_opcode_print, OC_PRINT, "print");
		MAP_OPCODE( vm, geni_vm_opcode_stop, OC_STOP, "stop");
		MAP_OPCODE( vm, geni_vm_opcode_movzb, OC_MOVZB, "movzb");
		MAP_OPCODE( vm, geni_vm_opcode_movzw, OC_MOVZW, "movzw");
		MAP_OPCODE( vm, geni_vm_opcode_shl, OC_SHL, "shl");
		MAP_OPCODE( vm, geni_vm_opcode_shr, OC_SHR, "shr");
		MAP_OPCODE( vm, geni_vm_opcode_mkcb, OC_MKCB, "mkcb");
		MAP_OPCODE( vm, geni_vm_opcode_rmcb, OC_RMCB, "rmcb");
		MAP_OPCODE( vm, geni_vm_opcode_dump, OC_DUMP, "dump");
		MAP_OPCODE( vm, geni_vm_opcode_debugbreak, OC_DEBUGBREAK, "debugbreak");
		MAP_OPCODE( vm, geni_vm_opcode_sprintf, OC_SPRINTF, "sprintf");
		MAP_OPCODE( vm, geni_vm_opcode_api, OC_API, "api");
		MAP_OPCODE( vm, geni_vm_opcode_hcall32, OC_HJUMP, "hjmp");
		//MAP_OPCODE( vm, geni_vm_opcode_finddw, OC_FINDDW, "finddw");

		//...
	}
	return vm;
}

void geni_vm_set_userdata( GENI_VM* vm, void* userdata){
	if(vm){
		vm->userdata = userdata;
	}
}
void* geni_vm_get_userdata( GENI_VM* vm){
	return vm?vm->userdata:NULL;
}

void geni_vm_delete( GENI_VM* vm ){
#ifdef VM_DEBUG
	OutputDebugString("-> geni_vm_delete");
#endif
	if(vm){
		DWORD i;
		HANDLE heap;
		vm->code.stop = TRUE;
		safe_semaphore_deinit(vm);
		//foreach callback, free them
		for(i=0;i<vm->callback.count;i++){
			geni_vm_remove_callback( vm, i );
		}
		safe_free( vm->api.items );
		safe_free( vm->callback.items );
		safe_free( vm->code.opcodes );
		safe_free( vm->memory.ptr );
		safe_free( vm->stack.ptr );
		safe_free( vm->callstack.ptr );		
		heap = vm->heap;
		safe_free( vm );		
		HeapDestroy( heap);
	}
}

void geni_vm_run( GENI_VM* vm ){
	GENI_OPCODE* opcode;
	fnHandler* handler;
#ifdef VM_DEBUG
	OutputDebugString("-> geni_vm_run");
#endif
	if(!vm) return;
	if(!vm->code.size){
		printf("no code to run\n");
		return;
	}
	if(!vm->code.opcodes){
		printf("bizzar opcodes !\n");
		return;
	}
	
	if(vm->code.ip >= vm->code.size){//assert ip is at 0 or less than size
		printf("Code already executed, you may forgot to rewind ?\n");
		return;
	}
	
	//run code
			/* 
				CE TEST NE PERMET PAS D'EXECUTER UNE 
				PORTION EXTERIEUR DE CODE VM !!
			/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
			*/
	vm->code.stop = FALSE;
	for( ; vm->code.ip < vm->code.size && !vm->code.stop; vm->code.ip++ ){
		//dispatch opcodes
		opcode = &vm->code.opcodes[ vm->code.ip ];
#ifdef CAN_DUMP
		if(vm->flags.debug){
			printf("ip: %08X - ", vm->code.ip );
			geni_vm_dump_opcode( vm, opcode );
		}	
#endif
		handler = (fnHandler*)vm->opcodeset[ opcode->instruction ].handler;
		if(/*opcode->instruction > MAX_OPCODES && : on pourra enrichir le bytecode !*/ !handler){
			printf("Invalid opcode %02x at IP=%08X!\n", opcode->instruction, vm->code.ip);
			exit(255);
		}
		if(!vm->flags.critical) safe_semaphore_begin(vm);
		handler( vm, opcode );
		if(!vm->flags.critical) safe_semaphore_end(vm);
	}
}

BOOL geni_vm_load_code( GENI_VM* vm, void* buffer, DWORD buffersize ){
	//Verifier que buffersize et un multiple de sizeof( GENI_OPCODE )
#ifdef VM_DEBUG
	OutputDebugString("-> geni_vm_load_code");
#endif
	vm->code.size = buffersize / sizeof( GENI_OPCODE );
	if( vm->code.size * sizeof( GENI_OPCODE ) != buffersize ){
		printf("geni_vm_load_code: buffer size must be a multiple of %d !\n", sizeof( GENI_OPCODE ) );
		return FALSE;
	}
	if( vm->code.opcodes = safe_alloc( buffersize ) ){
		memcpy( vm->code.opcodes, buffer, buffersize );
	}
	else{
		printf("Could not allocate enought memory !\n");
		return FALSE;
	}
	return TRUE;
}
void geni_vm_register_api(GENI_VM* vm, fnAPIHandler* apihandler, DWORD id, char* name){
	//to code !
	safe_realloc(&vm->api.items, (++vm->api.count) * sizeof(GENI_API) );
	vm->api.items[vm->api.count -1].id = id;
	vm->api.items[vm->api.count -1].handler = apihandler;
	vm->api.items[vm->api.count -1].name = name;
}//void geni_vm_unregister_api(GENI_VM* vm, DWORD id){	//}


BOOL load_gvm_data(GENI_VM* vm, BYTE* binarydata){
	BOOL ok = TRUE;
	GENI_OPCODE* code_seg = NULL;
	GENI_BINARY_HEADER* fheader = NULL;
	DWORD memory_size = 0;
	DWORD instruction_count = 0;
	//TODO test on version number
	fheader = (GENI_BINARY_HEADER*)binarydata;
	//load binary directly from inputfile
	memory_size = fheader->memory_size;
	instruction_count = fheader->instruction_count;
	//read memory block (if any)
	if(fheader->memory_size > vm->memory.size){
		vm->memory.size = fheader->memory_size;
		safe_realloc( &vm->memory.ptr, fheader->memory_size );
	}
	if(fheader->memory_stored){
#ifdef HAS_CORE
		if(verbose) printf("Loading initial memory state..");
#endif
		memcpy( vm->memory.ptr, &fheader->data, fheader->memory_stored);
#ifdef HAS_CORE
		if(verbose) printf("ok\n");
#endif
	}
	vm->memory.storedbytes = fheader->memory_stored;
	//read code block
#ifdef HAS_CORE
	if(verbose) printf("Loading code..");
#endif
	code_seg = (GENI_OPCODE*) (((DWORD)&fheader->data) + fheader->memory_stored );
	geni_vm_load_code( vm, code_seg, sizeof(GENI_OPCODE) * instruction_count );
#ifdef HAS_CORE
	if(verbose) printf("ok\n");
#endif
	return ok;
}

BOOL load_file(char* inputfile,DWORD* filesize, void** filebuffer){
	FILE* h = NULL;
	fopen_s(&h,inputfile, "rb");
	if(!h){
		return FALSE;
	}
	fseek(h, 0, SEEK_END );	//go to the end
	*filesize = ftell(h);
	fseek(h, 0, SEEK_SET);	//go to the begining
	*filebuffer = safe_alloc( 1 + *filesize );
	if(!*filebuffer){
		//could not allocate memory
		return FALSE;
	}
	//only assert the last byte is NULL
	((BYTE*)*filebuffer)[(*filesize) -1] = 0;
	fread(*filebuffer, *filesize, 1, h);
	fclose(h);
	return TRUE;
}

BOOL load_gvm_file(GENI_VM* vm, char* binaryfile){
	BYTE* filebuffer = NULL;
	DWORD filesize=0;
	BOOL ok = TRUE;
	if(!load_file(binaryfile,&filesize,(VOID*)&filebuffer)){
		printf("Could not read input binary file %s!\n",binaryfile);
		return FALSE;
	}
	ok = load_gvm_data( vm, filebuffer );
	safe_free(filebuffer);
	return ok;
}

#ifdef CAN_DUMP
//############ Debuging feature ( Dumping functions ) ##################
void geni_vm_dump_stack(char* name, GENI_STACK* stack ){
	DWORD i;
	printf("%s:\tsp: 0x%08x\n", name, stack->sp);
	for(i=0;i<stack->size;i+=4){
		printf("\t%03x: 0x%08x", i, stack->ptr[i]);
		printf(" %03x: 0x%08x", i+1, stack->ptr[i+1]);
		printf(" %03x: 0x%08x", i+2, stack->ptr[i+2]);
		printf(" %03x: 0x%08x\n", i+3, stack->ptr[i+3]);
	}
}
void geni_vm_dump_memory(DWORD address, DWORD addr_offset, DWORD size){
	long i = 0, l = 0, base = 0, page = 0x100;
	unsigned char curr_char;
	unsigned char row[] = { 0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0, 
					0 
				};

	base = address;
	page = size;	//0x400;
//~ reprise:
	l=0;
	printf("\t%08X - ", addr_offset + base - address);
	for(i=base;i<base + page;i++){
		__asm{
			push eax
			mov eax, addr_offset
			add eax, i
			mov al, byte ptr [eax]
			mov curr_char, al
			pop eax
		}
		printf("%02X ", curr_char );
		if(curr_char<32 || curr_char>254){
			curr_char = ' ';
		}
		row[l++]=curr_char;
		if(l>=16){
			row[l]=0;
			l=0;
			printf(" %s\n\t%08X - ", row, addr_offset + i + 1 - address );
		}
	}
	base += page;
	//~ scanf("%s",row);
	//~ if(row[0]=='1'){
		//~ goto reprise;
	//~ }
	printf("...\n");
}

void geni_vm_dump_operand( GENI_OPERAND* operand ){
	if(operand->mode == OM_HOST_POINTER){
		printf("h[");
	}
	else if(operand->mode == OM_REG_POINTER){
		printf("r[");
	}
	else if(operand->mode == OM_VM_POINTER){
		printf("m[");
	}
	switch(operand->type){
		case OK_CONSTANT:
			printf("0x%x",operand->dword);
			break;
		case OK_REGISTER:
			printf("r%d",operand->dword);
			break;
		case OK_VM_MEMORY:
			printf("m%d",operand->dword);
			break;
		case OK_HOST_MEMORY:
			printf("h%d",operand->dword);
			break;
		default:
			printf("???");
			break;
	}
	if(operand->mode != OM_NORMAL){
		printf("]");
	}
	//~ printf("@%d", operand->size );
}

void geni_vm_dump_opcode( GENI_VM* vm, GENI_OPCODE* opcode ){
	printf("%s ", 
#ifdef CAN_COMPILE
		vm->opcodeset[ opcode->instruction ].mnemonic 
#else
		"???"
#endif
		);
	geni_vm_dump_operand( &opcode->leftop );
	printf(", ");
	geni_vm_dump_operand( &opcode->rightop);
	//~ printf("@%d", opcode->size );
	printf("\n");
}

void geni_vm_dump_code( GENI_VM* vm, GENI_OPCODE* code, DWORD size, DWORD ip){
	DWORD i;
	printf("\tip: %08x (%d instructions)\n", ip, size);
	for(i=0;i<size;i++){
		printf("%08X: ", i);
		geni_vm_dump_opcode( vm, &code[i] );
	}
}
void geni_vm_dump( GENI_VM* vm){
	DWORD i;
	printf("Dumping VM:\n");
	//dump information about the VM
	printf("Dumping Code:\n");
	geni_vm_dump_code( vm, vm->code.opcodes, vm->code.size, vm->code.ip );
	// memory ( may be long )
	printf("Memory:\n");
	geni_vm_dump_memory( (DWORD)vm->memory.ptr, 0, vm->memory.size );
	
	// flags
	printf("Flags:\n\tZero:%1d\n\tGreater:%1d\n\tLower:%1d\n\tDebug:%1d\n", 
		vm->flags.zero, 
		vm->flags.greater, 
		vm->flags.lower, 
		vm->flags.debug
		);
	// registers
	printf("Registers:\n");
	for(i=0;i<MAX_REGISTER;i+=4){
		printf("\tr%03d: 0x%08x", i, vm->registers.r[i]);
		printf("\tr%03d: 0x%08x", i+1, vm->registers.r[i+1]);
		printf("\tr%03d: 0x%08x", i+2, vm->registers.r[i+2]);
		printf("\tr%03d: 0x%08x\n", i+3, vm->registers.r[i+3]);
	}
	// stack( sp, ... )
	geni_vm_dump_stack("Args Stack", &vm->stack);
	geni_vm_dump_stack("Call Stack", &vm->callstack);
	//Callbacks ?
	//Semaphore ?
	//Memory / Heap ?
}

#endif
#ifdef CAN_COMPILE
void geni_vm_compile_error(GENI_COMPILER_CONTEXT* context, char** codebufferptr, char* msg){
	char* linestr = *codebufferptr;
	while(*linestr && *linestr!='\n'){
		linestr++;
	}
	*linestr=0;
	if(msg){
		printf("line %d: %s\n\t^--- Syntax error: %s\n", context->current_line, *codebufferptr, msg);
	}
	else{
		printf("line %d: %s\n\t^--- Syntax error\n", context->current_line, *codebufferptr);
	}
}

BOOL geni_vm_compile_is_keyword(GENI_COMPILER_CONTEXT* context, char* name, DWORD len){
	long i;
	DWORD mlen;
	switch(len){
		case 4:
			if(_strnicmp(name, "word", len)==0) return TRUE;
			if(_strnicmp(name, "byte", len)==0) return TRUE;
			break;
		case 5:
			if(_strnicmp(name, "dword", len)==0) return TRUE;
			break;
		case 6:
			if(_strnicmp(name, "string", len)==0) return TRUE;
			break;
	}
	//if match [hmr]\d+ : disallowed !
	if(*name=='R'||*name=='r'||*name=='m'||*name=='M'||*name=='h'||*name=='H'){
		char* reserved = name + 1;
		i=len;
		while(--i){
			if(*reserved>='0' && *reserved<='9'){
				reserved++;
			}
			else{
				break;
			}
		}
		if(!i) 
			return TRUE;
	}
	for(i=0;i<MAX_OPCODES;i++){
		mlen = strlen( context->vm->opcodeset[i].mnemonic );
		if(len==mlen && _strnicmp(name, context->vm->opcodeset[i].mnemonic, mlen)==0) 
			return TRUE;
	}
	return FALSE;
}

void geni_vm_compile_gen_ref(GENI_COMPILER_CONTEXT* context, GENI_SEGMENT* segment, DWORD* address, DWORD* source_line, BOOL final){
	GENI_OPCODE dummy;
	*segment = context->current_segment;
	if( context->current_segment==SEG_DATA ){
		*address = context->memory_current_offset;
	}
	else if(final){
		*address = context->instruction_count;
	}
	else{
		*address = context->instruction_count * sizeof(GENI_OPCODE)
			+ ((DWORD)(context->is_operand_left ? &dummy.leftop.dword : &dummy.rightop.dword) )
			- (DWORD)(&dummy);
	}
	*source_line = context->current_line;
}

void geni_vm_compile_newname_ref(GENI_COMPILER_CONTEXT* context, GENI_COMPILER_NAMESPACE* name){
	GENI_COMPILER_NAMESPACE_REF* ref;
	name->ref_count++;
	safe_realloc(&name->references, sizeof(GENI_COMPILER_NAMESPACE_REF) * name->ref_count);
	ref = &name->references[ name->ref_count -1 ];
	geni_vm_compile_gen_ref(context, &ref->segment, &ref->address, &ref->source_line, FALSE);
}

GENI_COMPILER_NAMESPACE* geni_vm_compile_new_name(GENI_COMPILER_CONTEXT* context, char* label_decl, DWORD label_len){
	GENI_COMPILER_NAMESPACE* name;
	context->name_count++;
	safe_realloc( &context->names, context->name_count * sizeof(GENI_COMPILER_NAMESPACE) );	
	name = &context->names[ context->name_count -1 ];

	name->name = safe_alloc(label_len+1);
	strncpy_s(name->name,label_len+1, label_decl,label_len);	//Must copy the NULL termination char !
	name->address = -1;
	name->line_defined = -1;
	name->references = NULL;
	name->ref_count = 0;
	name->segment = SEG_UNKNOW;
	return name;
}

GENI_COMPILER_NAMESPACE* geni_vm_compile_get_name(GENI_COMPILER_CONTEXT* context, char* label_decl, DWORD label_len){	
	DWORD i;
	for(i=0;i<context->name_count;i++){
		if(_strnicmp(context->names[i].name, label_decl, label_len)==0){
			return &context->names[i];
		}
	}
	return (GENI_COMPILER_NAMESPACE*)NULL;
}

BOOL geni_vm_compile_have_label_declaration(GENI_COMPILER_CONTEXT* context, char** codebufferptr ){	
	BOOL label_def = FALSE;
	//If yes, then it will be automatically added into the names and the buffer will be consummed.
	char* code = *codebufferptr;
	while(*code && *code!=':'){
		if((*code>='A' && *code<='Z')
			|| (*code>='a' && *code<='z')
			|| (*code=='_')
			|| (code>*codebufferptr && (*code>='0' && *code<='9'))){
			code++;
		}
		else{
			break;	//this is not a label declaration
		}
	}
	label_def = (*code==':') && (code > *codebufferptr);
	if(label_def){
		GENI_COMPILER_NAMESPACE* name;
		DWORD label_len;
		label_len = (DWORD)code - (DWORD)*codebufferptr;
		//search for it in the namespaces
		if(name = geni_vm_compile_get_name(context,*codebufferptr, label_len)){
			//assert it is still empty!!! : avoid duplication of labels !)
			if(name->segment != SEG_UNKNOW){
				//Label already defined !
				char buffer[64];
				sprintf_s(buffer,64,"label already defined at line %d!", name->line_defined);
				geni_vm_compile_error(context,codebufferptr,buffer);
				//continue with previous declaration !
			}
		}
		else{
			//add it to the list of label if not found
			name = geni_vm_compile_new_name(context,*codebufferptr, label_len);
		}
		//patch it's definition 
		geni_vm_compile_gen_ref(context,&name->segment, &name->address, &name->line_defined, TRUE);
		*codebufferptr = code + 1;
	}
	return label_def;
}
BOOL geni_vm_compile_is_label(GENI_COMPILER_CONTEXT* context,char** codebufferptr){
	BOOL label_ref = FALSE;
	//If it looks like a label, it return TRUE (if the label is not declared, it will add a reference to be resolved later).
	char* code = *codebufferptr;
	while(*code){
		if((*code>='A' && *code<='Z')
			|| (*code>='a' && *code<='z')
			|| (*code=='_')
			|| (code>*codebufferptr && (*code>='0' && *code<='9'))){
			code++;
		}
		else{
			break;	//this is not a label declaration
		}
	}
	if(label_ref = (code > *codebufferptr && *code!=':')){
		GENI_COMPILER_NAMESPACE* name;
		DWORD label_len;
		label_len = (DWORD)code - (DWORD)*codebufferptr;
		if(geni_vm_compile_is_keyword(context,*codebufferptr,label_len)){
			label_ref = FALSE;
		}
		else{
			name=geni_vm_compile_get_name(context,*codebufferptr, label_len);
			if(!name){
				name = geni_vm_compile_new_name(context,*codebufferptr, label_len);
			}
			geni_vm_compile_newname_ref(context, name);
			*codebufferptr = code;
		}
	}
	return label_ref;
}

BOOL geni_vm_compile_resolve_labels(GENI_COMPILER_CONTEXT* context ){
	BOOL ok = TRUE;
	DWORD i, r;
	GENI_COMPILER_NAMESPACE* name;
	GENI_COMPILER_NAMESPACE_REF* ref;
	DWORD* reference;
	//Try to resolve labels, and throw list of unresolved labels if any.
	for(i=0;i<context->name_count;i++){
		name = &context->names[i];
		for(r=0;r<name->ref_count;r++){
			ref = &name->references[r];
			if(name->segment==SEG_UNKNOW){
				printf("unresolved label `%s` at line %d!\n",name->name, ref->source_line);
				ok=FALSE;
			}
			else{
				if(ref->segment==SEG_DATA){
					reference = (DWORD*)( (DWORD)context->memory_segment + (DWORD)ref->address );
				}
				else{//code segement
					reference = (DWORD*)( (DWORD)context->code_segment + (DWORD)ref->address );
				}
				*reference = name->address;	//patch reference ( must be -1 before to be patched ! )
				//TODO: FIXME: ATTENTION, une référence à une zone mémoire ne sera pas patchée magiquement !
				// c-à-d que: " push my_var, 0 " mettera sur la pile l'addresse virtuelle de la variable et non son contenu,
				// alors que: " push m[my_var],0" mettra sur la pile le contenu de la variable.
			}
		}
		if(r==0){
			printf("unreferenced label `%s` at line %d!\n",name->name, name->line_defined);
		}
		if(context->post_build && context->post_build==name){
			context->post_build = (GENI_COMPILER_NAMESPACE*)name->address;
		}
		safe_free(name->references);
		safe_free(name->name);
	}
	safe_free(context->names);
	return ok;
}

//VOID because it cannot have a syntax error :)
DWORD geni_vm_compile_skip_whitespaces(GENI_COMPILER_CONTEXT* context, char** codestr){
	char* code = *codestr;
	DWORD eaten = (DWORD)*codestr;
	BOOL b_continue = TRUE;
	BOOL b_comment = FALSE;
	while(b_continue && *code){
		switch(*code){
			case ' ': case '\t': case '\r':
				code++;
				break;
			case '\n':
				if(b_comment){
					b_comment = FALSE;
				}
				context->current_line++;
				code++;
				break;
			case ';': case '#':
				b_comment = TRUE;
				code++;
				break;
			default:
				if(!b_comment){
					b_continue = FALSE;
				}
				else{
					code++;
				}
				break;
		}
	}
	*codestr = code;
	return ((DWORD)*codestr) - eaten;
}
BOOL geni_vm_compile_number(GENI_COMPILER_CONTEXT* context,char** codebufferptr,DWORD* number){
	BOOL stop = FALSE;
	BOOL ok = TRUE;
	BOOL isnumber=FALSE;
	BOOL ishexa=FALSE;
	DWORD value=0;
	geni_vm_compile_skip_whitespaces(context,codebufferptr);
	if(geni_vm_compile_is_label(context,codebufferptr)){
		stop = TRUE;
		isnumber = TRUE;
		value = -1;		//All reference will be resolved at the end of the compilation.
	}
	while(**codebufferptr && !stop){
		switch(**codebufferptr){
			case 'x': case 'X':			//entring in an HEXA number notation
				ishexa=TRUE;
				isnumber=TRUE;	//may be patched to introduce labels (or alias/macros/variables)
				if(value != 0){	//bad format number !
					stop = TRUE;
					ok = FALSE;
				}
				break;
			case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9': 
				isnumber=TRUE;				
				value *= ishexa ? 16 : 10;
				value += (**codebufferptr) - '0';
				break;
			case 'A': case 'B': case 'C': case 'D': case 'E': case 'F': 
			case 'a': case 'b': case 'c': case 'd': case 'e': case 'f': 
				if(!ishexa){
					stop = TRUE;
					ok = FALSE;
				}
				else{
					value *= 16;
					value += ( ((**codebufferptr) | ' ') - 'a' ) + 10;
				}
				break;
			case '_':
				if(!isnumber){
					stop = TRUE;
					break;
				}
				break;
			default:
				stop = TRUE;
				break;
		}
		if(!stop){
			(*codebufferptr)++;
		}
	}
	*number = value;
	return ok && isnumber;
}

BOOL geni_vm_compile_operand(GENI_COMPILER_CONTEXT* context, GENI_OPERAND* operand, char** instruction){
	char* ptr = *instruction;
	BOOL stop = FALSE, ok = TRUE;
	BOOL ishexa = FALSE;
	BOOL isnumber = FALSE;
	BOOL isregister = FALSE;
	BOOL isregisterptr = FALSE;
	BOOL ispointer = FALSE;
	BOOL ishost = FALSE;
	BOOL ishostptr = FALSE;
	BOOL isvm = FALSE;
	BOOL isvmptr = FALSE;
	BOOL canbelabel = TRUE;
	DWORD value = 0;
	memset(operand, 0, sizeof( GENI_OPERAND ) );
	/*
		Voici les notations que l'on devra pouvoir parser :
		r0
		r1234
		r0x1234
		rx123
		[r0]		: the value pointed in VM memory at the offset of the value which is in r0
		m[r0]		: the value pointed in VM memory at the offset of the value which is in r0
		h[r0]		: the value pointed in HOST memory at the offset of the value which is r0
		h[m0]		: the value pointed in HOST memory at the offset of the value which is in m0
		h[0x7000]	: the value pointed in HOST memory at the offset 0x7000
		h0x7000		: the value pointed in HOST memory at the offset 0x7000
		0123
		0x123AF
				
	*/
	while(*ptr && !stop){
		if(canbelabel && geni_vm_compile_is_label(context,&ptr)){
			//assert *instruction is still valid
			isnumber=TRUE;
			value=-1;
			if(ispointer && !ishost){
				isvm = TRUE;
			}
			//Be sure that context can know the real position in code_segment for the value to replace (leftop or rightop value !)
		}
		switch( *ptr ){
			case '[':
				ispointer = TRUE;
				if(ishost){
					ishost = FALSE;
					ishostptr = TRUE;
				}
				else if(isvm){
					isvm = FALSE;
					isvmptr = TRUE;
				}
				else if(isregister){
					isregister = FALSE;
					isregisterptr = TRUE;
				}

				break;
			case '@':
				//just here to accept: 
				//	m_or_h@a_label_or_value
				//	[m_or_h@a_label_or_value]
				//	h[m_or_h@a_label_or_value]
				if(!ishost && !isvm && !isregister){
					isvm = TRUE;
				}
				break;
			case 'h': case 'H':
				if(ishost || isvm || isregister){ // do not accept multiple "h" prefix
					ok = FALSE;
					stop = TRUE;
				}
				else{
					ishost = TRUE;
				}
				break;
			case 'm': case 'M':
				if(isvm || ishost || isregister){ //do not accept multiple "m" prefix
					ok = FALSE;
					stop = TRUE;
				}
				else{
					isvm = TRUE;
				}
				break;
			case ']':
				if(!ispointer){
					ok = FALSE;
				}
				stop = TRUE;
				break;
			case 'r': case 'R':
				if(isvm || ishost || isregister){
					ok = FALSE;
					stop = TRUE;
				}
				else{
					isregister = TRUE;
				}
				break;
			case ' ': case '\t':		//Skip spaces
				break;
			case 'x': case 'X':			//entring in an HEXA number notation
				ishexa=1;
				isnumber=1;	//may be patched to introduce labels (or alias/macros/variables)
				if(value != 0){	//bad format number !
					stop = TRUE;
					ok = FALSE;
				}
				break;
			case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9': 
				isnumber = TRUE;
				canbelabel = FALSE;
				value *= ishexa ? 16 : 10;
				value += (*ptr) - '0';
				break;
			case 'A': case 'B': case 'C': case 'D': case 'E': case 'F': 
			case 'a': case 'b': case 'c': case 'd': case 'e': case 'f': 
				if(!ishexa){
					stop = TRUE;
					ok = FALSE;
				}
				else{
					value *= 16;
					value += ( ((*ptr) | ' ') - 'a' ) + 10;
				}
				break;
			case '>':
				isnumber=TRUE;
				value = context->instruction_count +1;	//Hack for writting Jmps next
			case '_':
				if(!isnumber){
					stop = TRUE;
					ptr --;
				}
				break;
			default:
				//must stop 
				if(ispointer){
					ok = FALSE;
				}
				if(isregister && !isnumber){
					ok = FALSE;
				}
				stop = TRUE;
				ptr --;
				break;
		}
		ptr++;		
	}
	*instruction = ptr;
	if(ok){
		operand->size = 4;			//default is 32bits
		operand->type = isregister ? OK_REGISTER :
										ishost ? OK_HOST_MEMORY :
										isvm ? OK_VM_MEMORY : OK_CONSTANT;
		operand->mode = ishostptr ? OM_HOST_POINTER : ( isvmptr ? OM_VM_POINTER : (isregisterptr ? OM_REG_POINTER : OM_NORMAL ) );
		operand->dword = value;
	}
	return ok;
}

BOOL geni_vm_compile_opcode(GENI_COMPILER_CONTEXT* context, GENI_OPCODE* opcode, char** codestr){
	long len;
	BYTE i;
	BOOL ret;
	char* mnemo;
	char *instruction;
	instruction = *codestr;

	geni_vm_compile_skip_whitespaces(context, &instruction);
	if(*instruction=='.'){
		*codestr = instruction;
		return FALSE;
	}
	if(geni_vm_compile_have_label_declaration(context,&instruction)){
		//do nothing special here let's consume
		geni_vm_compile_skip_whitespaces(context, &instruction);
		if(!*instruction){
			//TODO: this does not allow to add a pointer to void code at the end of code segement. 
			//	(no problem at this time because we are not able to read/write the code segement from code)
			geni_vm_compile_error(context, &instruction,"opcode expected !");
			*codestr = instruction;
			return FALSE;
		}
	}
	//Try to compile the given instruction into
	//first, look for a know instruction
	for(i=0;i<MAX_OPCODES;i++){
		mnemo = context->vm->opcodeset[i].mnemonic;
		len = strlen(mnemo);
		if(_strnicmp(instruction, mnemo, len)==0 
			&&(instruction[len]==' '
			||instruction[len]=='\t')){
			opcode->instruction = i;
			instruction += len;
			//~ printf("** found %i th '%s' mnemonic of len %d\n", i, mnemo, len);
			geni_vm_compile_skip_whitespaces(context, &instruction);	//skip spaces
			context->is_operand_left = TRUE;
			ret = geni_vm_compile_operand( context, &opcode->leftop, &instruction );
			//~ printf("** parsed leftop, now instr=`%s`\n", instruction);
			if(!ret){ 
				*codestr = instruction;
				//~ printf("** lefttop not compiled ! `%s`\n", instruction);
				return FALSE;
			}
			geni_vm_compile_skip_whitespaces(context, &instruction);	//skip spaces
			if(*instruction != ','){
				geni_vm_compile_error(context, &instruction,"Missing character: ',' !");
				*codestr = instruction;
				return FALSE;
			}
			instruction++;	//skip ','
			geni_vm_compile_skip_whitespaces(context, &instruction);	//skip spaces
			context->is_operand_left = FALSE;
			ret = geni_vm_compile_operand( context, &opcode->rightop, &instruction );
			//~ printf("** parsed rightop, now instr=`%s`\n", instruction);
			*codestr = instruction;
			if(!ret){
				//~ printf("** rightop not compiled ! `%s`\n", instruction);
				return FALSE;
			}
			return TRUE;
		}
	}
	geni_vm_compile_error(context, &instruction,"** mnemonic not found !\n");
	*codestr = instruction;
	return FALSE;
}

DWORD geni_vm_compile_code(GENI_COMPILER_CONTEXT* context, char** codebufferptr){
	GENI_OPCODE* opcodes;
	BOOL r = TRUE;
	DWORD i = 0;
	char* codebuffer = *codebufferptr;
	opcodes = context->code_segment;
	if(!opcodes){		
		opcodes = safe_alloc( (context->code_size = CODE_CHUNK_SIZE) * sizeof(GENI_OPCODE) );
	}
	while( *codebuffer && ( r = geni_vm_compile_opcode( context, &opcodes[i], &codebuffer) ) ){
		i++;
		context->instruction_count++;
		if(i == context->code_size){
			//adjust allocated memory of compiled code
			if(!safe_realloc( &opcodes, (context->code_size += CODE_CHUNK_SIZE) * sizeof(GENI_OPCODE) )){
				printf("Not enought memory (not optimized using temp file !)\n");
				r = -1;
				goto compile_safe_exit;
			}
		}
		geni_vm_compile_skip_whitespaces(context, &codebuffer);	//It was only a reader until the end of the line before refactoring
		//~ printf("** compiled %d instructions, `%s`\n", i, codebuffer);
		//codebuffer++;	//then strip end of line too
	}
compile_safe_exit:
	context->code_segment = opcodes;
	*codebufferptr = codebuffer;
	return r == -1 ? -1 : i;
}


void geni_vm_compile_assert_memory_space(GENI_COMPILER_CONTEXT* context){
	if(context->memory_size < context->memory_current_offset + DEFAULT_MEMORY_SIZE){
		//reallocate the memory block with a new chunk
		context->memory_size = context->memory_current_offset + DEFAULT_MEMORY_SIZE;
		safe_realloc( &context->memory_segment, context->memory_size);
	}
}

BOOL geni_vm_compile_data(GENI_COMPILER_CONTEXT* context, char** codebufferptr){
	//Can parse :
	//	string 'blablablablabl sdf qsdfg \\ and \' and so \\\'sdfg sdfg '
	//			-> do no have any other escapement characters (just use the new line chars into quotes :)
	//	[db|dw|dd|byte|word|dword] number_list
	//	number_list: number ([, /] number)*
	//	dd a_code_label
	//
	// Then introduce labels:
	//		BOOL geni_vm_compile_have_label_declaration( context, codebufferptr )	//If yes, then it will be automatically added into the names and the buffer will be consummed.
	//		BOOL geni_vm_compile_is_label( context, codebufferptr, &label_address)	//If it looks like a label, it return TRUE (if the label is not declared, it will add a reference to be resolved later).
	//		BOOL geni_vm_compile_resolve_labels( context )	//Try to resolve labels, and throw list of unresolved labels if any.
	//	-> How to handle not yet declared labels ?
	//		a) add a "to_declare_names" which can handled a list of memory/code references
	BOOL ok = TRUE;
	BOOL stop = FALSE;
	DWORD spaces;
	while(**codebufferptr && !stop){
		if(geni_vm_compile_have_label_declaration(context,codebufferptr)){
			geni_vm_compile_skip_whitespaces(context, codebufferptr);
			if(!**codebufferptr){
				stop = TRUE;
				break;
			}
		}
		if(_strnicmp(*codebufferptr,"dword", 5)==0){
			DWORD dd_value;
			*codebufferptr += 5;
			while( geni_vm_compile_number(context,codebufferptr,&dd_value) ){
				DWORD* next_dd_value;
				geni_vm_compile_assert_memory_space(context);
				next_dd_value = (DWORD*) (&context->memory_segment[context->memory_current_offset]);
				*next_dd_value = dd_value;
				context->memory_current_offset+=4;
				spaces = geni_vm_compile_skip_whitespaces(context, codebufferptr);
				if((**codebufferptr==',') || (**codebufferptr=='|') || (**codebufferptr=='/')){
					(*codebufferptr)++;	//continue to eat list of number as dword
				}
				else if(!spaces){
					//stop = TRUE;
					break;
				}
			}
		}
		else if(_strnicmp(*codebufferptr,"word", 4)==0){
			DWORD dw_value;
			*codebufferptr += 4;
			while( geni_vm_compile_number(context,codebufferptr,&dw_value) ){
				WORD* next_dwvalue;
				geni_vm_compile_assert_memory_space(context);
				next_dwvalue = (WORD*) (&context->memory_segment[context->memory_current_offset]);
				*next_dwvalue = (WORD)(dw_value & 0xFFFF);
				context->memory_current_offset+=2;
				spaces = geni_vm_compile_skip_whitespaces(context, codebufferptr);
				if((**codebufferptr==',') || (**codebufferptr=='|') || (**codebufferptr=='/')){
					(*codebufferptr)++;	//continue to eat list of number as dword
				}
				else if(!spaces){
					//stop = TRUE;
					break;
				}
			}
		}
		else if(_strnicmp(*codebufferptr,"byte", 4)==0){
			DWORD db_value;
			*codebufferptr += 4;
			while( geni_vm_compile_number(context,codebufferptr,&db_value) ){
				BYTE* next_dbvalue;
				geni_vm_compile_assert_memory_space(context);
				next_dbvalue = (BYTE*) (&context->memory_segment[context->memory_current_offset]);
				*next_dbvalue = (BYTE)(db_value & 0xFF);
				context->memory_current_offset++;
				spaces = geni_vm_compile_skip_whitespaces(context, codebufferptr);
				if((**codebufferptr==',') || (**codebufferptr=='|') || (**codebufferptr=='/')){
					(*codebufferptr)++;	//continue to eat list of number as dword
				}
				else if(!spaces){
					//stop = TRUE;
					break;
				}
			}
		}
		else if(_strnicmp(*codebufferptr,"string", 6)==0){
			char quote;
			char* string;
			char* mem_str;
			DWORD stringlength;
			*codebufferptr += 6;
			geni_vm_compile_skip_whitespaces(context, codebufferptr);			
			quote = **codebufferptr;
			switch(quote){
				case '(':
					quote = ')';
					break;
				case '[':
					quote = ']';
					break;
				case '{':
					quote = '}';
					break;
				case '<':
					quote = '>';
					break;
				case '"': case '\'': case '/': case '|': 
					break;
				default:
					//PARSE ERROR
					geni_vm_compile_error(context, codebufferptr, "bad quote string !");
					stop = TRUE;
					ok = FALSE;
					break;
			}
			if(stop) break;
			(*codebufferptr)++;
			string = *codebufferptr;
			while(*string && *string!=quote){
				if(*string=='\\' && *(string+1)){
					//TODO: THERE is a bug on ignoring this \ char !
					// 1) It is counted in strlen 
					// 2) It is copyed in vmmemory state
					string++;	//let's eat all meta chars, so \ itself
				}
				string++;
			}
			stringlength = (string - *codebufferptr +1);	//add a NULL terminal char.
			mem_str = (char*) context->memory_current_offset;
			context->memory_current_offset += stringlength;
			geni_vm_compile_assert_memory_space(context);
			mem_str = (char*) ((DWORD)context->memory_segment + (DWORD)mem_str);
			memcpy(mem_str, *codebufferptr, stringlength);
			mem_str += stringlength -1;
			*mem_str=0;	//NULL terminate char
			(*codebufferptr) += stringlength + 1;
			spaces = geni_vm_compile_skip_whitespaces(context, codebufferptr);
			/* : TODO, allow multiples strings
			if((**codebufferptr==',') || (**codebufferptr=='|') || (**codebufferptr=='/')){
				(*codebufferptr)++;	//continue to eat list of number as dword
			}
			else if(!spaces){
				//stop = TRUE;
				break;
			}*/
		}
		else{
			stop = TRUE;
		}
	}
	if(context->memory_max_offset < context->memory_current_offset){
		context->memory_max_offset = context->memory_current_offset;
	}
	return ok;
}

BOOL geni_vm_compile(GENI_COMPILER_CONTEXT* context, char** codebufferptr){
	BOOL ret=TRUE;
	geni_vm_compile_skip_whitespaces(context, codebufferptr);
	while(ret && **codebufferptr){
		switch(**codebufferptr){
			case '.':
				//Must be followed by [code|data|memsize|org]
				//allow to switch current segment !
				if(_strnicmp(*codebufferptr,".code", 5)==0){
					*codebufferptr += 5;
					context->current_segment = SEG_CODE;
				}
				else if(_strnicmp(*codebufferptr,".data", 5)==0){
					*codebufferptr += 5;
					context->current_segment = SEG_DATA;
				}
				//TODO/TOTEST: Implement the org to allow multiple sub segment in memory
				else if(_strnicmp(*codebufferptr,".org", 4)==0){
					DWORD newmemory_offset;
					*codebufferptr += 4;
					if(geni_vm_compile_number(context,codebufferptr,&newmemory_offset)){
						context->memory_current_offset = newmemory_offset;
						//FIXME: may assert memory-size now ?
					}
					else{
						geni_vm_compile_error(context, codebufferptr, "Number expected !");
						ret = FALSE;
					}
				}
				else if(_strnicmp(*codebufferptr,".memsize", 8)==0){
					DWORD newmemory_size;
					*codebufferptr += 8;
					if(geni_vm_compile_number(context,codebufferptr,&newmemory_size)){
						//TODO: assert newmemory_size is enought to handled stored size !
						context->memory_size = newmemory_size;
					}
					else{
						ret = FALSE;
					}
				}
				else if(_strnicmp(*codebufferptr,".postbuild", 10)==0){
					DWORD pblen;
					char* lbl;
					*codebufferptr += 10;
					geni_vm_compile_skip_whitespaces(context, codebufferptr);
					lbl = *codebufferptr;
					for(pblen = 0; 
						*lbl && *lbl!=' ' && *lbl!=';' && *lbl!='\r' && *lbl!='\n';
						pblen++) 
						lbl++;
					context->post_build = geni_vm_compile_get_name(context,*codebufferptr, pblen);
					if(!context->post_build){
						geni_vm_compile_error(context, codebufferptr, "namespace expected !");
						ret = FALSE;
					}
					else{
						*codebufferptr += pblen;
					}
				}
				else{
					geni_vm_compile_error(context, codebufferptr, "Unknow pragma !");
					ret = FALSE;
				}
				break;
			default:
				switch(context->current_segment){
					case SEG_CODE:
					{
						DWORD instruction_count = geni_vm_compile_code(context, codebufferptr);
						ret = (instruction_count!=-1);
						//context->instruction_count += instruction_count; //Est fait en direct lors de l'ajout d'une instruction
					}
					break;
					case SEG_DATA:
						ret = geni_vm_compile_data(context, codebufferptr);
						break;
					default:
						//error !
						geni_vm_compile_error(context, codebufferptr, "Internal error: unknow segment!");
						ret = FALSE;
						break;
				}
				break;
		}
		if(ret){
			geni_vm_compile_skip_whitespaces(context, codebufferptr);
		}
	}
	if(ret){
		ret = geni_vm_compile_resolve_labels(context);
		if(ret && context->post_build){	/*ARF a postbuild could not start at offset IP=0 !*/
			//execute the given proc before to store it into a file.
			//requiere to duplicate the VM etc...
			GENI_VM* vm;
			if(vm = geni_vm_init()){
				geni_vm_load_code(vm, context->code_segment, context->code_size * sizeof(GENI_OPCODE));
				//prepare memory
				vm->memory.size = context->memory_size;
				safe_realloc(&vm->memory.ptr, vm->memory.size);
				memcpy(vm->memory.ptr, context->memory_segment, context->memory_size);
				vm->code.ip = (DWORD)context->post_build;
				geni_vm_run(vm);
				//got the new memory state.
				memcpy(context->memory_segment, vm->memory.ptr, context->memory_size);
				geni_vm_delete(vm);
			}
		}
	}
	return ret;
}
#endif
#ifdef HAS_CORE

void geni_vm_display_help(){
	printf("\n"
"vm -h\n"
"  this help message\n"
"vm -c outputfilename[.gvm] sourcefile.gvs\n"
"  compile\n"
"vm binary.gvm\n"
"  execute binary\n"
"vm -sw watermarkfile.txt binary.gvm\n"
"	set a watermark file in the vm binary file\n"
"vm -gw watermarkfile.txt binary.gvm\n"
"	get a watermark file in the vm binary file\n"
"vm -r source.gvs\n"
"  compile and run source (but do not produce an output binary file)\n"
"vm -d binary.gvm\n"
"  run code and dump vm\n"
"vm -dc binary.gvm\n"
"  dump code and initial memory state but do not run code\n"
"vm -v ...\n"
"  actiave verbose mode\n"
"vm -D binary.gvm\n"
"  Interactive DEBUGGER\n"
"  gvm>h\n"
"    q        quit\n"
"    n        next  : show new 'instruction' (and final value of operands + ip + sp + flags)\n"
"    d        dump all ( flags, register, memory, stack, but code)\n"
"    dvm      dump all including full loaded code\n"
"    dr        dump registers\n"
"    dm      dump memory\n"
"    dh  [x]    dump host memory from x\n"
"    dc [x[..y]]  dump code from X [ to Y ]\n"
"    ds        dump stack\n"
"    Setters/Getters( whitout the last arg)\n"
"    ip [newip]  set ip to newip and display next matching instruction\n"
"    sp [newsp]  set sp to newsp value\n"
"    rxxx  yyy  set register xxx to the value yyy\n"
"    mxxx [byte/word/dword*] yyy  set vm-memory xxx to the value yyy\n"
"    hxxx [byte/word/dword*] yyy  set \n"
"    do INSTRUCTION\n"
"            compile and execute INSTRUCTION whitout appendind it to the code-chunk.\n"
"    c [address] INSTRUCTION\n"
"            compile INSTRUCTION at [address], this will replace previous instruction\n"
"    !!! Becare, THOSE will NOT re-arrange code pointers for jmp / call instructions !::\n"
"    ic [address] INSTRUCTION\n"
"            insert compiled INSTRUCTION at [address] by increasing code and moving next instructions\n"
"    rc  [address] [count=1]\n"
"            remove instruction at [address] or ip if nothing specified.\n"
"    h        display this help screen\n"
"\n");
}



BOOL geni_vm_is_binary(char* filename){
	FILE* h = NULL;
	DWORD marka = 0, markb = 0;
	fopen_s(&h,filename, "rb");
	if(!h){
		return -1;
	}
	fread(&marka, sizeof(DWORD), 1, h);
	fread(&markb, sizeof(DWORD), 1, h);
	fclose(h);
	return marka == 'INEG' && ((markb & 0xFFFF)=='MV');
}

BOOL save_file(char* outputfile, DWORD filesize, void* filebuffer){
	FILE* ho;
	DWORD chunks;
	fopen_s(&ho, outputfile,"wb+");
	if(!ho){
		return FALSE;
	}
	chunks = fwrite(filebuffer, filesize, 1, ho);
	fclose(ho);
	return chunks == 1;
}
BOOL save_gvm_file(GENI_VM* vm, char* binaryfile){
	FILE* ho;
	DWORD instruction_count = 0;
	instruction_count = vm->code.size;
	if(verbose) printf("Openning output file `%s`..", binaryfile);
	fopen_s(&ho, binaryfile,"wb+");
	if(ho){
		DWORD marka = 'INEG';
		DWORD markb = (GENI_VM_VERSION<<16)+'MV';
		size_t bytes;
		if(verbose){
			printf("ok\n");
			printf("Output file `%s` (%08x)\n", binaryfile, ho);
			printf("Header is (%08x)\n", instruction_count);
		}
		if(verbose) printf("Writting header..");				
		bytes = fwrite(&marka, 1, sizeof( DWORD ), ho);
		bytes = fwrite(&markb, 1, sizeof( DWORD ), ho);
		bytes = fwrite(&vm->memory.size, 1, sizeof( DWORD ), ho);
		bytes = fwrite(&vm->memory.storedbytes, 1, sizeof( DWORD ), ho);
		bytes = fwrite(&instruction_count, 1, sizeof( DWORD ), ho);
		if(verbose) printf("ok\n");
		if(verbose) printf("Writing memory state..");
		bytes = fwrite(vm->memory.ptr, vm->memory.storedbytes, 1, ho);
		if(verbose) printf("ok\n");
		if(verbose) printf("Instructions are %d bytes length\n", instruction_count * sizeof(GENI_OPCODE) );
		if(verbose) printf("Writting code..");
		bytes = fwrite(vm->code.opcodes, vm->code.size,sizeof(GENI_OPCODE), ho);
		if(verbose) printf("ok\n");
		fclose(ho);
	}
	else{
		if(verbose) printf("not ok\n");
		printf("Unable to write into the output file `%s` !\n", binaryfile);
		return FALSE;
	}
	return TRUE;
}

BYTE* geni_vm_watermark_next(GENI_VM_WATERMARK_ITERATOR* iterator){
	BYTE* byte;
	GENI_OPCODE* opcode;
	opcode = &iterator->opcodes[iterator->current_opcode];
	switch(iterator->byte_idx){
		case 0: case 1: case 2: case 3:
			byte = (BYTE*)&opcode->size;
			break;
		case 4: case 5: case 6: case 7:
			byte = (BYTE*)&opcode->leftop.size;
			break;
		case 8: case 9: case 10: case 11:
			byte = (BYTE*)&opcode->rightop.size;
			break;
	}
	byte += (iterator->byte_idx & 0x3);
	if(++iterator->byte_idx >= 3*4){
		iterator->byte_idx = 0;
		iterator->current_opcode++;
	}
	return byte;
}

BOOL geni_vm_set_watermark(GENI_VM* vm, char* watermarkfile, char* binaryfile){
	//vm must be already loaded.
	DWORD filesize=0, wmfilesize;
	byte* filebuffer=NULL;
	DWORD i;
	BYTE byte;
	BYTE* wmb=NULL;
	GENI_VM_WATERMARK_ITERATOR wm_iterator;
	if(!load_file(watermarkfile,&filesize,(void**)&filebuffer)){
		printf("Could not read watermark file %s !\n", watermarkfile);
		return FALSE;
	}
	//ASSERT FILE SIZE ( codesize * (4 * 3 available bytes) ) >= filesize+4
	//initialize watermark iterator
	wm_iterator.byte_idx =0;
	wm_iterator.current_opcode=0;
	wm_iterator.opcodes = vm->code.opcodes;

	wmfilesize = ~filesize;
	wmfilesize = xor(wmfilesize,0xA073B);
	wmb = geni_vm_watermark_next(&wm_iterator);
	*wmb = wmfilesize & 0xFF;
	wmb = geni_vm_watermark_next(&wm_iterator);
	*wmb = (wmfilesize >> 8) & 0xFF;
	wmb = geni_vm_watermark_next(&wm_iterator);
	*wmb = (wmfilesize >> 16) & 0xFF;
	wmb = geni_vm_watermark_next(&wm_iterator);
	*wmb = (wmfilesize >> 24) & 0xFF;
	//set watermark bytes for filesize
	for(i=0;i<filesize;i++){
		wmb = geni_vm_watermark_next(&wm_iterator);
		byte = filebuffer[i];
		//we could obfuscate byte here.
		byte = byte + ((wm_iterator.current_opcode + wm_iterator.byte_idx) & 0xFF);
		*wmb = byte;
	}
	//ofbuscate to the end of the last opcode !
	srand( ~ GetTickCount() );
	while(wm_iterator.current_opcode<=vm->code.size){
		wmb = geni_vm_watermark_next(&wm_iterator);
		byte = ( wm_iterator.byte_idx + rand() ) & 0xFF;
		*wmb = byte;
	}
	return save_gvm_file(vm, binaryfile);
}

BOOL geni_vm_get_watermark(GENI_VM* vm, char* watermarkfile){
	DWORD filesize=0, i;
	BYTE* filebuffer=NULL;
	BYTE* wmb;
	BYTE byte;
	BOOL ok = TRUE;
	GENI_VM_WATERMARK_ITERATOR wm_iterator;

	wm_iterator.byte_idx = 0;
	wm_iterator.current_opcode = 0;
	wm_iterator.opcodes = vm->code.opcodes;
	
	wmb = geni_vm_watermark_next(&wm_iterator);
	filesize = *wmb;
	wmb = geni_vm_watermark_next(&wm_iterator);
	filesize += (*wmb) << 8;
	wmb = geni_vm_watermark_next(&wm_iterator);
	filesize += (*wmb) << 16;
	wmb = geni_vm_watermark_next(&wm_iterator);
	filesize += (*wmb) << 24;//filesize = (filesize<<8) + *wmb;
	//Desobfuscate file size.
	filesize = xor(filesize,0xA073B);
	filesize = ~filesize;
	if(filesize){
		//now we could allocate a filebuffer
		filebuffer = (BYTE*)safe_alloc(filesize);
		for(i=0;i<filesize;i++){
			wmb = geni_vm_watermark_next(&wm_iterator);
			byte = *wmb;
			//unobfuscate byte
			byte = byte - ((wm_iterator.current_opcode + wm_iterator.byte_idx) & 0xFF);

			filebuffer[i] = byte;
		}

		if(!filebuffer){
			printf("Could not allocate %d byte of memory !\n", filesize);
			return FALSE;
		}

		if(save_file(watermarkfile,filesize,(void*)filebuffer)){
			printf("Found watermark of %d byte(s)!\n",filesize);
		}
		else{
			printf("Could not write watermark file %s!\n",watermarkfile);
			ok = FALSE;
		}
	}
	else{
		printf("Could not found any watermark there !\n");
		return FALSE;
	}
	return ok;
}

int main(int argc, char** argv){
	int return_code = 0;
	char* outputfile = NULL;
	char* sourcefile = NULL;
	char* binaryfile = NULL;
	BOOL compile = FALSE;
	BOOL run = TRUE;
	BOOL dump = FALSE;
	BOOL debug = FALSE;
	char* watermarkfile = NULL;
	BOOL getwatermark=FALSE;
	BOOL setwatermark=FALSE;
	GENI_VM* vm = NULL;
	int i;
#ifdef CAN_COMPILE
	GENI_COMPILER_CONTEXT context;
	context.code_segment = NULL;
	context.code_size=0;
	context.instruction_count = 0;
	context.current_line = 1;
	context.memory_segment = NULL;
	context.memory_size = 0;
	context.memory_current_offset = 0;
	context.memory_max_offset = 0;
	context.name_count = 0;
	context.names = NULL;
	context.current_segment = SEG_CODE;
	context.is_operand_left = FALSE;
	context.post_build = NULL;
	context.vm = NULL;
#endif

	for(i=1;i<argc;i++){
		if(strcmp(argv[i],"h")==0 
			|| strcmp(argv[i],"-h")==0
			|| strcmp(argv[i],"-?")==0
			|| strcmp(argv[i],"-help")==0){
			geni_vm_display_help();
			return_code = 0;
			goto safe_exit;
		}
		else if(strcmp(argv[i],"-c")==0){
			//it must be followed by the outputfile name
			i++;
			outputfile = argv[i];
			compile = TRUE;
			run = FALSE;
		}
		else if(strcmp(argv[i],"-sw")==0){
			//it must be followed by the outputfile name
			if(getwatermark){
				printf("Could not use -gw and -sw at the same command !\n");
				return_code=9;
				goto safe_exit;
			}
			i++;
			watermarkfile = argv[i];
			setwatermark = TRUE;
		}
		else if(strcmp(argv[i],"-gw")==0){
			//it must be followed by the outputfile name
			if(setwatermark){
				printf("Could not use -gw and -sw at the same command !\n");
				return_code=9;
				goto safe_exit;
			}
			i++;
			watermarkfile = argv[i];
			getwatermark = TRUE;
		}
		else if(strcmp(argv[i],"-r")==0){
			//it must be followed by the outputfile name
			i++;
			compile = TRUE;
			run = TRUE;
		}
		else if(strcmp(argv[i],"-v")==0){
			verbose = TRUE;
		}
		else if(strcmp(argv[i],"-d")==0){
			dump = TRUE;
		}
		else if(strcmp(argv[i],"-dc")==0){
			dump = TRUE;
			run = FALSE;
		}
		else if(strcmp(argv[i],"-D")==0){
			debug = TRUE;
			printf("NOT IMPLEMENTED YET !\n");
			return_code = -1;
			goto safe_exit;
		}
		else{
			BOOL filetype;
			filetype = geni_vm_is_binary(argv[i]);
			if(filetype==-1){
				printf("file %s does not exists !\n", argv[i]);
				return_code = 12;
				goto safe_exit;
			}
			if(filetype){
				binaryfile = argv[i];
			}
			else{
				sourcefile = argv[i];
			}
		}
	}

	if(!sourcefile && !binaryfile){
		printf("no inputfile ? (see usage: -h)\n");
		return_code = 1;
		goto safe_exit;
	}

	//initialize VM (requiered for load_file)
	if(verbose) printf("Initializing VM..");
	vm = geni_vm_init();
	if(!vm){
		if(verbose) printf("not ok\n");
		return_code = 4;
		goto safe_exit;
	}

	if(verbose) printf("ok\n");
	if(debug){
		vm->flags.debug = TRUE;
		if(verbose) printf("Debug activated\n");
	}

	if(compile){
#ifdef CAN_COMPILE
		BOOL ok = TRUE;
		char* code = NULL;
		DWORD filesize = 0, instruction_count = 0;
		void* filebuffer = NULL;

		if(!load_file(sourcefile,&filesize, &filebuffer)){
			printf("Could not read inputfile !");
			//could not read file
			return_code = 3;
			goto safe_exit;
		}

		code = (char*)filebuffer;
		context.vm = vm;
		//assert it is a source code (not a binary file)
		//~ ...
		if(verbose) printf("Compiling input..");
		ok = geni_vm_compile(&context, (char**) &code /*MUST BE NULL TERMINATED !*/);
		if(!ok){
			return_code = 7;
			goto safe_exit;
		}
		instruction_count = context.instruction_count;
		//requiere to free context allocated pointers
		if(instruction_count <= 0){
			if(verbose) printf("not ok\n");
			//~ code_buffer --;
			printf("Syntax error near `%s`\n", code);
			return_code = 13;
			goto safe_compile_exit;
		}
		if(verbose) printf("ok\n");
		//compile code source
		/*
		//TODO: check if it could be re-activated...
		if(!outputfile){
			printf("no outputfile ? (see usage: -h)\n");
			return_code = 2;
			goto safe_exit;
		}*/

		//Loading Code and memory into the VM.
		if(verbose) printf("Loading compiled memory..");
		vm->memory.storedbytes = context.memory_max_offset;
		vm->memory.size = context.memory_size;
		vm->memory.ptr = safe_alloc( vm->memory.size );
		memcpy(vm->memory.ptr, context.memory_segment, vm->memory.storedbytes);
		if(verbose) printf("ok\n");

		if(verbose) printf("Loading compiled code..");
		geni_vm_load_code( vm, context.code_segment, sizeof(GENI_OPCODE) * instruction_count );
		if(verbose) printf("ok\n");
		
		if(outputfile){//write to outputfile
			save_gvm_file(vm, outputfile);
		}
safe_compile_exit:
		safe_free( context.code_segment );
		safe_free( context.memory_segment );
#else
		printf("Was not compiled with compilation features (CAN_COMPILE symbol)!\n");
#endif
	}

	if(binaryfile){
		load_gvm_file(vm, binaryfile);
	}

	if(setwatermark){
		if(!geni_vm_set_watermark(vm, watermarkfile, binaryfile)){
			printf("Could not set watermark file !");
			return_code = 12;
			goto safe_exit;
		}
	}

	if(getwatermark){
		if(!geni_vm_get_watermark(vm, watermarkfile)){
			printf("Could not get watermark file !");
			//could not read file
			return_code = 11;
			goto safe_exit;
		}
	}

	if(run){
		//run a binary, it could come from file or memory !
		if(verbose) printf("Running code..\n");
		geni_vm_run( vm );
	}

	if(dump){
#ifdef CAN_DUMP
		if(verbose) printf("Dumping VM..\n");
		geni_vm_dump( vm );
#else
		printf("Was not compiled with CAN_DUMP symbol !\n");
#endif
	}
	
safe_exit:
	if(verbose) printf("Safe exit..");
	geni_vm_delete( vm );	
	if(verbose) printf("ok\n");
	return return_code;
}
#endif