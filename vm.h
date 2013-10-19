//vm.h
/***
 * Author: GeNi
 * Date: Mars 2011
 ***/
#ifndef _VM_HEADER	//Include once
#define _VM_HEADER

#ifdef Debug
	#define CAN_DUMP
//	#define VM_HEAP_TRACE
	#define VM_DEBUG
//	#define VM_DEBUG_STACK
#endif
#ifdef Library
	//Nothing special to do
#else
	#define CAN_COMPILE
	#define HAS_CORE
#endif

#include <windows.h>
#include <stdio.h>

#define GENI_VM_VERSION 0x0101

#define DEFAULT_STACK_SIZE 64
#define DEFAULT_CALLSTACK_SIZE 64
#define MAX_REGISTER 32
#define DEFAULT_MEMORY_SIZE 64
#define CODE_CHUNK_SIZE 10

#define UNDEF32					OS_DWORD, OK_CONSTANT,		OM_NORMAL,			0
#define CST32( num )			OS_DWORD, OK_CONSTANT,		OM_NORMAL,			num
#define REG32( num )			OS_DWORD, OK_REGISTER,		OM_NORMAL,			num
#define REG32PTR( num )			OS_DWORD, OK_REGISTER,		OM_POINTER,			num
#define REG32HPTR( num )		OS_DWORD, OK_REGISTER,		OM_HOST_POINTER,	num
#define REG32MODE( num, mode )	OS_DWORD, OK_REGISTER,		mode,				num
#define VMM32( num )			OS_DWORD, OK_VM_MEMORY,		OM_NORMAL,			num
#define VMM32PTR( num )			OS_DWORD, OK_VM_MEMORY,		OM_POINTER,			num
#define VMM32HPTR( num )		OS_DWORD, OK_VM_MEMORY,		OM_HOST_POINTER,	num
#define VMM32MODE( num, mode )	OS_DWORD, OK_VM_MEMORY,		mode,				num
#define HSTM32( num )			OS_DWORD, OK_HOST_MEMORY,	OM_NORMAL,			num
#define HSTM32PTR( num )		OS_DWORD, OK_HOST_MEMORY,	OM_POINTER,			num
#define HSTM32MODE( num, mode )	OS_DWORD, OK_HOST_MEMORY,	mode,				num

//~ typedef unsigned char BOOL;
#ifndef BYTE
typedef unsigned char BYTE;
#endif
#ifndef WORD
typedef unsigned short WORD;
#endif
#ifndef DWORD
typedef unsigned long DWORD;
#endif

typedef enum{
	OC_MOV = 0,
	OC_ADD,
	OC_SUB,
	OC_DIV,
	OC_MUL,
	OC_AND,
	OC_OR,
	OC_XOR,
	OC_NOT,
	OC_JMP,
	OC_PUSH, 
	OC_POP,
	OC_HCALL,
	OC_CALL, 
	OC_RET, 
	OC_CMP,
	OC_JZ,
	OC_JNZ,
	OC_JG,
	OC_JL,
	OC_JGE,
	OC_JLE,
	OC_VM2H,
	OC_H2VM,
	OC_PRINTL,	// Output a data with a linefeed (printf("%s\n",)
	OC_PRINT,
	OC_STOP,
	OC_MOVZB,
	OC_MOVZW,
	OC_SHL,
	OC_SHR,
	OC_NOP,
	OC_MKCB,	// CREATE A CALLBACK	: ip, stack-size to copy from HOST to VM STACK
	OC_RMCB,	// DESTROY A CALLBACK: ip, not-used
	OC_DUMP,	// DUMP FROM left for "right" count values
	OC_DEBUGBREAK,
	OC_SPRINTF,
	OC_NEG,
	OC_API,
	OC_HJUMP,
	//OC_FINDDW,	// Find the dword matching leftop.dword from rightop (incrementing rightop value by 4), returning rightop modified.
	//OC_REPZ,		// repz r4, 1; leftop = counter, rightop = instruction count (after current IP) ??? not finished to be digged !
	//~ OC_GETBC,	// GetBytecode function for the given ID
	//~ OC_SETBC,	// Set Bytecode function for the given ID
	//~ OC_BP,		// breakpoint
	MAX_OPCODES		// MUST be the last entry!
} OPCODES;

typedef enum{
	OK_CONSTANT = 0,
	OK_REGISTER,
	OK_VM_MEMORY,
	OK_HOST_MEMORY
} OPERAND_KIND;

typedef enum{			// This is something like a segment in traditional assembly.

	OM_NORMAL = 0,		// constant
	OM_REG_POINTER,		// register [ value ]
	OM_VM_POINTER,		// VM pointer
	OM_HOST_POINTER		// Host pointer
						// Code pointer
						// Stack pointer
} OPERAND_MODE;

typedef enum{
	OS_BYTE  = 1,
	OS_WORD  = 2,
	OS_DWORD = 4
} OPERAND_SIZE, OPCODE_SIZE;

typedef struct _GENI_OPERAND{
	OPERAND_SIZE size;		//Currently not used, intend to be Byte, Word, Dword (, String ...) : more a datatype indicator
	OPERAND_KIND type;
	OPERAND_MODE mode;
	union{			//value
		BYTE byte;
		WORD word;
		DWORD dword;
	} ;
} GENI_OPERAND;

typedef struct _GENI_OPCODE{
	OPCODE_SIZE size;	//8, 16, 24, 32 bits
	OPCODES instruction;		//OPCODES
	GENI_OPERAND leftop;
	GENI_OPERAND rightop;
} GENI_OPCODE;

typedef struct _GENI_CODE{
	DWORD size;
	DWORD ip;
	BYTE stop;
	GENI_OPCODE* opcodes;
	//~ GENI_CODE* next;
} GENI_CODE;

typedef struct _GENI_MEMORY{
	DWORD size;
	DWORD storedbytes;
	BYTE* ptr;
} GENI_MEMORY;

typedef struct _GENI_STACK{
	DWORD size;
	DWORD sp;
	DWORD* ptr;
} GENI_STACK;

typedef struct _GENI_FLAGS{ //Like assembly processor flags, they are indicator of the last operation.
	BOOL zero;
	BOOL greater;
	BOOL lower;
	BOOL debug;
	BOOL critical;
} GENI_FLAGS;

typedef struct _GENI_VM_INSTRUCTIONDEF{
#ifdef CAN_COMPILE
	char* mnemonic;
#endif
	DWORD/* fnHandler* */ handler;
} GENI_VM_INSTRUCTIONDEF;
//~ typedef struct _GENI_VM_INSTRUCTIONSET{
	//~ GENI_VM_INSTRUCTIONDEF set[ 256 ];
//~ } GENI_VM_INSTRUCTIONSET;
typedef struct _GENI_REGISTER{
	DWORD r[ MAX_REGISTER ];
} GENI_REGISTER;


typedef struct _GENI_CALLBACK{
	DWORD host_address;
	DWORD vm_ip;
	DWORD arg_count;
	DWORD vmptr;
	DWORD* host_stack;
	DWORD return_value;
} GENI_CALLBACK;

typedef struct _GENI_CALLBACKS{
	DWORD count;
	GENI_CALLBACK* items;
} GENI_CALLBACKS;

typedef DWORD (fnAPIHandler) (DWORD* , DWORD );
typedef struct _GENI_API{
	DWORD id;
	char* name;
	fnAPIHandler* handler;
} GENI_API;

typedef struct _GENI_APIS{
	DWORD count;
	GENI_API* items;
} GENI_APIS;

typedef struct _GENI_VM{
	GENI_CODE code;
	GENI_MEMORY memory;
	GENI_REGISTER registers;
	GENI_STACK stack;
	GENI_FLAGS flags;
	GENI_VM_INSTRUCTIONDEF opcodeset[256];
	GENI_STACK callstack;
	GENI_CALLBACKS callback;
	GENI_APIS api;
	CRITICAL_SECTION criticalsection;
	HANDLE semaphore;
	HANDLE heap;
	DWORD return_value;
	void* userdata;
} GENI_VM;


//COMPILATION SPECIFICS
typedef enum{
	SEG_CODE=0,
	SEG_DATA,
	SEG_UNKNOW
} GENI_SEGMENT;

#ifdef CAN_COMPILE
typedef struct _GENI_COMPILER_NAMESPACE_REF{
	DWORD address;			//Real address in codesegment / datasegment, *((DWORD)(segement + address)) = ptr...
	DWORD source_line;		//line in code-source
	GENI_SEGMENT segment;
} GENI_COMPILER_NAMESPACE_REF;

typedef struct _GENI_COMPILER_NAMESPACE{
	char* name;
	DWORD address;
	DWORD line_defined;
	GENI_SEGMENT segment;		//to allow to refer to DATA or CODE labels !
	//and a pointer to keep trace of reference to a not yet declared label.
	GENI_COMPILER_NAMESPACE_REF* references;
	DWORD ref_count;
} GENI_COMPILER_NAMESPACE;

typedef struct _GENI_COMPILER_CONTEXT{
	GENI_VM* vm;
	DWORD current_line;	//current line in the input buffer
	DWORD name_count;
	GENI_COMPILER_NAMESPACE* names;	// an array of names
	GENI_SEGMENT current_segment;
	GENI_OPCODE* code_segment;
	DWORD code_size;
	DWORD instruction_count;
	BOOL is_operand_left;
	BYTE* memory_segment;
	DWORD memory_size;
	DWORD memory_current_offset;
	DWORD memory_max_offset;		//keep trace of minimal memory size !
	GENI_COMPILER_NAMESPACE* post_build;			//used for encryption
	//BYTECODE modifiers ...
	//MEMORY_STATE (build from source code !)
} GENI_COMPILER_CONTEXT;
#endif

typedef struct _GENI_VM_WATERMARK_ITERATOR{
	BYTE byte_idx;
	DWORD current_opcode;
	GENI_OPCODE* opcodes;
} GENI_VM_WATERMARK_ITERATOR;

typedef struct _GENI_BINARY_HEADER{
	DWORD markvm;			// = GENI
	DWORD markversion;		// = VMWW
	DWORD memory_size;
	DWORD memory_stored;
	DWORD instruction_count;
	BYTE * data;
} GENI_BINARY_HEADER;

typedef void (fnHandler)(GENI_VM* , GENI_OPCODE* );

#ifdef CAN_COMPILE
#define MAP_OPCODE( VM, handlerptr, opcode, mnemonicstr ) \
	VM->opcodeset[ opcode ].handler = (DWORD)handlerptr; \
	VM->opcodeset[ opcode ].mnemonic = mnemonicstr;
#else
#define MAP_OPCODE( VM, handlerptr, opcode, mnemonicstr ) \
	VM->opcodeset[ opcode ].handler = (DWORD)handlerptr;
#endif

void OPCODE32( GENI_OPCODE* opptr, DWORD opcode, 
	DWORD oplsize, BYTE oplkind, BYTE oplmode, DWORD oplvalue, 
	DWORD oprsize, BYTE oprkind, BYTE oprmode, DWORD oprvalue ){
	(opptr)->size=OS_DWORD;
	(opptr)->instruction=opcode;
	(opptr)->leftop.size=oplsize;
	(opptr)->leftop.type=oplkind;
	(opptr)->leftop.mode=oplmode;
	(opptr)->leftop.dword=oplvalue;
	(opptr)->rightop.size=oprsize;
	(opptr)->rightop.type=oprkind;
	(opptr)->rightop.mode=oprmode;
	(opptr)->rightop.dword=oprvalue;
}

#ifdef CAN_DUMP
void geni_vm_dump_memory(DWORD address, DWORD offset, DWORD size);
void geni_vm_dump_operand( GENI_OPERAND* operand );
void geni_vm_dump_opcode( GENI_VM* vm, GENI_OPCODE* opcode );
void geni_vm_dump_stack( char* name, GENI_STACK* stack );
void geni_vm_dump( GENI_VM* vm);
#endif
DWORD geni_vm_do_sub(GENI_CALLBACK* cb, DWORD* host_stackpointer);
void geni_vm_run( GENI_VM* vm );
void geni_vm_set_userdata( GENI_VM* vm, void* userdata);
void* geni_vm_get_userdata( GENI_VM* vm);
#endif	// #ifdef _VM_HEADER