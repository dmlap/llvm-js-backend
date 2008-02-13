// RUN: %llvmgxx -S -O0 -emit-llvm %s -o - | grep retval | grep S242 | grep {i32 1} | count 2

// Test that all 8 bytes of ret in check242 are copied.  llvm-gcc was
// treating S242 as if it were S93, which does not need to have the
// last 4 padding bytes copied.
typedef __builtin_va_list va_list;
typedef unsigned long size_t;
void *memset(void *, int, size_t);
struct S92 { int a:14; } ;
 extern struct S92 s92;

 struct S92 check92 () { struct S92 ret;
 memset (&ret, 0, sizeof (ret));
 ret.a = s92.a;
 return ret; }

struct S93 { __attribute__((aligned (8))) void * a; } ;
 extern struct S93 s93;
 struct S93 check93 () { 
  struct S93 ret;
 memset (&ret, 0, sizeof (ret));
 ret.a = s93.a; 
 return ret; }

struct S242 { char * a;int b[1]; } ;
 extern struct S242 s242;

 struct S242 check242 () {
 struct S242 ret;
 memset (&ret, 0, sizeof (ret));
 ret.a = s242.a;
 ret.b[0] = s242.b[0];
 return ret; }

void check93va (int z, ...) { 
 struct S93 arg;
 va_list ap;
 __builtin_va_start(ap,z);
 arg = __builtin_va_arg(ap,struct S93);
  __builtin_va_end(ap); }

void check242va (int z, ...) { 
struct S242 arg;
va_list ap;
__builtin_va_start(ap,z);
 arg = __builtin_va_arg(ap,struct S242);
 __builtin_va_end(ap); }

