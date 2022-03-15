/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: Signal.proto */

#ifndef PROTOBUF_C_Signal_2eproto__INCLUDED
#define PROTOBUF_C_Signal_2eproto__INCLUDED

#include <protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1003000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1003003 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _Signal Signal;


/* --- enums --- */


/* --- messages --- */

struct  _Signal
{
  ProtobufCMessage base;
  char *name;
};
#define SIGNAL__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&signal__descriptor) \
    , (char *)protobuf_c_empty_string }


/* Signal methods */
void   signal__init
                     (Signal         *message);
size_t signal__get_packed_size
                     (const Signal   *message);
size_t signal__pack
                     (const Signal   *message,
                      uint8_t             *out);
size_t signal__pack_to_buffer
                     (const Signal   *message,
                      ProtobufCBuffer     *buffer);
Signal *
       signal__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   signal__free_unpacked
                     (Signal *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*Signal_Closure)
                 (const Signal *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor signal__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_Signal_2eproto__INCLUDED */