/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: recieve_sentry.proto */

#ifndef PROTOBUF_C_recieve_5fsentry_2eproto__INCLUDED
#define PROTOBUF_C_recieve_5fsentry_2eproto__INCLUDED

#include <protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1003000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1003003 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _EToV__Frame EToV__Frame;


/* --- enums --- */


/* --- messages --- */

struct  _EToV__Frame
{
  ProtobufCMessage base;
  int32_t currentpitch_;
  int32_t currentyaw_;
  int32_t currentcolor_;
  int32_t bulletspeed_;
  int32_t booldvol_;
  int32_t mode_;
  int32_t guard_;
  int32_t hero_;
  int32_t engineer_;
  int32_t infantry3_;
  int32_t infantry4_;
  int32_t infantry5_;
  int32_t base_;
};
#define E_TO_V__FRAME__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&e_to_v__frame__descriptor) \
    , 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }


/* EToV__Frame methods */
void   e_to_v__frame__init
                     (EToV__Frame         *message);
size_t e_to_v__frame__get_packed_size
                     (const EToV__Frame   *message);
size_t e_to_v__frame__pack
                     (const EToV__Frame   *message,
                      uint8_t             *out);
size_t e_to_v__frame__pack_to_buffer
                     (const EToV__Frame   *message,
                      ProtobufCBuffer     *buffer);
EToV__Frame *
       e_to_v__frame__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   e_to_v__frame__free_unpacked
                     (EToV__Frame *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*EToV__Frame_Closure)
                 (const EToV__Frame *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor e_to_v__frame__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_recieve_5fsentry_2eproto__INCLUDED */