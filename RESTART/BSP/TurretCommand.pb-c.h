/* Generated by the protocol buffer compiler.  DO NOT EDIT! */
/* Generated from: TurretCommand.proto */

#ifndef PROTOBUF_C_TurretCommand_2eproto__INCLUDED
#define PROTOBUF_C_TurretCommand_2eproto__INCLUDED

#include <protobuf-c.h>

PROTOBUF_C__BEGIN_DECLS

#if PROTOBUF_C_VERSION_NUMBER < 1003000
# error This file was generated by a newer version of protoc-c which is incompatible with your libprotobuf-c headers. Please update your headers.
#elif 1003003 < PROTOBUF_C_MIN_COMPILER_VERSION
# error This file was generated by an older version of protoc-c which is incompatible with your libprotobuf-c headers. Please regenerate this file with a newer version of protoc-c.
#endif


typedef struct _TurretCommand TurretCommand;


/* --- enums --- */


/* --- messages --- */

struct  _TurretCommand
{
  ProtobufCMessage base;
  uint32_t command;
  float yaw;
  float pitch;
  float diatance;
};
#define TURRET_COMMAND__INIT \
 { PROTOBUF_C_MESSAGE_INIT (&turret_command__descriptor) \
    , 0, 0, 0, 0 }


/* TurretCommand methods */
void   turret_command__init
                     (TurretCommand         *message);
size_t turret_command__get_packed_size
                     (const TurretCommand   *message);
size_t turret_command__pack
                     (const TurretCommand   *message,
                      uint8_t             *out);
size_t turret_command__pack_to_buffer
                     (const TurretCommand   *message,
                      ProtobufCBuffer     *buffer);
TurretCommand *
       turret_command__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);
void   turret_command__free_unpacked
                     (TurretCommand *message,
                      ProtobufCAllocator *allocator);
/* --- per-message closures --- */

typedef void (*TurretCommand_Closure)
                 (const TurretCommand *message,
                  void *closure_data);

/* --- services --- */


/* --- descriptors --- */

extern const ProtobufCMessageDescriptor turret_command__descriptor;

PROTOBUF_C__END_DECLS


#endif  /* PROTOBUF_C_TurretCommand_2eproto__INCLUDED */
