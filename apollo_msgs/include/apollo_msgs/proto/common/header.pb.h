// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: apollo_msgs/proto/common/header.proto

#ifndef PROTOBUF_INCLUDED_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto
#define PROTOBUF_INCLUDED_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "apollo_msgs/proto/common/error_code.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto 

namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto
namespace apollo {
namespace common {
class Header;
class HeaderDefaultTypeInternal;
extern HeaderDefaultTypeInternal _Header_default_instance_;
}  // namespace common
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::common::Header* Arena::CreateMaybeMessage<::apollo::common::Header>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace common {

// ===================================================================

class Header : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.common.Header) */ {
 public:
  Header();
  virtual ~Header();

  Header(const Header& from);

  inline Header& operator=(const Header& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Header(Header&& from) noexcept
    : Header() {
    *this = ::std::move(from);
  }

  inline Header& operator=(Header&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Header& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Header* internal_default_instance() {
    return reinterpret_cast<const Header*>(
               &_Header_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Header* other);
  friend void swap(Header& a, Header& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Header* New() const final {
    return CreateMaybeMessage<Header>(NULL);
  }

  Header* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Header>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Header& from);
  void MergeFrom(const Header& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Header* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // string module_name = 2;
  void clear_module_name();
  static const int kModuleNameFieldNumber = 2;
  const ::std::string& module_name() const;
  void set_module_name(const ::std::string& value);
  #if LANG_CXX11
  void set_module_name(::std::string&& value);
  #endif
  void set_module_name(const char* value);
  void set_module_name(const char* value, size_t size);
  ::std::string* mutable_module_name();
  ::std::string* release_module_name();
  void set_allocated_module_name(::std::string* module_name);

  // .apollo.common.StatusPb status = 8;
  bool has_status() const;
  void clear_status();
  static const int kStatusFieldNumber = 8;
  private:
  const ::apollo::common::StatusPb& _internal_status() const;
  public:
  const ::apollo::common::StatusPb& status() const;
  ::apollo::common::StatusPb* release_status();
  ::apollo::common::StatusPb* mutable_status();
  void set_allocated_status(::apollo::common::StatusPb* status);

  // double timestamp_sec = 1;
  void clear_timestamp_sec();
  static const int kTimestampSecFieldNumber = 1;
  double timestamp_sec() const;
  void set_timestamp_sec(double value);

  // uint64 lidar_timestamp = 4;
  void clear_lidar_timestamp();
  static const int kLidarTimestampFieldNumber = 4;
  ::google::protobuf::uint64 lidar_timestamp() const;
  void set_lidar_timestamp(::google::protobuf::uint64 value);

  // uint64 camera_timestamp = 5;
  void clear_camera_timestamp();
  static const int kCameraTimestampFieldNumber = 5;
  ::google::protobuf::uint64 camera_timestamp() const;
  void set_camera_timestamp(::google::protobuf::uint64 value);

  // uint32 sequence_num = 3;
  void clear_sequence_num();
  static const int kSequenceNumFieldNumber = 3;
  ::google::protobuf::uint32 sequence_num() const;
  void set_sequence_num(::google::protobuf::uint32 value);

  // uint32 version = 7;
  void clear_version();
  static const int kVersionFieldNumber = 7;
  ::google::protobuf::uint32 version() const;
  void set_version(::google::protobuf::uint32 value);

  // uint64 radar_timestamp = 6;
  void clear_radar_timestamp();
  static const int kRadarTimestampFieldNumber = 6;
  ::google::protobuf::uint64 radar_timestamp() const;
  void set_radar_timestamp(::google::protobuf::uint64 value);

  // @@protoc_insertion_point(class_scope:apollo.common.Header)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr module_name_;
  ::apollo::common::StatusPb* status_;
  double timestamp_sec_;
  ::google::protobuf::uint64 lidar_timestamp_;
  ::google::protobuf::uint64 camera_timestamp_;
  ::google::protobuf::uint32 sequence_num_;
  ::google::protobuf::uint32 version_;
  ::google::protobuf::uint64 radar_timestamp_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Header

// double timestamp_sec = 1;
inline void Header::clear_timestamp_sec() {
  timestamp_sec_ = 0;
}
inline double Header::timestamp_sec() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.timestamp_sec)
  return timestamp_sec_;
}
inline void Header::set_timestamp_sec(double value) {
  
  timestamp_sec_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.Header.timestamp_sec)
}

// string module_name = 2;
inline void Header::clear_module_name() {
  module_name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& Header::module_name() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.module_name)
  return module_name_.GetNoArena();
}
inline void Header::set_module_name(const ::std::string& value) {
  
  module_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.common.Header.module_name)
}
#if LANG_CXX11
inline void Header::set_module_name(::std::string&& value) {
  
  module_name_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.common.Header.module_name)
}
#endif
inline void Header::set_module_name(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  module_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.common.Header.module_name)
}
inline void Header::set_module_name(const char* value, size_t size) {
  
  module_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.common.Header.module_name)
}
inline ::std::string* Header::mutable_module_name() {
  
  // @@protoc_insertion_point(field_mutable:apollo.common.Header.module_name)
  return module_name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Header::release_module_name() {
  // @@protoc_insertion_point(field_release:apollo.common.Header.module_name)
  
  return module_name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Header::set_allocated_module_name(::std::string* module_name) {
  if (module_name != NULL) {
    
  } else {
    
  }
  module_name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), module_name);
  // @@protoc_insertion_point(field_set_allocated:apollo.common.Header.module_name)
}

// uint32 sequence_num = 3;
inline void Header::clear_sequence_num() {
  sequence_num_ = 0u;
}
inline ::google::protobuf::uint32 Header::sequence_num() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.sequence_num)
  return sequence_num_;
}
inline void Header::set_sequence_num(::google::protobuf::uint32 value) {
  
  sequence_num_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.Header.sequence_num)
}

// uint64 lidar_timestamp = 4;
inline void Header::clear_lidar_timestamp() {
  lidar_timestamp_ = GOOGLE_ULONGLONG(0);
}
inline ::google::protobuf::uint64 Header::lidar_timestamp() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.lidar_timestamp)
  return lidar_timestamp_;
}
inline void Header::set_lidar_timestamp(::google::protobuf::uint64 value) {
  
  lidar_timestamp_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.Header.lidar_timestamp)
}

// uint64 camera_timestamp = 5;
inline void Header::clear_camera_timestamp() {
  camera_timestamp_ = GOOGLE_ULONGLONG(0);
}
inline ::google::protobuf::uint64 Header::camera_timestamp() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.camera_timestamp)
  return camera_timestamp_;
}
inline void Header::set_camera_timestamp(::google::protobuf::uint64 value) {
  
  camera_timestamp_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.Header.camera_timestamp)
}

// uint64 radar_timestamp = 6;
inline void Header::clear_radar_timestamp() {
  radar_timestamp_ = GOOGLE_ULONGLONG(0);
}
inline ::google::protobuf::uint64 Header::radar_timestamp() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.radar_timestamp)
  return radar_timestamp_;
}
inline void Header::set_radar_timestamp(::google::protobuf::uint64 value) {
  
  radar_timestamp_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.Header.radar_timestamp)
}

// uint32 version = 7;
inline void Header::clear_version() {
  version_ = 0u;
}
inline ::google::protobuf::uint32 Header::version() const {
  // @@protoc_insertion_point(field_get:apollo.common.Header.version)
  return version_;
}
inline void Header::set_version(::google::protobuf::uint32 value) {
  
  version_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.Header.version)
}

// .apollo.common.StatusPb status = 8;
inline bool Header::has_status() const {
  return this != internal_default_instance() && status_ != NULL;
}
inline const ::apollo::common::StatusPb& Header::_internal_status() const {
  return *status_;
}
inline const ::apollo::common::StatusPb& Header::status() const {
  const ::apollo::common::StatusPb* p = status_;
  // @@protoc_insertion_point(field_get:apollo.common.Header.status)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::StatusPb*>(
      &::apollo::common::_StatusPb_default_instance_);
}
inline ::apollo::common::StatusPb* Header::release_status() {
  // @@protoc_insertion_point(field_release:apollo.common.Header.status)
  
  ::apollo::common::StatusPb* temp = status_;
  status_ = NULL;
  return temp;
}
inline ::apollo::common::StatusPb* Header::mutable_status() {
  
  if (status_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::StatusPb>(GetArenaNoVirtual());
    status_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.common.Header.status)
  return status_;
}
inline void Header::set_allocated_status(::apollo::common::StatusPb* status) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(status_);
  }
  if (status) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      status = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, status, submessage_arena);
    }
    
  } else {
    
  }
  status_ = status;
  // @@protoc_insertion_point(field_set_allocated:apollo.common.Header.status)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace common
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto
