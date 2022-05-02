// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: apollo_msgs/proto/common/gnss_status.proto

#ifndef PROTOBUF_INCLUDED_apollo_5fmsgs_2fproto_2fcommon_2fgnss_5fstatus_2eproto
#define PROTOBUF_INCLUDED_apollo_5fmsgs_2fproto_2fcommon_2fgnss_5fstatus_2eproto

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
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "apollo_msgs/proto/common/header.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgnss_5fstatus_2eproto 

namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgnss_5fstatus_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[3];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgnss_5fstatus_2eproto
namespace apollo {
namespace common {
namespace gnss_status {
class GnssStatus;
class GnssStatusDefaultTypeInternal;
extern GnssStatusDefaultTypeInternal _GnssStatus_default_instance_;
class InsStatus;
class InsStatusDefaultTypeInternal;
extern InsStatusDefaultTypeInternal _InsStatus_default_instance_;
class StreamStatus;
class StreamStatusDefaultTypeInternal;
extern StreamStatusDefaultTypeInternal _StreamStatus_default_instance_;
}  // namespace gnss_status
}  // namespace common
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::common::gnss_status::GnssStatus* Arena::CreateMaybeMessage<::apollo::common::gnss_status::GnssStatus>(Arena*);
template<> ::apollo::common::gnss_status::InsStatus* Arena::CreateMaybeMessage<::apollo::common::gnss_status::InsStatus>(Arena*);
template<> ::apollo::common::gnss_status::StreamStatus* Arena::CreateMaybeMessage<::apollo::common::gnss_status::StreamStatus>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace common {
namespace gnss_status {

enum StreamStatus_Type {
  StreamStatus_Type_DISCONNECTED = 0,
  StreamStatus_Type_CONNECTED = 1,
  StreamStatus_Type_StreamStatus_Type_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  StreamStatus_Type_StreamStatus_Type_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool StreamStatus_Type_IsValid(int value);
const StreamStatus_Type StreamStatus_Type_Type_MIN = StreamStatus_Type_DISCONNECTED;
const StreamStatus_Type StreamStatus_Type_Type_MAX = StreamStatus_Type_CONNECTED;
const int StreamStatus_Type_Type_ARRAYSIZE = StreamStatus_Type_Type_MAX + 1;

const ::google::protobuf::EnumDescriptor* StreamStatus_Type_descriptor();
inline const ::std::string& StreamStatus_Type_Name(StreamStatus_Type value) {
  return ::google::protobuf::internal::NameOfEnum(
    StreamStatus_Type_descriptor(), value);
}
inline bool StreamStatus_Type_Parse(
    const ::std::string& name, StreamStatus_Type* value) {
  return ::google::protobuf::internal::ParseNamedEnum<StreamStatus_Type>(
    StreamStatus_Type_descriptor(), name, value);
}
enum InsStatus_Type {
  InsStatus_Type_INVALID = 0,
  InsStatus_Type_CONVERGING = 1,
  InsStatus_Type_GOOD = 2,
  InsStatus_Type_InsStatus_Type_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  InsStatus_Type_InsStatus_Type_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool InsStatus_Type_IsValid(int value);
const InsStatus_Type InsStatus_Type_Type_MIN = InsStatus_Type_INVALID;
const InsStatus_Type InsStatus_Type_Type_MAX = InsStatus_Type_GOOD;
const int InsStatus_Type_Type_ARRAYSIZE = InsStatus_Type_Type_MAX + 1;

const ::google::protobuf::EnumDescriptor* InsStatus_Type_descriptor();
inline const ::std::string& InsStatus_Type_Name(InsStatus_Type value) {
  return ::google::protobuf::internal::NameOfEnum(
    InsStatus_Type_descriptor(), value);
}
inline bool InsStatus_Type_Parse(
    const ::std::string& name, InsStatus_Type* value) {
  return ::google::protobuf::internal::ParseNamedEnum<InsStatus_Type>(
    InsStatus_Type_descriptor(), name, value);
}
// ===================================================================

class StreamStatus : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.common.gnss_status.StreamStatus) */ {
 public:
  StreamStatus();
  virtual ~StreamStatus();

  StreamStatus(const StreamStatus& from);

  inline StreamStatus& operator=(const StreamStatus& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  StreamStatus(StreamStatus&& from) noexcept
    : StreamStatus() {
    *this = ::std::move(from);
  }

  inline StreamStatus& operator=(StreamStatus&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const StreamStatus& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const StreamStatus* internal_default_instance() {
    return reinterpret_cast<const StreamStatus*>(
               &_StreamStatus_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(StreamStatus* other);
  friend void swap(StreamStatus& a, StreamStatus& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline StreamStatus* New() const final {
    return CreateMaybeMessage<StreamStatus>(NULL);
  }

  StreamStatus* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<StreamStatus>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const StreamStatus& from);
  void MergeFrom(const StreamStatus& from);
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
  void InternalSwap(StreamStatus* other);
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

  typedef StreamStatus_Type Type;
  static const Type DISCONNECTED =
    StreamStatus_Type_DISCONNECTED;
  static const Type CONNECTED =
    StreamStatus_Type_CONNECTED;
  static inline bool Type_IsValid(int value) {
    return StreamStatus_Type_IsValid(value);
  }
  static const Type Type_MIN =
    StreamStatus_Type_Type_MIN;
  static const Type Type_MAX =
    StreamStatus_Type_Type_MAX;
  static const int Type_ARRAYSIZE =
    StreamStatus_Type_Type_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Type_descriptor() {
    return StreamStatus_Type_descriptor();
  }
  static inline const ::std::string& Type_Name(Type value) {
    return StreamStatus_Type_Name(value);
  }
  static inline bool Type_Parse(const ::std::string& name,
      Type* value) {
    return StreamStatus_Type_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // .apollo.common.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  private:
  const ::apollo::common::Header& _internal_header() const;
  public:
  const ::apollo::common::Header& header() const;
  ::apollo::common::Header* release_header();
  ::apollo::common::Header* mutable_header();
  void set_allocated_header(::apollo::common::Header* header);

  // .apollo.common.gnss_status.StreamStatus.Type ins_stream_type = 2;
  void clear_ins_stream_type();
  static const int kInsStreamTypeFieldNumber = 2;
  ::apollo::common::gnss_status::StreamStatus_Type ins_stream_type() const;
  void set_ins_stream_type(::apollo::common::gnss_status::StreamStatus_Type value);

  // .apollo.common.gnss_status.StreamStatus.Type rtk_stream_in_type = 3;
  void clear_rtk_stream_in_type();
  static const int kRtkStreamInTypeFieldNumber = 3;
  ::apollo::common::gnss_status::StreamStatus_Type rtk_stream_in_type() const;
  void set_rtk_stream_in_type(::apollo::common::gnss_status::StreamStatus_Type value);

  // .apollo.common.gnss_status.StreamStatus.Type rtk_stream_out_type = 4;
  void clear_rtk_stream_out_type();
  static const int kRtkStreamOutTypeFieldNumber = 4;
  ::apollo::common::gnss_status::StreamStatus_Type rtk_stream_out_type() const;
  void set_rtk_stream_out_type(::apollo::common::gnss_status::StreamStatus_Type value);

  // @@protoc_insertion_point(class_scope:apollo.common.gnss_status.StreamStatus)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::apollo::common::Header* header_;
  int ins_stream_type_;
  int rtk_stream_in_type_;
  int rtk_stream_out_type_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgnss_5fstatus_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class InsStatus : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.common.gnss_status.InsStatus) */ {
 public:
  InsStatus();
  virtual ~InsStatus();

  InsStatus(const InsStatus& from);

  inline InsStatus& operator=(const InsStatus& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  InsStatus(InsStatus&& from) noexcept
    : InsStatus() {
    *this = ::std::move(from);
  }

  inline InsStatus& operator=(InsStatus&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const InsStatus& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const InsStatus* internal_default_instance() {
    return reinterpret_cast<const InsStatus*>(
               &_InsStatus_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(InsStatus* other);
  friend void swap(InsStatus& a, InsStatus& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline InsStatus* New() const final {
    return CreateMaybeMessage<InsStatus>(NULL);
  }

  InsStatus* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<InsStatus>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const InsStatus& from);
  void MergeFrom(const InsStatus& from);
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
  void InternalSwap(InsStatus* other);
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

  typedef InsStatus_Type Type;
  static const Type INVALID =
    InsStatus_Type_INVALID;
  static const Type CONVERGING =
    InsStatus_Type_CONVERGING;
  static const Type GOOD =
    InsStatus_Type_GOOD;
  static inline bool Type_IsValid(int value) {
    return InsStatus_Type_IsValid(value);
  }
  static const Type Type_MIN =
    InsStatus_Type_Type_MIN;
  static const Type Type_MAX =
    InsStatus_Type_Type_MAX;
  static const int Type_ARRAYSIZE =
    InsStatus_Type_Type_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Type_descriptor() {
    return InsStatus_Type_descriptor();
  }
  static inline const ::std::string& Type_Name(Type value) {
    return InsStatus_Type_Name(value);
  }
  static inline bool Type_Parse(const ::std::string& name,
      Type* value) {
    return InsStatus_Type_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // .apollo.common.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  private:
  const ::apollo::common::Header& _internal_header() const;
  public:
  const ::apollo::common::Header& header() const;
  ::apollo::common::Header* release_header();
  ::apollo::common::Header* mutable_header();
  void set_allocated_header(::apollo::common::Header* header);

  // .apollo.common.gnss_status.InsStatus.Type type = 2;
  void clear_type();
  static const int kTypeFieldNumber = 2;
  ::apollo::common::gnss_status::InsStatus_Type type() const;
  void set_type(::apollo::common::gnss_status::InsStatus_Type value);

  // @@protoc_insertion_point(class_scope:apollo.common.gnss_status.InsStatus)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::apollo::common::Header* header_;
  int type_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgnss_5fstatus_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class GnssStatus : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.common.gnss_status.GnssStatus) */ {
 public:
  GnssStatus();
  virtual ~GnssStatus();

  GnssStatus(const GnssStatus& from);

  inline GnssStatus& operator=(const GnssStatus& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  GnssStatus(GnssStatus&& from) noexcept
    : GnssStatus() {
    *this = ::std::move(from);
  }

  inline GnssStatus& operator=(GnssStatus&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const GnssStatus& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const GnssStatus* internal_default_instance() {
    return reinterpret_cast<const GnssStatus*>(
               &_GnssStatus_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  void Swap(GnssStatus* other);
  friend void swap(GnssStatus& a, GnssStatus& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline GnssStatus* New() const final {
    return CreateMaybeMessage<GnssStatus>(NULL);
  }

  GnssStatus* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<GnssStatus>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const GnssStatus& from);
  void MergeFrom(const GnssStatus& from);
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
  void InternalSwap(GnssStatus* other);
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

  // .apollo.common.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  private:
  const ::apollo::common::Header& _internal_header() const;
  public:
  const ::apollo::common::Header& header() const;
  ::apollo::common::Header* release_header();
  ::apollo::common::Header* mutable_header();
  void set_allocated_header(::apollo::common::Header* header);

  // bool solution_completed = 2;
  void clear_solution_completed();
  static const int kSolutionCompletedFieldNumber = 2;
  bool solution_completed() const;
  void set_solution_completed(bool value);

  // uint32 solution_status = 3;
  void clear_solution_status();
  static const int kSolutionStatusFieldNumber = 3;
  ::google::protobuf::uint32 solution_status() const;
  void set_solution_status(::google::protobuf::uint32 value);

  // uint32 position_type = 4;
  void clear_position_type();
  static const int kPositionTypeFieldNumber = 4;
  ::google::protobuf::uint32 position_type() const;
  void set_position_type(::google::protobuf::uint32 value);

  // int32 num_sats = 5;
  void clear_num_sats();
  static const int kNumSatsFieldNumber = 5;
  ::google::protobuf::int32 num_sats() const;
  void set_num_sats(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:apollo.common.gnss_status.GnssStatus)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::apollo::common::Header* header_;
  bool solution_completed_;
  ::google::protobuf::uint32 solution_status_;
  ::google::protobuf::uint32 position_type_;
  ::google::protobuf::int32 num_sats_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgnss_5fstatus_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// StreamStatus

// .apollo.common.Header header = 1;
inline bool StreamStatus::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::apollo::common::Header& StreamStatus::_internal_header() const {
  return *header_;
}
inline const ::apollo::common::Header& StreamStatus::header() const {
  const ::apollo::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:apollo.common.gnss_status.StreamStatus.header)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline ::apollo::common::Header* StreamStatus::release_header() {
  // @@protoc_insertion_point(field_release:apollo.common.gnss_status.StreamStatus.header)
  
  ::apollo::common::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::apollo::common::Header* StreamStatus::mutable_header() {
  
  if (header_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.common.gnss_status.StreamStatus.header)
  return header_;
}
inline void StreamStatus::set_allocated_header(::apollo::common::Header* header) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(header_);
  }
  if (header) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      header = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:apollo.common.gnss_status.StreamStatus.header)
}

// .apollo.common.gnss_status.StreamStatus.Type ins_stream_type = 2;
inline void StreamStatus::clear_ins_stream_type() {
  ins_stream_type_ = 0;
}
inline ::apollo::common::gnss_status::StreamStatus_Type StreamStatus::ins_stream_type() const {
  // @@protoc_insertion_point(field_get:apollo.common.gnss_status.StreamStatus.ins_stream_type)
  return static_cast< ::apollo::common::gnss_status::StreamStatus_Type >(ins_stream_type_);
}
inline void StreamStatus::set_ins_stream_type(::apollo::common::gnss_status::StreamStatus_Type value) {
  
  ins_stream_type_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.gnss_status.StreamStatus.ins_stream_type)
}

// .apollo.common.gnss_status.StreamStatus.Type rtk_stream_in_type = 3;
inline void StreamStatus::clear_rtk_stream_in_type() {
  rtk_stream_in_type_ = 0;
}
inline ::apollo::common::gnss_status::StreamStatus_Type StreamStatus::rtk_stream_in_type() const {
  // @@protoc_insertion_point(field_get:apollo.common.gnss_status.StreamStatus.rtk_stream_in_type)
  return static_cast< ::apollo::common::gnss_status::StreamStatus_Type >(rtk_stream_in_type_);
}
inline void StreamStatus::set_rtk_stream_in_type(::apollo::common::gnss_status::StreamStatus_Type value) {
  
  rtk_stream_in_type_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.gnss_status.StreamStatus.rtk_stream_in_type)
}

// .apollo.common.gnss_status.StreamStatus.Type rtk_stream_out_type = 4;
inline void StreamStatus::clear_rtk_stream_out_type() {
  rtk_stream_out_type_ = 0;
}
inline ::apollo::common::gnss_status::StreamStatus_Type StreamStatus::rtk_stream_out_type() const {
  // @@protoc_insertion_point(field_get:apollo.common.gnss_status.StreamStatus.rtk_stream_out_type)
  return static_cast< ::apollo::common::gnss_status::StreamStatus_Type >(rtk_stream_out_type_);
}
inline void StreamStatus::set_rtk_stream_out_type(::apollo::common::gnss_status::StreamStatus_Type value) {
  
  rtk_stream_out_type_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.gnss_status.StreamStatus.rtk_stream_out_type)
}

// -------------------------------------------------------------------

// InsStatus

// .apollo.common.Header header = 1;
inline bool InsStatus::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::apollo::common::Header& InsStatus::_internal_header() const {
  return *header_;
}
inline const ::apollo::common::Header& InsStatus::header() const {
  const ::apollo::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:apollo.common.gnss_status.InsStatus.header)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline ::apollo::common::Header* InsStatus::release_header() {
  // @@protoc_insertion_point(field_release:apollo.common.gnss_status.InsStatus.header)
  
  ::apollo::common::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::apollo::common::Header* InsStatus::mutable_header() {
  
  if (header_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.common.gnss_status.InsStatus.header)
  return header_;
}
inline void InsStatus::set_allocated_header(::apollo::common::Header* header) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(header_);
  }
  if (header) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      header = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:apollo.common.gnss_status.InsStatus.header)
}

// .apollo.common.gnss_status.InsStatus.Type type = 2;
inline void InsStatus::clear_type() {
  type_ = 0;
}
inline ::apollo::common::gnss_status::InsStatus_Type InsStatus::type() const {
  // @@protoc_insertion_point(field_get:apollo.common.gnss_status.InsStatus.type)
  return static_cast< ::apollo::common::gnss_status::InsStatus_Type >(type_);
}
inline void InsStatus::set_type(::apollo::common::gnss_status::InsStatus_Type value) {
  
  type_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.gnss_status.InsStatus.type)
}

// -------------------------------------------------------------------

// GnssStatus

// .apollo.common.Header header = 1;
inline bool GnssStatus::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::apollo::common::Header& GnssStatus::_internal_header() const {
  return *header_;
}
inline const ::apollo::common::Header& GnssStatus::header() const {
  const ::apollo::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:apollo.common.gnss_status.GnssStatus.header)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::common::Header*>(
      &::apollo::common::_Header_default_instance_);
}
inline ::apollo::common::Header* GnssStatus::release_header() {
  // @@protoc_insertion_point(field_release:apollo.common.gnss_status.GnssStatus.header)
  
  ::apollo::common::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::apollo::common::Header* GnssStatus::mutable_header() {
  
  if (header_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.common.gnss_status.GnssStatus.header)
  return header_;
}
inline void GnssStatus::set_allocated_header(::apollo::common::Header* header) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(header_);
  }
  if (header) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      header = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:apollo.common.gnss_status.GnssStatus.header)
}

// bool solution_completed = 2;
inline void GnssStatus::clear_solution_completed() {
  solution_completed_ = false;
}
inline bool GnssStatus::solution_completed() const {
  // @@protoc_insertion_point(field_get:apollo.common.gnss_status.GnssStatus.solution_completed)
  return solution_completed_;
}
inline void GnssStatus::set_solution_completed(bool value) {
  
  solution_completed_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.gnss_status.GnssStatus.solution_completed)
}

// uint32 solution_status = 3;
inline void GnssStatus::clear_solution_status() {
  solution_status_ = 0u;
}
inline ::google::protobuf::uint32 GnssStatus::solution_status() const {
  // @@protoc_insertion_point(field_get:apollo.common.gnss_status.GnssStatus.solution_status)
  return solution_status_;
}
inline void GnssStatus::set_solution_status(::google::protobuf::uint32 value) {
  
  solution_status_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.gnss_status.GnssStatus.solution_status)
}

// uint32 position_type = 4;
inline void GnssStatus::clear_position_type() {
  position_type_ = 0u;
}
inline ::google::protobuf::uint32 GnssStatus::position_type() const {
  // @@protoc_insertion_point(field_get:apollo.common.gnss_status.GnssStatus.position_type)
  return position_type_;
}
inline void GnssStatus::set_position_type(::google::protobuf::uint32 value) {
  
  position_type_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.gnss_status.GnssStatus.position_type)
}

// int32 num_sats = 5;
inline void GnssStatus::clear_num_sats() {
  num_sats_ = 0;
}
inline ::google::protobuf::int32 GnssStatus::num_sats() const {
  // @@protoc_insertion_point(field_get:apollo.common.gnss_status.GnssStatus.num_sats)
  return num_sats_;
}
inline void GnssStatus::set_num_sats(::google::protobuf::int32 value) {
  
  num_sats_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.gnss_status.GnssStatus.num_sats)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace gnss_status
}  // namespace common
}  // namespace apollo

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::apollo::common::gnss_status::StreamStatus_Type> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::common::gnss_status::StreamStatus_Type>() {
  return ::apollo::common::gnss_status::StreamStatus_Type_descriptor();
}
template <> struct is_proto_enum< ::apollo::common::gnss_status::InsStatus_Type> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::common::gnss_status::InsStatus_Type>() {
  return ::apollo::common::gnss_status::InsStatus_Type_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_apollo_5fmsgs_2fproto_2fcommon_2fgnss_5fstatus_2eproto
