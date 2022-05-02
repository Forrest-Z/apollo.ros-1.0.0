// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: apollo_msgs/proto/localization/camera_parameter.proto

#ifndef PROTOBUF_INCLUDED_apollo_5fmsgs_2fproto_2flocalization_2fcamera_5fparameter_2eproto
#define PROTOBUF_INCLUDED_apollo_5fmsgs_2fproto_2flocalization_2fcamera_5fparameter_2eproto

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
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_apollo_5fmsgs_2fproto_2flocalization_2fcamera_5fparameter_2eproto 

namespace protobuf_apollo_5fmsgs_2fproto_2flocalization_2fcamera_5fparameter_2eproto {
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
}  // namespace protobuf_apollo_5fmsgs_2fproto_2flocalization_2fcamera_5fparameter_2eproto
namespace apollo {
namespace localization {
class CameraExtrinsicParameter;
class CameraExtrinsicParameterDefaultTypeInternal;
extern CameraExtrinsicParameterDefaultTypeInternal _CameraExtrinsicParameter_default_instance_;
class CameraIntrinsicParameter;
class CameraIntrinsicParameterDefaultTypeInternal;
extern CameraIntrinsicParameterDefaultTypeInternal _CameraIntrinsicParameter_default_instance_;
class CameraParameter;
class CameraParameterDefaultTypeInternal;
extern CameraParameterDefaultTypeInternal _CameraParameter_default_instance_;
}  // namespace localization
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::localization::CameraExtrinsicParameter* Arena::CreateMaybeMessage<::apollo::localization::CameraExtrinsicParameter>(Arena*);
template<> ::apollo::localization::CameraIntrinsicParameter* Arena::CreateMaybeMessage<::apollo::localization::CameraIntrinsicParameter>(Arena*);
template<> ::apollo::localization::CameraParameter* Arena::CreateMaybeMessage<::apollo::localization::CameraParameter>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace localization {

// ===================================================================

class CameraIntrinsicParameter : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.localization.CameraIntrinsicParameter) */ {
 public:
  CameraIntrinsicParameter();
  virtual ~CameraIntrinsicParameter();

  CameraIntrinsicParameter(const CameraIntrinsicParameter& from);

  inline CameraIntrinsicParameter& operator=(const CameraIntrinsicParameter& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  CameraIntrinsicParameter(CameraIntrinsicParameter&& from) noexcept
    : CameraIntrinsicParameter() {
    *this = ::std::move(from);
  }

  inline CameraIntrinsicParameter& operator=(CameraIntrinsicParameter&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const CameraIntrinsicParameter& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CameraIntrinsicParameter* internal_default_instance() {
    return reinterpret_cast<const CameraIntrinsicParameter*>(
               &_CameraIntrinsicParameter_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(CameraIntrinsicParameter* other);
  friend void swap(CameraIntrinsicParameter& a, CameraIntrinsicParameter& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline CameraIntrinsicParameter* New() const final {
    return CreateMaybeMessage<CameraIntrinsicParameter>(NULL);
  }

  CameraIntrinsicParameter* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<CameraIntrinsicParameter>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const CameraIntrinsicParameter& from);
  void MergeFrom(const CameraIntrinsicParameter& from);
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
  void InternalSwap(CameraIntrinsicParameter* other);
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

  // double fx = 1;
  void clear_fx();
  static const int kFxFieldNumber = 1;
  double fx() const;
  void set_fx(double value);

  // double fy = 2;
  void clear_fy();
  static const int kFyFieldNumber = 2;
  double fy() const;
  void set_fy(double value);

  // double cx = 3;
  void clear_cx();
  static const int kCxFieldNumber = 3;
  double cx() const;
  void set_cx(double value);

  // double cy = 4;
  void clear_cy();
  static const int kCyFieldNumber = 4;
  double cy() const;
  void set_cy(double value);

  // @@protoc_insertion_point(class_scope:apollo.localization.CameraIntrinsicParameter)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_apollo_5fmsgs_2fproto_2flocalization_2fcamera_5fparameter_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class CameraExtrinsicParameter : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.localization.CameraExtrinsicParameter) */ {
 public:
  CameraExtrinsicParameter();
  virtual ~CameraExtrinsicParameter();

  CameraExtrinsicParameter(const CameraExtrinsicParameter& from);

  inline CameraExtrinsicParameter& operator=(const CameraExtrinsicParameter& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  CameraExtrinsicParameter(CameraExtrinsicParameter&& from) noexcept
    : CameraExtrinsicParameter() {
    *this = ::std::move(from);
  }

  inline CameraExtrinsicParameter& operator=(CameraExtrinsicParameter&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const CameraExtrinsicParameter& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CameraExtrinsicParameter* internal_default_instance() {
    return reinterpret_cast<const CameraExtrinsicParameter*>(
               &_CameraExtrinsicParameter_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(CameraExtrinsicParameter* other);
  friend void swap(CameraExtrinsicParameter& a, CameraExtrinsicParameter& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline CameraExtrinsicParameter* New() const final {
    return CreateMaybeMessage<CameraExtrinsicParameter>(NULL);
  }

  CameraExtrinsicParameter* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<CameraExtrinsicParameter>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const CameraExtrinsicParameter& from);
  void MergeFrom(const CameraExtrinsicParameter& from);
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
  void InternalSwap(CameraExtrinsicParameter* other);
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

  // double roll = 1;
  void clear_roll();
  static const int kRollFieldNumber = 1;
  double roll() const;
  void set_roll(double value);

  // double pitch = 2;
  void clear_pitch();
  static const int kPitchFieldNumber = 2;
  double pitch() const;
  void set_pitch(double value);

  // double yaw = 3;
  void clear_yaw();
  static const int kYawFieldNumber = 3;
  double yaw() const;
  void set_yaw(double value);

  // double tx = 4;
  void clear_tx();
  static const int kTxFieldNumber = 4;
  double tx() const;
  void set_tx(double value);

  // double ty = 5;
  void clear_ty();
  static const int kTyFieldNumber = 5;
  double ty() const;
  void set_ty(double value);

  // double tz = 6;
  void clear_tz();
  static const int kTzFieldNumber = 6;
  double tz() const;
  void set_tz(double value);

  // @@protoc_insertion_point(class_scope:apollo.localization.CameraExtrinsicParameter)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double roll_;
  double pitch_;
  double yaw_;
  double tx_;
  double ty_;
  double tz_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_apollo_5fmsgs_2fproto_2flocalization_2fcamera_5fparameter_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class CameraParameter : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.localization.CameraParameter) */ {
 public:
  CameraParameter();
  virtual ~CameraParameter();

  CameraParameter(const CameraParameter& from);

  inline CameraParameter& operator=(const CameraParameter& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  CameraParameter(CameraParameter&& from) noexcept
    : CameraParameter() {
    *this = ::std::move(from);
  }

  inline CameraParameter& operator=(CameraParameter&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const CameraParameter& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CameraParameter* internal_default_instance() {
    return reinterpret_cast<const CameraParameter*>(
               &_CameraParameter_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  void Swap(CameraParameter* other);
  friend void swap(CameraParameter& a, CameraParameter& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline CameraParameter* New() const final {
    return CreateMaybeMessage<CameraParameter>(NULL);
  }

  CameraParameter* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<CameraParameter>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const CameraParameter& from);
  void MergeFrom(const CameraParameter& from);
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
  void InternalSwap(CameraParameter* other);
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

  // .apollo.localization.CameraIntrinsicParameter intrisic_parameter = 1;
  bool has_intrisic_parameter() const;
  void clear_intrisic_parameter();
  static const int kIntrisicParameterFieldNumber = 1;
  private:
  const ::apollo::localization::CameraIntrinsicParameter& _internal_intrisic_parameter() const;
  public:
  const ::apollo::localization::CameraIntrinsicParameter& intrisic_parameter() const;
  ::apollo::localization::CameraIntrinsicParameter* release_intrisic_parameter();
  ::apollo::localization::CameraIntrinsicParameter* mutable_intrisic_parameter();
  void set_allocated_intrisic_parameter(::apollo::localization::CameraIntrinsicParameter* intrisic_parameter);

  // .apollo.localization.CameraExtrinsicParameter extrisic_parameter = 2;
  bool has_extrisic_parameter() const;
  void clear_extrisic_parameter();
  static const int kExtrisicParameterFieldNumber = 2;
  private:
  const ::apollo::localization::CameraExtrinsicParameter& _internal_extrisic_parameter() const;
  public:
  const ::apollo::localization::CameraExtrinsicParameter& extrisic_parameter() const;
  ::apollo::localization::CameraExtrinsicParameter* release_extrisic_parameter();
  ::apollo::localization::CameraExtrinsicParameter* mutable_extrisic_parameter();
  void set_allocated_extrisic_parameter(::apollo::localization::CameraExtrinsicParameter* extrisic_parameter);

  // @@protoc_insertion_point(class_scope:apollo.localization.CameraParameter)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::apollo::localization::CameraIntrinsicParameter* intrisic_parameter_;
  ::apollo::localization::CameraExtrinsicParameter* extrisic_parameter_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_apollo_5fmsgs_2fproto_2flocalization_2fcamera_5fparameter_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// CameraIntrinsicParameter

// double fx = 1;
inline void CameraIntrinsicParameter::clear_fx() {
  fx_ = 0;
}
inline double CameraIntrinsicParameter::fx() const {
  // @@protoc_insertion_point(field_get:apollo.localization.CameraIntrinsicParameter.fx)
  return fx_;
}
inline void CameraIntrinsicParameter::set_fx(double value) {
  
  fx_ = value;
  // @@protoc_insertion_point(field_set:apollo.localization.CameraIntrinsicParameter.fx)
}

// double fy = 2;
inline void CameraIntrinsicParameter::clear_fy() {
  fy_ = 0;
}
inline double CameraIntrinsicParameter::fy() const {
  // @@protoc_insertion_point(field_get:apollo.localization.CameraIntrinsicParameter.fy)
  return fy_;
}
inline void CameraIntrinsicParameter::set_fy(double value) {
  
  fy_ = value;
  // @@protoc_insertion_point(field_set:apollo.localization.CameraIntrinsicParameter.fy)
}

// double cx = 3;
inline void CameraIntrinsicParameter::clear_cx() {
  cx_ = 0;
}
inline double CameraIntrinsicParameter::cx() const {
  // @@protoc_insertion_point(field_get:apollo.localization.CameraIntrinsicParameter.cx)
  return cx_;
}
inline void CameraIntrinsicParameter::set_cx(double value) {
  
  cx_ = value;
  // @@protoc_insertion_point(field_set:apollo.localization.CameraIntrinsicParameter.cx)
}

// double cy = 4;
inline void CameraIntrinsicParameter::clear_cy() {
  cy_ = 0;
}
inline double CameraIntrinsicParameter::cy() const {
  // @@protoc_insertion_point(field_get:apollo.localization.CameraIntrinsicParameter.cy)
  return cy_;
}
inline void CameraIntrinsicParameter::set_cy(double value) {
  
  cy_ = value;
  // @@protoc_insertion_point(field_set:apollo.localization.CameraIntrinsicParameter.cy)
}

// -------------------------------------------------------------------

// CameraExtrinsicParameter

// double roll = 1;
inline void CameraExtrinsicParameter::clear_roll() {
  roll_ = 0;
}
inline double CameraExtrinsicParameter::roll() const {
  // @@protoc_insertion_point(field_get:apollo.localization.CameraExtrinsicParameter.roll)
  return roll_;
}
inline void CameraExtrinsicParameter::set_roll(double value) {
  
  roll_ = value;
  // @@protoc_insertion_point(field_set:apollo.localization.CameraExtrinsicParameter.roll)
}

// double pitch = 2;
inline void CameraExtrinsicParameter::clear_pitch() {
  pitch_ = 0;
}
inline double CameraExtrinsicParameter::pitch() const {
  // @@protoc_insertion_point(field_get:apollo.localization.CameraExtrinsicParameter.pitch)
  return pitch_;
}
inline void CameraExtrinsicParameter::set_pitch(double value) {
  
  pitch_ = value;
  // @@protoc_insertion_point(field_set:apollo.localization.CameraExtrinsicParameter.pitch)
}

// double yaw = 3;
inline void CameraExtrinsicParameter::clear_yaw() {
  yaw_ = 0;
}
inline double CameraExtrinsicParameter::yaw() const {
  // @@protoc_insertion_point(field_get:apollo.localization.CameraExtrinsicParameter.yaw)
  return yaw_;
}
inline void CameraExtrinsicParameter::set_yaw(double value) {
  
  yaw_ = value;
  // @@protoc_insertion_point(field_set:apollo.localization.CameraExtrinsicParameter.yaw)
}

// double tx = 4;
inline void CameraExtrinsicParameter::clear_tx() {
  tx_ = 0;
}
inline double CameraExtrinsicParameter::tx() const {
  // @@protoc_insertion_point(field_get:apollo.localization.CameraExtrinsicParameter.tx)
  return tx_;
}
inline void CameraExtrinsicParameter::set_tx(double value) {
  
  tx_ = value;
  // @@protoc_insertion_point(field_set:apollo.localization.CameraExtrinsicParameter.tx)
}

// double ty = 5;
inline void CameraExtrinsicParameter::clear_ty() {
  ty_ = 0;
}
inline double CameraExtrinsicParameter::ty() const {
  // @@protoc_insertion_point(field_get:apollo.localization.CameraExtrinsicParameter.ty)
  return ty_;
}
inline void CameraExtrinsicParameter::set_ty(double value) {
  
  ty_ = value;
  // @@protoc_insertion_point(field_set:apollo.localization.CameraExtrinsicParameter.ty)
}

// double tz = 6;
inline void CameraExtrinsicParameter::clear_tz() {
  tz_ = 0;
}
inline double CameraExtrinsicParameter::tz() const {
  // @@protoc_insertion_point(field_get:apollo.localization.CameraExtrinsicParameter.tz)
  return tz_;
}
inline void CameraExtrinsicParameter::set_tz(double value) {
  
  tz_ = value;
  // @@protoc_insertion_point(field_set:apollo.localization.CameraExtrinsicParameter.tz)
}

// -------------------------------------------------------------------

// CameraParameter

// .apollo.localization.CameraIntrinsicParameter intrisic_parameter = 1;
inline bool CameraParameter::has_intrisic_parameter() const {
  return this != internal_default_instance() && intrisic_parameter_ != NULL;
}
inline void CameraParameter::clear_intrisic_parameter() {
  if (GetArenaNoVirtual() == NULL && intrisic_parameter_ != NULL) {
    delete intrisic_parameter_;
  }
  intrisic_parameter_ = NULL;
}
inline const ::apollo::localization::CameraIntrinsicParameter& CameraParameter::_internal_intrisic_parameter() const {
  return *intrisic_parameter_;
}
inline const ::apollo::localization::CameraIntrinsicParameter& CameraParameter::intrisic_parameter() const {
  const ::apollo::localization::CameraIntrinsicParameter* p = intrisic_parameter_;
  // @@protoc_insertion_point(field_get:apollo.localization.CameraParameter.intrisic_parameter)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::localization::CameraIntrinsicParameter*>(
      &::apollo::localization::_CameraIntrinsicParameter_default_instance_);
}
inline ::apollo::localization::CameraIntrinsicParameter* CameraParameter::release_intrisic_parameter() {
  // @@protoc_insertion_point(field_release:apollo.localization.CameraParameter.intrisic_parameter)
  
  ::apollo::localization::CameraIntrinsicParameter* temp = intrisic_parameter_;
  intrisic_parameter_ = NULL;
  return temp;
}
inline ::apollo::localization::CameraIntrinsicParameter* CameraParameter::mutable_intrisic_parameter() {
  
  if (intrisic_parameter_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::localization::CameraIntrinsicParameter>(GetArenaNoVirtual());
    intrisic_parameter_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.localization.CameraParameter.intrisic_parameter)
  return intrisic_parameter_;
}
inline void CameraParameter::set_allocated_intrisic_parameter(::apollo::localization::CameraIntrinsicParameter* intrisic_parameter) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete intrisic_parameter_;
  }
  if (intrisic_parameter) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      intrisic_parameter = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, intrisic_parameter, submessage_arena);
    }
    
  } else {
    
  }
  intrisic_parameter_ = intrisic_parameter;
  // @@protoc_insertion_point(field_set_allocated:apollo.localization.CameraParameter.intrisic_parameter)
}

// .apollo.localization.CameraExtrinsicParameter extrisic_parameter = 2;
inline bool CameraParameter::has_extrisic_parameter() const {
  return this != internal_default_instance() && extrisic_parameter_ != NULL;
}
inline void CameraParameter::clear_extrisic_parameter() {
  if (GetArenaNoVirtual() == NULL && extrisic_parameter_ != NULL) {
    delete extrisic_parameter_;
  }
  extrisic_parameter_ = NULL;
}
inline const ::apollo::localization::CameraExtrinsicParameter& CameraParameter::_internal_extrisic_parameter() const {
  return *extrisic_parameter_;
}
inline const ::apollo::localization::CameraExtrinsicParameter& CameraParameter::extrisic_parameter() const {
  const ::apollo::localization::CameraExtrinsicParameter* p = extrisic_parameter_;
  // @@protoc_insertion_point(field_get:apollo.localization.CameraParameter.extrisic_parameter)
  return p != NULL ? *p : *reinterpret_cast<const ::apollo::localization::CameraExtrinsicParameter*>(
      &::apollo::localization::_CameraExtrinsicParameter_default_instance_);
}
inline ::apollo::localization::CameraExtrinsicParameter* CameraParameter::release_extrisic_parameter() {
  // @@protoc_insertion_point(field_release:apollo.localization.CameraParameter.extrisic_parameter)
  
  ::apollo::localization::CameraExtrinsicParameter* temp = extrisic_parameter_;
  extrisic_parameter_ = NULL;
  return temp;
}
inline ::apollo::localization::CameraExtrinsicParameter* CameraParameter::mutable_extrisic_parameter() {
  
  if (extrisic_parameter_ == NULL) {
    auto* p = CreateMaybeMessage<::apollo::localization::CameraExtrinsicParameter>(GetArenaNoVirtual());
    extrisic_parameter_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.localization.CameraParameter.extrisic_parameter)
  return extrisic_parameter_;
}
inline void CameraParameter::set_allocated_extrisic_parameter(::apollo::localization::CameraExtrinsicParameter* extrisic_parameter) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete extrisic_parameter_;
  }
  if (extrisic_parameter) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      extrisic_parameter = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, extrisic_parameter, submessage_arena);
    }
    
  } else {
    
  }
  extrisic_parameter_ = extrisic_parameter;
  // @@protoc_insertion_point(field_set_allocated:apollo.localization.CameraParameter.extrisic_parameter)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace localization
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_apollo_5fmsgs_2fproto_2flocalization_2fcamera_5fparameter_2eproto