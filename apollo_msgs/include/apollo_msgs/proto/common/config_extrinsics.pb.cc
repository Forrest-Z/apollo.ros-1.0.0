// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: apollo_msgs/proto/common/config_extrinsics.proto

#include "apollo_msgs/proto/common/config_extrinsics.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto ::google::protobuf::internal::SCCInfo<2> scc_info_Transform;
}  // namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto
namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgeometry_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgeometry_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Point3D;
extern PROTOBUF_INTERNAL_EXPORT_protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgeometry_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Quaternion;
}  // namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgeometry_2eproto
namespace apollo {
namespace common {
namespace config {
class TransformDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Transform>
      _instance;
} _Transform_default_instance_;
class ExtrinsicsDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Extrinsics>
      _instance;
} _Extrinsics_default_instance_;
}  // namespace config
}  // namespace common
}  // namespace apollo
namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto {
static void InitDefaultsTransform() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::common::config::_Transform_default_instance_;
    new (ptr) ::apollo::common::config::Transform();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::common::config::Transform::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<2> scc_info_Transform =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsTransform}, {
      &protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgeometry_2eproto::scc_info_Point3D.base,
      &protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgeometry_2eproto::scc_info_Quaternion.base,}};

static void InitDefaultsExtrinsics() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::common::config::_Extrinsics_default_instance_;
    new (ptr) ::apollo::common::config::Extrinsics();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::common::config::Extrinsics::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_Extrinsics =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsExtrinsics}, {
      &protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::scc_info_Transform.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_Transform.base);
  ::google::protobuf::internal::InitSCC(&scc_info_Extrinsics.base);
}

::google::protobuf::Metadata file_level_metadata[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::config::Transform, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::config::Transform, source_frame_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::config::Transform, target_frame_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::config::Transform, translation_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::config::Transform, rotation_),
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::config::Extrinsics, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::config::Extrinsics, tansforms_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::apollo::common::config::Transform)},
  { 9, -1, sizeof(::apollo::common::config::Extrinsics)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::common::config::_Transform_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::common::config::_Extrinsics_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "apollo_msgs/proto/common/config_extrinsics.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 2);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n0apollo_msgs/proto/common/config_extrin"
      "sics.proto\022\024apollo.common.config\032\'apollo"
      "_msgs/proto/common/geometry.proto\"\221\001\n\tTr"
      "ansform\022\024\n\014source_frame\030\001 \001(\014\022\024\n\014target_"
      "frame\030\002 \001(\014\022+\n\013translation\030\003 \001(\0132\026.apoll"
      "o.common.Point3D\022+\n\010rotation\030\004 \001(\0132\031.apo"
      "llo.common.Quaternion\"@\n\nExtrinsics\0222\n\tt"
      "ansforms\030\001 \003(\0132\037.apollo.common.config.Tr"
      "ansformb\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 335);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "apollo_msgs/proto/common/config_extrinsics.proto", &protobuf_RegisterTypes);
  ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fgeometry_2eproto::AddDescriptors();
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto
namespace apollo {
namespace common {
namespace config {

// ===================================================================

void Transform::InitAsDefaultInstance() {
  ::apollo::common::config::_Transform_default_instance_._instance.get_mutable()->translation_ = const_cast< ::apollo::common::Point3D*>(
      ::apollo::common::Point3D::internal_default_instance());
  ::apollo::common::config::_Transform_default_instance_._instance.get_mutable()->rotation_ = const_cast< ::apollo::common::Quaternion*>(
      ::apollo::common::Quaternion::internal_default_instance());
}
void Transform::clear_translation() {
  if (GetArenaNoVirtual() == NULL && translation_ != NULL) {
    delete translation_;
  }
  translation_ = NULL;
}
void Transform::clear_rotation() {
  if (GetArenaNoVirtual() == NULL && rotation_ != NULL) {
    delete rotation_;
  }
  rotation_ = NULL;
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Transform::kSourceFrameFieldNumber;
const int Transform::kTargetFrameFieldNumber;
const int Transform::kTranslationFieldNumber;
const int Transform::kRotationFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Transform::Transform()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::scc_info_Transform.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.common.config.Transform)
}
Transform::Transform(const Transform& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  source_frame_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.source_frame().size() > 0) {
    source_frame_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.source_frame_);
  }
  target_frame_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.target_frame().size() > 0) {
    target_frame_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.target_frame_);
  }
  if (from.has_translation()) {
    translation_ = new ::apollo::common::Point3D(*from.translation_);
  } else {
    translation_ = NULL;
  }
  if (from.has_rotation()) {
    rotation_ = new ::apollo::common::Quaternion(*from.rotation_);
  } else {
    rotation_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.common.config.Transform)
}

void Transform::SharedCtor() {
  source_frame_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  target_frame_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  ::memset(&translation_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&rotation_) -
      reinterpret_cast<char*>(&translation_)) + sizeof(rotation_));
}

Transform::~Transform() {
  // @@protoc_insertion_point(destructor:apollo.common.config.Transform)
  SharedDtor();
}

void Transform::SharedDtor() {
  source_frame_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  target_frame_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete translation_;
  if (this != internal_default_instance()) delete rotation_;
}

void Transform::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Transform::descriptor() {
  ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Transform& Transform::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::scc_info_Transform.base);
  return *internal_default_instance();
}


void Transform::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.config.Transform)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  source_frame_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  target_frame_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (GetArenaNoVirtual() == NULL && translation_ != NULL) {
    delete translation_;
  }
  translation_ = NULL;
  if (GetArenaNoVirtual() == NULL && rotation_ != NULL) {
    delete rotation_;
  }
  rotation_ = NULL;
  _internal_metadata_.Clear();
}

bool Transform::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.common.config.Transform)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // bytes source_frame = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadBytes(
                input, this->mutable_source_frame()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // bytes target_frame = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadBytes(
                input, this->mutable_target_frame()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .apollo.common.Point3D translation = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_translation()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .apollo.common.Quaternion rotation = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_rotation()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:apollo.common.config.Transform)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.common.config.Transform)
  return false;
#undef DO_
}

void Transform::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.common.config.Transform)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // bytes source_frame = 1;
  if (this->source_frame().size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteBytesMaybeAliased(
      1, this->source_frame(), output);
  }

  // bytes target_frame = 2;
  if (this->target_frame().size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteBytesMaybeAliased(
      2, this->target_frame(), output);
  }

  // .apollo.common.Point3D translation = 3;
  if (this->has_translation()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->_internal_translation(), output);
  }

  // .apollo.common.Quaternion rotation = 4;
  if (this->has_rotation()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      4, this->_internal_rotation(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.common.config.Transform)
}

::google::protobuf::uint8* Transform::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.config.Transform)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // bytes source_frame = 1;
  if (this->source_frame().size() > 0) {
    target =
      ::google::protobuf::internal::WireFormatLite::WriteBytesToArray(
        1, this->source_frame(), target);
  }

  // bytes target_frame = 2;
  if (this->target_frame().size() > 0) {
    target =
      ::google::protobuf::internal::WireFormatLite::WriteBytesToArray(
        2, this->target_frame(), target);
  }

  // .apollo.common.Point3D translation = 3;
  if (this->has_translation()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        3, this->_internal_translation(), deterministic, target);
  }

  // .apollo.common.Quaternion rotation = 4;
  if (this->has_rotation()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        4, this->_internal_rotation(), deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.config.Transform)
  return target;
}

size_t Transform::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.config.Transform)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // bytes source_frame = 1;
  if (this->source_frame().size() > 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::BytesSize(
        this->source_frame());
  }

  // bytes target_frame = 2;
  if (this->target_frame().size() > 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::BytesSize(
        this->target_frame());
  }

  // .apollo.common.Point3D translation = 3;
  if (this->has_translation()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *translation_);
  }

  // .apollo.common.Quaternion rotation = 4;
  if (this->has_rotation()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *rotation_);
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Transform::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.common.config.Transform)
  GOOGLE_DCHECK_NE(&from, this);
  const Transform* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Transform>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.common.config.Transform)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.common.config.Transform)
    MergeFrom(*source);
  }
}

void Transform::MergeFrom(const Transform& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.config.Transform)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.source_frame().size() > 0) {

    source_frame_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.source_frame_);
  }
  if (from.target_frame().size() > 0) {

    target_frame_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.target_frame_);
  }
  if (from.has_translation()) {
    mutable_translation()->::apollo::common::Point3D::MergeFrom(from.translation());
  }
  if (from.has_rotation()) {
    mutable_rotation()->::apollo::common::Quaternion::MergeFrom(from.rotation());
  }
}

void Transform::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.common.config.Transform)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Transform::CopyFrom(const Transform& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.config.Transform)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Transform::IsInitialized() const {
  return true;
}

void Transform::Swap(Transform* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Transform::InternalSwap(Transform* other) {
  using std::swap;
  source_frame_.Swap(&other->source_frame_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  target_frame_.Swap(&other->target_frame_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(translation_, other->translation_);
  swap(rotation_, other->rotation_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Transform::GetMetadata() const {
  protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void Extrinsics::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Extrinsics::kTansformsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Extrinsics::Extrinsics()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::scc_info_Extrinsics.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.common.config.Extrinsics)
}
Extrinsics::Extrinsics(const Extrinsics& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      tansforms_(from.tansforms_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:apollo.common.config.Extrinsics)
}

void Extrinsics::SharedCtor() {
}

Extrinsics::~Extrinsics() {
  // @@protoc_insertion_point(destructor:apollo.common.config.Extrinsics)
  SharedDtor();
}

void Extrinsics::SharedDtor() {
}

void Extrinsics::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Extrinsics::descriptor() {
  ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Extrinsics& Extrinsics::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::scc_info_Extrinsics.base);
  return *internal_default_instance();
}


void Extrinsics::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.config.Extrinsics)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  tansforms_.Clear();
  _internal_metadata_.Clear();
}

bool Extrinsics::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.common.config.Extrinsics)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .apollo.common.config.Transform tansforms = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_tansforms()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:apollo.common.config.Extrinsics)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.common.config.Extrinsics)
  return false;
#undef DO_
}

void Extrinsics::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.common.config.Extrinsics)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .apollo.common.config.Transform tansforms = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->tansforms_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1,
      this->tansforms(static_cast<int>(i)),
      output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.common.config.Extrinsics)
}

::google::protobuf::uint8* Extrinsics::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.config.Extrinsics)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .apollo.common.config.Transform tansforms = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->tansforms_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->tansforms(static_cast<int>(i)), deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.config.Extrinsics)
  return target;
}

size_t Extrinsics::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.config.Extrinsics)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // repeated .apollo.common.config.Transform tansforms = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->tansforms_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->tansforms(static_cast<int>(i)));
    }
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Extrinsics::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.common.config.Extrinsics)
  GOOGLE_DCHECK_NE(&from, this);
  const Extrinsics* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Extrinsics>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.common.config.Extrinsics)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.common.config.Extrinsics)
    MergeFrom(*source);
  }
}

void Extrinsics::MergeFrom(const Extrinsics& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.config.Extrinsics)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  tansforms_.MergeFrom(from.tansforms_);
}

void Extrinsics::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.common.config.Extrinsics)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Extrinsics::CopyFrom(const Extrinsics& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.config.Extrinsics)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Extrinsics::IsInitialized() const {
  return true;
}

void Extrinsics::Swap(Extrinsics* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Extrinsics::InternalSwap(Extrinsics* other) {
  using std::swap;
  CastToBase(&tansforms_)->InternalSwap(CastToBase(&other->tansforms_));
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Extrinsics::GetMetadata() const {
  protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fconfig_5fextrinsics_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace config
}  // namespace common
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::common::config::Transform* Arena::CreateMaybeMessage< ::apollo::common::config::Transform >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::common::config::Transform >(arena);
}
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::common::config::Extrinsics* Arena::CreateMaybeMessage< ::apollo::common::config::Extrinsics >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::common::config::Extrinsics >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
