// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: apollo_msgs/proto/common/adapter_config.proto

#include "apollo_msgs/proto/common/adapter_config.pb.h"

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

namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_AdapterConfig;
}  // namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto
namespace apollo {
namespace common {
namespace adapter {
class AdapterConfigDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<AdapterConfig>
      _instance;
} _AdapterConfig_default_instance_;
class AdapterManagerConfigDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<AdapterManagerConfig>
      _instance;
} _AdapterManagerConfig_default_instance_;
}  // namespace adapter
}  // namespace common
}  // namespace apollo
namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto {
static void InitDefaultsAdapterConfig() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::common::adapter::_AdapterConfig_default_instance_;
    new (ptr) ::apollo::common::adapter::AdapterConfig();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::common::adapter::AdapterConfig::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_AdapterConfig =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsAdapterConfig}, {}};

static void InitDefaultsAdapterManagerConfig() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::common::adapter::_AdapterManagerConfig_default_instance_;
    new (ptr) ::apollo::common::adapter::AdapterManagerConfig();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::common::adapter::AdapterManagerConfig::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_AdapterManagerConfig =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsAdapterManagerConfig}, {
      &protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::scc_info_AdapterConfig.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_AdapterConfig.base);
  ::google::protobuf::internal::InitSCC(&scc_info_AdapterManagerConfig.base);
}

::google::protobuf::Metadata file_level_metadata[2];
const ::google::protobuf::EnumDescriptor* file_level_enum_descriptors[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::adapter::AdapterConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::adapter::AdapterConfig, type_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::adapter::AdapterConfig, mode_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::adapter::AdapterConfig, message_history_limit_),
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::adapter::AdapterManagerConfig, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::adapter::AdapterManagerConfig, config_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::common::adapter::AdapterManagerConfig, is_ros_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::apollo::common::adapter::AdapterConfig)},
  { 8, -1, sizeof(::apollo::common::adapter::AdapterManagerConfig)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::common::adapter::_AdapterConfig_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::common::adapter::_AdapterManagerConfig_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "apollo_msgs/proto/common/adapter_config.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, file_level_enum_descriptors, NULL);
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
      "\n-apollo_msgs/proto/common/adapter_confi"
      "g.proto\022\025apollo.common.adapter\"\352\003\n\rAdapt"
      "erConfig\022>\n\004type\030\001 \001(\01620.apollo.common.a"
      "dapter.AdapterConfig.MessageType\0227\n\004mode"
      "\030\002 \001(\0162).apollo.common.adapter.AdapterCo"
      "nfig.Mode\022\035\n\025message_history_limit\030\003 \001(\005"
      "\"\210\002\n\013MessageType\022\017\n\013POINT_CLOUD\020\000\022\007\n\003GPS"
      "\020\001\022\007\n\003IMU\020\002\022\013\n\007CHASSIS\020\003\022\020\n\014LOCALIZATION"
      "\020\004\022\027\n\023PLANNING_TRAJECTORY\020\005\022\013\n\007MONITOR\020\006"
      "\022\007\n\003PAD\020\007\022\023\n\017CONTROL_COMMAND\020\010\022\016\n\nPREDIC"
      "TION\020\t\022\030\n\024PERCEPTION_OBSTACLES\020\n\022\033\n\027TRAF"
      "FIC_LIGHT_DETECTION\020\013\022\022\n\016CHASSIS_DETAIL\020"
      "\014\022\014\n\010DECISION\020\r\022\n\n\006CANBUS\020\016\"6\n\004Mode\022\020\n\014R"
      "ECEIVE_ONLY\020\000\022\020\n\014PUBLISH_ONLY\020\001\022\n\n\006DUPLE"
      "X\020\002\"\\\n\024AdapterManagerConfig\0224\n\006config\030\001 "
      "\003(\0132$.apollo.common.adapter.AdapterConfi"
      "g\022\016\n\006is_ros\030\002 \001(\010b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 665);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "apollo_msgs/proto/common/adapter_config.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto
namespace apollo {
namespace common {
namespace adapter {
const ::google::protobuf::EnumDescriptor* AdapterConfig_MessageType_descriptor() {
  protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::file_level_enum_descriptors[0];
}
bool AdapterConfig_MessageType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const AdapterConfig_MessageType AdapterConfig::POINT_CLOUD;
const AdapterConfig_MessageType AdapterConfig::GPS;
const AdapterConfig_MessageType AdapterConfig::IMU;
const AdapterConfig_MessageType AdapterConfig::CHASSIS;
const AdapterConfig_MessageType AdapterConfig::LOCALIZATION;
const AdapterConfig_MessageType AdapterConfig::PLANNING_TRAJECTORY;
const AdapterConfig_MessageType AdapterConfig::MONITOR;
const AdapterConfig_MessageType AdapterConfig::PAD;
const AdapterConfig_MessageType AdapterConfig::CONTROL_COMMAND;
const AdapterConfig_MessageType AdapterConfig::PREDICTION;
const AdapterConfig_MessageType AdapterConfig::PERCEPTION_OBSTACLES;
const AdapterConfig_MessageType AdapterConfig::TRAFFIC_LIGHT_DETECTION;
const AdapterConfig_MessageType AdapterConfig::CHASSIS_DETAIL;
const AdapterConfig_MessageType AdapterConfig::DECISION;
const AdapterConfig_MessageType AdapterConfig::CANBUS;
const AdapterConfig_MessageType AdapterConfig::MessageType_MIN;
const AdapterConfig_MessageType AdapterConfig::MessageType_MAX;
const int AdapterConfig::MessageType_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900
const ::google::protobuf::EnumDescriptor* AdapterConfig_Mode_descriptor() {
  protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::file_level_enum_descriptors[1];
}
bool AdapterConfig_Mode_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const AdapterConfig_Mode AdapterConfig::RECEIVE_ONLY;
const AdapterConfig_Mode AdapterConfig::PUBLISH_ONLY;
const AdapterConfig_Mode AdapterConfig::DUPLEX;
const AdapterConfig_Mode AdapterConfig::Mode_MIN;
const AdapterConfig_Mode AdapterConfig::Mode_MAX;
const int AdapterConfig::Mode_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

// ===================================================================

void AdapterConfig::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int AdapterConfig::kTypeFieldNumber;
const int AdapterConfig::kModeFieldNumber;
const int AdapterConfig::kMessageHistoryLimitFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

AdapterConfig::AdapterConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::scc_info_AdapterConfig.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.common.adapter.AdapterConfig)
}
AdapterConfig::AdapterConfig(const AdapterConfig& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&type_, &from.type_,
    static_cast<size_t>(reinterpret_cast<char*>(&message_history_limit_) -
    reinterpret_cast<char*>(&type_)) + sizeof(message_history_limit_));
  // @@protoc_insertion_point(copy_constructor:apollo.common.adapter.AdapterConfig)
}

void AdapterConfig::SharedCtor() {
  ::memset(&type_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&message_history_limit_) -
      reinterpret_cast<char*>(&type_)) + sizeof(message_history_limit_));
}

AdapterConfig::~AdapterConfig() {
  // @@protoc_insertion_point(destructor:apollo.common.adapter.AdapterConfig)
  SharedDtor();
}

void AdapterConfig::SharedDtor() {
}

void AdapterConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* AdapterConfig::descriptor() {
  ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const AdapterConfig& AdapterConfig::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::scc_info_AdapterConfig.base);
  return *internal_default_instance();
}


void AdapterConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.adapter.AdapterConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&type_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&message_history_limit_) -
      reinterpret_cast<char*>(&type_)) + sizeof(message_history_limit_));
  _internal_metadata_.Clear();
}

bool AdapterConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.common.adapter.AdapterConfig)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // .apollo.common.adapter.AdapterConfig.MessageType type = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          set_type(static_cast< ::apollo::common::adapter::AdapterConfig_MessageType >(value));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .apollo.common.adapter.AdapterConfig.Mode mode = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          set_mode(static_cast< ::apollo::common::adapter::AdapterConfig_Mode >(value));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // int32 message_history_limit = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &message_history_limit_)));
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
  // @@protoc_insertion_point(parse_success:apollo.common.adapter.AdapterConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.common.adapter.AdapterConfig)
  return false;
#undef DO_
}

void AdapterConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.common.adapter.AdapterConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .apollo.common.adapter.AdapterConfig.MessageType type = 1;
  if (this->type() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      1, this->type(), output);
  }

  // .apollo.common.adapter.AdapterConfig.Mode mode = 2;
  if (this->mode() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      2, this->mode(), output);
  }

  // int32 message_history_limit = 3;
  if (this->message_history_limit() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(3, this->message_history_limit(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.common.adapter.AdapterConfig)
}

::google::protobuf::uint8* AdapterConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.adapter.AdapterConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .apollo.common.adapter.AdapterConfig.MessageType type = 1;
  if (this->type() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      1, this->type(), target);
  }

  // .apollo.common.adapter.AdapterConfig.Mode mode = 2;
  if (this->mode() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      2, this->mode(), target);
  }

  // int32 message_history_limit = 3;
  if (this->message_history_limit() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(3, this->message_history_limit(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.adapter.AdapterConfig)
  return target;
}

size_t AdapterConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.adapter.AdapterConfig)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // .apollo.common.adapter.AdapterConfig.MessageType type = 1;
  if (this->type() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::EnumSize(this->type());
  }

  // .apollo.common.adapter.AdapterConfig.Mode mode = 2;
  if (this->mode() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::EnumSize(this->mode());
  }

  // int32 message_history_limit = 3;
  if (this->message_history_limit() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->message_history_limit());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void AdapterConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.common.adapter.AdapterConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const AdapterConfig* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const AdapterConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.common.adapter.AdapterConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.common.adapter.AdapterConfig)
    MergeFrom(*source);
  }
}

void AdapterConfig::MergeFrom(const AdapterConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.adapter.AdapterConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.type() != 0) {
    set_type(from.type());
  }
  if (from.mode() != 0) {
    set_mode(from.mode());
  }
  if (from.message_history_limit() != 0) {
    set_message_history_limit(from.message_history_limit());
  }
}

void AdapterConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.common.adapter.AdapterConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void AdapterConfig::CopyFrom(const AdapterConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.adapter.AdapterConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool AdapterConfig::IsInitialized() const {
  return true;
}

void AdapterConfig::Swap(AdapterConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void AdapterConfig::InternalSwap(AdapterConfig* other) {
  using std::swap;
  swap(type_, other->type_);
  swap(mode_, other->mode_);
  swap(message_history_limit_, other->message_history_limit_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata AdapterConfig::GetMetadata() const {
  protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void AdapterManagerConfig::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int AdapterManagerConfig::kConfigFieldNumber;
const int AdapterManagerConfig::kIsRosFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

AdapterManagerConfig::AdapterManagerConfig()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::scc_info_AdapterManagerConfig.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.common.adapter.AdapterManagerConfig)
}
AdapterManagerConfig::AdapterManagerConfig(const AdapterManagerConfig& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      config_(from.config_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  is_ros_ = from.is_ros_;
  // @@protoc_insertion_point(copy_constructor:apollo.common.adapter.AdapterManagerConfig)
}

void AdapterManagerConfig::SharedCtor() {
  is_ros_ = false;
}

AdapterManagerConfig::~AdapterManagerConfig() {
  // @@protoc_insertion_point(destructor:apollo.common.adapter.AdapterManagerConfig)
  SharedDtor();
}

void AdapterManagerConfig::SharedDtor() {
}

void AdapterManagerConfig::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* AdapterManagerConfig::descriptor() {
  ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const AdapterManagerConfig& AdapterManagerConfig::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::scc_info_AdapterManagerConfig.base);
  return *internal_default_instance();
}


void AdapterManagerConfig::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.adapter.AdapterManagerConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  config_.Clear();
  is_ros_ = false;
  _internal_metadata_.Clear();
}

bool AdapterManagerConfig::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.common.adapter.AdapterManagerConfig)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .apollo.common.adapter.AdapterConfig config = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_config()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // bool is_ros = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &is_ros_)));
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
  // @@protoc_insertion_point(parse_success:apollo.common.adapter.AdapterManagerConfig)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.common.adapter.AdapterManagerConfig)
  return false;
#undef DO_
}

void AdapterManagerConfig::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.common.adapter.AdapterManagerConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .apollo.common.adapter.AdapterConfig config = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->config_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1,
      this->config(static_cast<int>(i)),
      output);
  }

  // bool is_ros = 2;
  if (this->is_ros() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(2, this->is_ros(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.common.adapter.AdapterManagerConfig)
}

::google::protobuf::uint8* AdapterManagerConfig::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.adapter.AdapterManagerConfig)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .apollo.common.adapter.AdapterConfig config = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->config_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->config(static_cast<int>(i)), deterministic, target);
  }

  // bool is_ros = 2;
  if (this->is_ros() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(2, this->is_ros(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.adapter.AdapterManagerConfig)
  return target;
}

size_t AdapterManagerConfig::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.adapter.AdapterManagerConfig)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // repeated .apollo.common.adapter.AdapterConfig config = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->config_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->config(static_cast<int>(i)));
    }
  }

  // bool is_ros = 2;
  if (this->is_ros() != 0) {
    total_size += 1 + 1;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void AdapterManagerConfig::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.common.adapter.AdapterManagerConfig)
  GOOGLE_DCHECK_NE(&from, this);
  const AdapterManagerConfig* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const AdapterManagerConfig>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.common.adapter.AdapterManagerConfig)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.common.adapter.AdapterManagerConfig)
    MergeFrom(*source);
  }
}

void AdapterManagerConfig::MergeFrom(const AdapterManagerConfig& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.adapter.AdapterManagerConfig)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  config_.MergeFrom(from.config_);
  if (from.is_ros() != 0) {
    set_is_ros(from.is_ros());
  }
}

void AdapterManagerConfig::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.common.adapter.AdapterManagerConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void AdapterManagerConfig::CopyFrom(const AdapterManagerConfig& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.adapter.AdapterManagerConfig)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool AdapterManagerConfig::IsInitialized() const {
  return true;
}

void AdapterManagerConfig::Swap(AdapterManagerConfig* other) {
  if (other == this) return;
  InternalSwap(other);
}
void AdapterManagerConfig::InternalSwap(AdapterManagerConfig* other) {
  using std::swap;
  CastToBase(&config_)->InternalSwap(CastToBase(&other->config_));
  swap(is_ros_, other->is_ros_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata AdapterManagerConfig::GetMetadata() const {
  protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fadapter_5fconfig_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace adapter
}  // namespace common
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::common::adapter::AdapterConfig* Arena::CreateMaybeMessage< ::apollo::common::adapter::AdapterConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::common::adapter::AdapterConfig >(arena);
}
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::common::adapter::AdapterManagerConfig* Arena::CreateMaybeMessage< ::apollo::common::adapter::AdapterManagerConfig >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::common::adapter::AdapterManagerConfig >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)