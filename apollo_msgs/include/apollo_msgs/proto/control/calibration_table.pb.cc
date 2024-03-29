// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: apollo_msgs/proto/control/calibration_table.proto

#include "apollo_msgs/proto/control/calibration_table.pb.h"

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

namespace protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_ControlCalibrationInfo;
}  // namespace protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto
namespace apollo {
namespace control {
namespace calibrationtable {
class ControlCalibrationTableDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<ControlCalibrationTable>
      _instance;
} _ControlCalibrationTable_default_instance_;
class ControlCalibrationInfoDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<ControlCalibrationInfo>
      _instance;
} _ControlCalibrationInfo_default_instance_;
}  // namespace calibrationtable
}  // namespace control
}  // namespace apollo
namespace protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto {
static void InitDefaultsControlCalibrationTable() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::control::calibrationtable::_ControlCalibrationTable_default_instance_;
    new (ptr) ::apollo::control::calibrationtable::ControlCalibrationTable();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::control::calibrationtable::ControlCalibrationTable::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<1> scc_info_ControlCalibrationTable =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsControlCalibrationTable}, {
      &protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::scc_info_ControlCalibrationInfo.base,}};

static void InitDefaultsControlCalibrationInfo() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::control::calibrationtable::_ControlCalibrationInfo_default_instance_;
    new (ptr) ::apollo::control::calibrationtable::ControlCalibrationInfo();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::control::calibrationtable::ControlCalibrationInfo::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_ControlCalibrationInfo =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsControlCalibrationInfo}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_ControlCalibrationTable.base);
  ::google::protobuf::internal::InitSCC(&scc_info_ControlCalibrationInfo.base);
}

::google::protobuf::Metadata file_level_metadata[2];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationTable, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationTable, calibration_),
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationInfo, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationInfo, speed_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationInfo, acceleration_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::control::calibrationtable::ControlCalibrationInfo, command_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::apollo::control::calibrationtable::ControlCalibrationTable)},
  { 6, -1, sizeof(::apollo::control::calibrationtable::ControlCalibrationInfo)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::control::calibrationtable::_ControlCalibrationTable_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::control::calibrationtable::_ControlCalibrationInfo_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "apollo_msgs/proto/control/calibration_table.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n1apollo_msgs/proto/control/calibration_"
      "table.proto\022\037apollo.control.calibrationt"
      "able\"g\n\027ControlCalibrationTable\022L\n\013calib"
      "ration\030\001 \003(\01327.apollo.control.calibratio"
      "ntable.ControlCalibrationInfo\"N\n\026Control"
      "CalibrationInfo\022\r\n\005speed\030\001 \001(\001\022\024\n\014accele"
      "ration\030\002 \001(\001\022\017\n\007command\030\003 \001(\001b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 277);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "apollo_msgs/proto/control/calibration_table.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto
namespace apollo {
namespace control {
namespace calibrationtable {

// ===================================================================

void ControlCalibrationTable::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ControlCalibrationTable::kCalibrationFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ControlCalibrationTable::ControlCalibrationTable()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::scc_info_ControlCalibrationTable.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.control.calibrationtable.ControlCalibrationTable)
}
ControlCalibrationTable::ControlCalibrationTable(const ControlCalibrationTable& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      calibration_(from.calibration_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:apollo.control.calibrationtable.ControlCalibrationTable)
}

void ControlCalibrationTable::SharedCtor() {
}

ControlCalibrationTable::~ControlCalibrationTable() {
  // @@protoc_insertion_point(destructor:apollo.control.calibrationtable.ControlCalibrationTable)
  SharedDtor();
}

void ControlCalibrationTable::SharedDtor() {
}

void ControlCalibrationTable::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* ControlCalibrationTable::descriptor() {
  ::protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ControlCalibrationTable& ControlCalibrationTable::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::scc_info_ControlCalibrationTable.base);
  return *internal_default_instance();
}


void ControlCalibrationTable::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.control.calibrationtable.ControlCalibrationTable)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  calibration_.Clear();
  _internal_metadata_.Clear();
}

bool ControlCalibrationTable::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.control.calibrationtable.ControlCalibrationTable)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .apollo.control.calibrationtable.ControlCalibrationInfo calibration = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_calibration()));
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
  // @@protoc_insertion_point(parse_success:apollo.control.calibrationtable.ControlCalibrationTable)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.control.calibrationtable.ControlCalibrationTable)
  return false;
#undef DO_
}

void ControlCalibrationTable::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.control.calibrationtable.ControlCalibrationTable)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .apollo.control.calibrationtable.ControlCalibrationInfo calibration = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->calibration_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1,
      this->calibration(static_cast<int>(i)),
      output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.control.calibrationtable.ControlCalibrationTable)
}

::google::protobuf::uint8* ControlCalibrationTable::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.control.calibrationtable.ControlCalibrationTable)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .apollo.control.calibrationtable.ControlCalibrationInfo calibration = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->calibration_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->calibration(static_cast<int>(i)), deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.control.calibrationtable.ControlCalibrationTable)
  return target;
}

size_t ControlCalibrationTable::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.control.calibrationtable.ControlCalibrationTable)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // repeated .apollo.control.calibrationtable.ControlCalibrationInfo calibration = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->calibration_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->calibration(static_cast<int>(i)));
    }
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ControlCalibrationTable::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.control.calibrationtable.ControlCalibrationTable)
  GOOGLE_DCHECK_NE(&from, this);
  const ControlCalibrationTable* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ControlCalibrationTable>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.control.calibrationtable.ControlCalibrationTable)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.control.calibrationtable.ControlCalibrationTable)
    MergeFrom(*source);
  }
}

void ControlCalibrationTable::MergeFrom(const ControlCalibrationTable& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.control.calibrationtable.ControlCalibrationTable)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  calibration_.MergeFrom(from.calibration_);
}

void ControlCalibrationTable::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.control.calibrationtable.ControlCalibrationTable)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ControlCalibrationTable::CopyFrom(const ControlCalibrationTable& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.control.calibrationtable.ControlCalibrationTable)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ControlCalibrationTable::IsInitialized() const {
  return true;
}

void ControlCalibrationTable::Swap(ControlCalibrationTable* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ControlCalibrationTable::InternalSwap(ControlCalibrationTable* other) {
  using std::swap;
  CastToBase(&calibration_)->InternalSwap(CastToBase(&other->calibration_));
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata ControlCalibrationTable::GetMetadata() const {
  protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::file_level_metadata[kIndexInFileMessages];
}


// ===================================================================

void ControlCalibrationInfo::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ControlCalibrationInfo::kSpeedFieldNumber;
const int ControlCalibrationInfo::kAccelerationFieldNumber;
const int ControlCalibrationInfo::kCommandFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ControlCalibrationInfo::ControlCalibrationInfo()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::scc_info_ControlCalibrationInfo.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.control.calibrationtable.ControlCalibrationInfo)
}
ControlCalibrationInfo::ControlCalibrationInfo(const ControlCalibrationInfo& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&speed_, &from.speed_,
    static_cast<size_t>(reinterpret_cast<char*>(&command_) -
    reinterpret_cast<char*>(&speed_)) + sizeof(command_));
  // @@protoc_insertion_point(copy_constructor:apollo.control.calibrationtable.ControlCalibrationInfo)
}

void ControlCalibrationInfo::SharedCtor() {
  ::memset(&speed_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&command_) -
      reinterpret_cast<char*>(&speed_)) + sizeof(command_));
}

ControlCalibrationInfo::~ControlCalibrationInfo() {
  // @@protoc_insertion_point(destructor:apollo.control.calibrationtable.ControlCalibrationInfo)
  SharedDtor();
}

void ControlCalibrationInfo::SharedDtor() {
}

void ControlCalibrationInfo::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* ControlCalibrationInfo::descriptor() {
  ::protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ControlCalibrationInfo& ControlCalibrationInfo::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::scc_info_ControlCalibrationInfo.base);
  return *internal_default_instance();
}


void ControlCalibrationInfo::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&speed_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&command_) -
      reinterpret_cast<char*>(&speed_)) + sizeof(command_));
  _internal_metadata_.Clear();
}

bool ControlCalibrationInfo::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // double speed = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &speed_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double acceleration = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &acceleration_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double command = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &command_)));
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
  // @@protoc_insertion_point(parse_success:apollo.control.calibrationtable.ControlCalibrationInfo)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.control.calibrationtable.ControlCalibrationInfo)
  return false;
#undef DO_
}

void ControlCalibrationInfo::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double speed = 1;
  if (this->speed() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->speed(), output);
  }

  // double acceleration = 2;
  if (this->acceleration() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->acceleration(), output);
  }

  // double command = 3;
  if (this->command() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->command(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.control.calibrationtable.ControlCalibrationInfo)
}

::google::protobuf::uint8* ControlCalibrationInfo::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double speed = 1;
  if (this->speed() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->speed(), target);
  }

  // double acceleration = 2;
  if (this->acceleration() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->acceleration(), target);
  }

  // double command = 3;
  if (this->command() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->command(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.control.calibrationtable.ControlCalibrationInfo)
  return target;
}

size_t ControlCalibrationInfo::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // double speed = 1;
  if (this->speed() != 0) {
    total_size += 1 + 8;
  }

  // double acceleration = 2;
  if (this->acceleration() != 0) {
    total_size += 1 + 8;
  }

  // double command = 3;
  if (this->command() != 0) {
    total_size += 1 + 8;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ControlCalibrationInfo::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  GOOGLE_DCHECK_NE(&from, this);
  const ControlCalibrationInfo* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ControlCalibrationInfo>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.control.calibrationtable.ControlCalibrationInfo)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.control.calibrationtable.ControlCalibrationInfo)
    MergeFrom(*source);
  }
}

void ControlCalibrationInfo::MergeFrom(const ControlCalibrationInfo& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.speed() != 0) {
    set_speed(from.speed());
  }
  if (from.acceleration() != 0) {
    set_acceleration(from.acceleration());
  }
  if (from.command() != 0) {
    set_command(from.command());
  }
}

void ControlCalibrationInfo::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ControlCalibrationInfo::CopyFrom(const ControlCalibrationInfo& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.control.calibrationtable.ControlCalibrationInfo)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ControlCalibrationInfo::IsInitialized() const {
  return true;
}

void ControlCalibrationInfo::Swap(ControlCalibrationInfo* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ControlCalibrationInfo::InternalSwap(ControlCalibrationInfo* other) {
  using std::swap;
  swap(speed_, other->speed_);
  swap(acceleration_, other->acceleration_);
  swap(command_, other->command_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata ControlCalibrationInfo::GetMetadata() const {
  protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2fcontrol_2fcalibration_5ftable_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace calibrationtable
}  // namespace control
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::control::calibrationtable::ControlCalibrationTable* Arena::CreateMaybeMessage< ::apollo::control::calibrationtable::ControlCalibrationTable >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::control::calibrationtable::ControlCalibrationTable >(arena);
}
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::control::calibrationtable::ControlCalibrationInfo* Arena::CreateMaybeMessage< ::apollo::control::calibrationtable::ControlCalibrationInfo >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::control::calibrationtable::ControlCalibrationInfo >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
