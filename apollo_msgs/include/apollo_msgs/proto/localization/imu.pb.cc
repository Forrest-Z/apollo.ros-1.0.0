// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: apollo_msgs/proto/localization/imu.proto

#include "apollo_msgs/proto/localization/imu.pb.h"

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

namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto ::google::protobuf::internal::SCCInfo<1> scc_info_Header;
}  // namespace protobuf_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto
namespace protobuf_apollo_5fmsgs_2fproto_2flocalization_2fpose_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_apollo_5fmsgs_2fproto_2flocalization_2fpose_2eproto ::google::protobuf::internal::SCCInfo<3> scc_info_Pose;
}  // namespace protobuf_apollo_5fmsgs_2fproto_2flocalization_2fpose_2eproto
namespace apollo {
namespace localization {
class ImuDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Imu>
      _instance;
} _Imu_default_instance_;
}  // namespace localization
}  // namespace apollo
namespace protobuf_apollo_5fmsgs_2fproto_2flocalization_2fimu_2eproto {
static void InitDefaultsImu() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::localization::_Imu_default_instance_;
    new (ptr) ::apollo::localization::Imu();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::localization::Imu::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<2> scc_info_Imu =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsImu}, {
      &protobuf_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto::scc_info_Header.base,
      &protobuf_apollo_5fmsgs_2fproto_2flocalization_2fpose_2eproto::scc_info_Pose.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_Imu.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::localization::Imu, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::localization::Imu, header_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::apollo::localization::Imu, imu_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::apollo::localization::Imu)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::apollo::localization::_Imu_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "apollo_msgs/proto/localization/imu.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n(apollo_msgs/proto/localization/imu.pro"
      "to\022\023apollo.localization\032%apollo_msgs/pro"
      "to/common/header.proto\032)apollo_msgs/prot"
      "o/localization/pose.proto\"T\n\003Imu\022%\n\006head"
      "er\030\001 \001(\0132\025.apollo.common.Header\022&\n\003imu\030\003"
      " \001(\0132\031.apollo.localization.Poseb\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 239);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "apollo_msgs/proto/localization/imu.proto", &protobuf_RegisterTypes);
  ::protobuf_apollo_5fmsgs_2fproto_2fcommon_2fheader_2eproto::AddDescriptors();
  ::protobuf_apollo_5fmsgs_2fproto_2flocalization_2fpose_2eproto::AddDescriptors();
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
}  // namespace protobuf_apollo_5fmsgs_2fproto_2flocalization_2fimu_2eproto
namespace apollo {
namespace localization {

// ===================================================================

void Imu::InitAsDefaultInstance() {
  ::apollo::localization::_Imu_default_instance_._instance.get_mutable()->header_ = const_cast< ::apollo::common::Header*>(
      ::apollo::common::Header::internal_default_instance());
  ::apollo::localization::_Imu_default_instance_._instance.get_mutable()->imu_ = const_cast< ::apollo::localization::Pose*>(
      ::apollo::localization::Pose::internal_default_instance());
}
void Imu::clear_header() {
  if (GetArenaNoVirtual() == NULL && header_ != NULL) {
    delete header_;
  }
  header_ = NULL;
}
void Imu::clear_imu() {
  if (GetArenaNoVirtual() == NULL && imu_ != NULL) {
    delete imu_;
  }
  imu_ = NULL;
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Imu::kHeaderFieldNumber;
const int Imu::kImuFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Imu::Imu()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_apollo_5fmsgs_2fproto_2flocalization_2fimu_2eproto::scc_info_Imu.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.localization.Imu)
}
Imu::Imu(const Imu& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = NULL;
  }
  if (from.has_imu()) {
    imu_ = new ::apollo::localization::Pose(*from.imu_);
  } else {
    imu_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.localization.Imu)
}

void Imu::SharedCtor() {
  ::memset(&header_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&imu_) -
      reinterpret_cast<char*>(&header_)) + sizeof(imu_));
}

Imu::~Imu() {
  // @@protoc_insertion_point(destructor:apollo.localization.Imu)
  SharedDtor();
}

void Imu::SharedDtor() {
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete imu_;
}

void Imu::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Imu::descriptor() {
  ::protobuf_apollo_5fmsgs_2fproto_2flocalization_2fimu_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2flocalization_2fimu_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Imu& Imu::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_apollo_5fmsgs_2fproto_2flocalization_2fimu_2eproto::scc_info_Imu.base);
  return *internal_default_instance();
}


void Imu::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.localization.Imu)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaNoVirtual() == NULL && header_ != NULL) {
    delete header_;
  }
  header_ = NULL;
  if (GetArenaNoVirtual() == NULL && imu_ != NULL) {
    delete imu_;
  }
  imu_ = NULL;
  _internal_metadata_.Clear();
}

bool Imu::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.localization.Imu)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // .apollo.common.Header header = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_header()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .apollo.localization.Pose imu = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_imu()));
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
  // @@protoc_insertion_point(parse_success:apollo.localization.Imu)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.localization.Imu)
  return false;
#undef DO_
}

void Imu::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.localization.Imu)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .apollo.common.Header header = 1;
  if (this->has_header()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_header(), output);
  }

  // .apollo.localization.Pose imu = 3;
  if (this->has_imu()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->_internal_imu(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.localization.Imu)
}

::google::protobuf::uint8* Imu::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:apollo.localization.Imu)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .apollo.common.Header header = 1;
  if (this->has_header()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_header(), deterministic, target);
  }

  // .apollo.localization.Pose imu = 3;
  if (this->has_imu()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        3, this->_internal_imu(), deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.localization.Imu)
  return target;
}

size_t Imu::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.localization.Imu)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // .apollo.common.Header header = 1;
  if (this->has_header()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *header_);
  }

  // .apollo.localization.Pose imu = 3;
  if (this->has_imu()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *imu_);
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Imu::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.localization.Imu)
  GOOGLE_DCHECK_NE(&from, this);
  const Imu* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Imu>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.localization.Imu)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.localization.Imu)
    MergeFrom(*source);
  }
}

void Imu::MergeFrom(const Imu& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.localization.Imu)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_header()) {
    mutable_header()->::apollo::common::Header::MergeFrom(from.header());
  }
  if (from.has_imu()) {
    mutable_imu()->::apollo::localization::Pose::MergeFrom(from.imu());
  }
}

void Imu::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.localization.Imu)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Imu::CopyFrom(const Imu& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.localization.Imu)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Imu::IsInitialized() const {
  return true;
}

void Imu::Swap(Imu* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Imu::InternalSwap(Imu* other) {
  using std::swap;
  swap(header_, other->header_);
  swap(imu_, other->imu_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Imu::GetMetadata() const {
  protobuf_apollo_5fmsgs_2fproto_2flocalization_2fimu_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_apollo_5fmsgs_2fproto_2flocalization_2fimu_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace localization
}  // namespace apollo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::apollo::localization::Imu* Arena::CreateMaybeMessage< ::apollo::localization::Imu >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::localization::Imu >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
