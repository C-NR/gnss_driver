// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: header.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "header.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace gnss_driver {
namespace pb {

namespace {

const ::google::protobuf::Descriptor* Header_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Header_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_header_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_header_2eproto() {
  protobuf_AddDesc_header_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "header.proto");
  GOOGLE_CHECK(file != NULL);
  Header_descriptor_ = file->message_type(0);
  static const int Header_offsets_[8] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, timestamp_sec_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, module_name_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, sequence_num_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, lidar_timestamp_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, camera_timestamp_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, radar_timestamp_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, version_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, status_),
  };
  Header_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      Header_descriptor_,
      Header::internal_default_instance(),
      Header_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, _has_bits_),
      -1,
      -1,
      sizeof(Header),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Header, _internal_metadata_));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_header_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      Header_descriptor_, Header::internal_default_instance());
}

}  // namespace

void protobuf_ShutdownFile_header_2eproto() {
  Header_default_instance_.Shutdown();
  delete Header_reflection_;
}

void protobuf_InitDefaults_header_2eproto_impl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::gnss_driver::pb::protobuf_InitDefaults_error_5fcode_2eproto();
  ::google::protobuf::internal::GetEmptyString();
  Header_default_instance_.DefaultConstruct();
  Header_default_instance_.get_mutable()->InitAsDefaultInstance();
}

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_InitDefaults_header_2eproto_once_);
void protobuf_InitDefaults_header_2eproto() {
  ::google::protobuf::GoogleOnceInit(&protobuf_InitDefaults_header_2eproto_once_,
                 &protobuf_InitDefaults_header_2eproto_impl);
}
void protobuf_AddDesc_header_2eproto_impl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  protobuf_InitDefaults_header_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\014header.proto\022\016gnss_driver.pb\032\020error_co"
    "de.proto\"\324\001\n\006Header\022\025\n\rtimestamp_sec\030\001 \001"
    "(\001\022\023\n\013module_name\030\002 \001(\t\022\024\n\014sequence_num\030"
    "\003 \001(\r\022\027\n\017lidar_timestamp\030\004 \001(\004\022\030\n\020camera"
    "_timestamp\030\005 \001(\004\022\027\n\017radar_timestamp\030\006 \001("
    "\004\022\022\n\007version\030\007 \001(\r:\0011\022(\n\006status\030\010 \001(\0132\030."
    "gnss_driver.pb.StatusPb", 263);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "header.proto", &protobuf_RegisterTypes);
  ::gnss_driver::pb::protobuf_AddDesc_error_5fcode_2eproto();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_header_2eproto);
}

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AddDesc_header_2eproto_once_);
void protobuf_AddDesc_header_2eproto() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AddDesc_header_2eproto_once_,
                 &protobuf_AddDesc_header_2eproto_impl);
}
// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_header_2eproto {
  StaticDescriptorInitializer_header_2eproto() {
    protobuf_AddDesc_header_2eproto();
  }
} static_descriptor_initializer_header_2eproto_;

namespace {

static void MergeFromFail(int line) GOOGLE_ATTRIBUTE_COLD GOOGLE_ATTRIBUTE_NORETURN;
static void MergeFromFail(int line) {
  ::google::protobuf::internal::MergeFromFail(__FILE__, line);
}

}  // namespace


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Header::kTimestampSecFieldNumber;
const int Header::kModuleNameFieldNumber;
const int Header::kSequenceNumFieldNumber;
const int Header::kLidarTimestampFieldNumber;
const int Header::kCameraTimestampFieldNumber;
const int Header::kRadarTimestampFieldNumber;
const int Header::kVersionFieldNumber;
const int Header::kStatusFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Header::Header()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (this != internal_default_instance()) protobuf_InitDefaults_header_2eproto();
  SharedCtor();
  // @@protoc_insertion_point(constructor:gnss_driver.pb.Header)
}

void Header::InitAsDefaultInstance() {
  status_ = const_cast< ::gnss_driver::pb::StatusPb*>(
      ::gnss_driver::pb::StatusPb::internal_default_instance());
}

Header::Header(const Header& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  UnsafeMergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:gnss_driver.pb.Header)
}

void Header::SharedCtor() {
  _cached_size_ = 0;
  module_name_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  status_ = NULL;
  ::memset(&timestamp_sec_, 0, reinterpret_cast<char*>(&sequence_num_) -
    reinterpret_cast<char*>(&timestamp_sec_) + sizeof(sequence_num_));
  version_ = 1u;
}

Header::~Header() {
  // @@protoc_insertion_point(destructor:gnss_driver.pb.Header)
  SharedDtor();
}

void Header::SharedDtor() {
  module_name_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (this != &Header_default_instance_.get()) {
    delete status_;
  }
}

void Header::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Header::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Header_descriptor_;
}

const Header& Header::default_instance() {
  protobuf_InitDefaults_header_2eproto();
  return *internal_default_instance();
}

::google::protobuf::internal::ExplicitlyConstructed<Header> Header_default_instance_;

Header* Header::New(::google::protobuf::Arena* arena) const {
  Header* n = new Header;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Header::Clear() {
// @@protoc_insertion_point(message_clear_start:gnss_driver.pb.Header)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(Header, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<Header*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&(first), 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  if (_has_bits_[0 / 32] & 255u) {
    ZR_(timestamp_sec_, sequence_num_);
    if (has_module_name()) {
      module_name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    }
    version_ = 1u;
    if (has_status()) {
      if (status_ != NULL) status_->::gnss_driver::pb::StatusPb::Clear();
    }
  }

#undef ZR_HELPER_
#undef ZR_

  _has_bits_.Clear();
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool Header::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gnss_driver.pb.Header)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double timestamp_sec = 1;
      case 1: {
        if (tag == 9) {
          set_has_timestamp_sec();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &timestamp_sec_)));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(18)) goto parse_module_name;
        break;
      }

      // optional string module_name = 2;
      case 2: {
        if (tag == 18) {
         parse_module_name:
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_module_name()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->module_name().data(), this->module_name().length(),
            ::google::protobuf::internal::WireFormat::PARSE,
            "gnss_driver.pb.Header.module_name");
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(24)) goto parse_sequence_num;
        break;
      }

      // optional uint32 sequence_num = 3;
      case 3: {
        if (tag == 24) {
         parse_sequence_num:
          set_has_sequence_num();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &sequence_num_)));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(32)) goto parse_lidar_timestamp;
        break;
      }

      // optional uint64 lidar_timestamp = 4;
      case 4: {
        if (tag == 32) {
         parse_lidar_timestamp:
          set_has_lidar_timestamp();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint64, ::google::protobuf::internal::WireFormatLite::TYPE_UINT64>(
                 input, &lidar_timestamp_)));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(40)) goto parse_camera_timestamp;
        break;
      }

      // optional uint64 camera_timestamp = 5;
      case 5: {
        if (tag == 40) {
         parse_camera_timestamp:
          set_has_camera_timestamp();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint64, ::google::protobuf::internal::WireFormatLite::TYPE_UINT64>(
                 input, &camera_timestamp_)));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(48)) goto parse_radar_timestamp;
        break;
      }

      // optional uint64 radar_timestamp = 6;
      case 6: {
        if (tag == 48) {
         parse_radar_timestamp:
          set_has_radar_timestamp();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint64, ::google::protobuf::internal::WireFormatLite::TYPE_UINT64>(
                 input, &radar_timestamp_)));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(56)) goto parse_version;
        break;
      }

      // optional uint32 version = 7 [default = 1];
      case 7: {
        if (tag == 56) {
         parse_version:
          set_has_version();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &version_)));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(66)) goto parse_status;
        break;
      }

      // optional .gnss_driver.pb.StatusPb status = 8;
      case 8: {
        if (tag == 66) {
         parse_status:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_status()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:gnss_driver.pb.Header)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gnss_driver.pb.Header)
  return false;
#undef DO_
}

void Header::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gnss_driver.pb.Header)
  // optional double timestamp_sec = 1;
  if (has_timestamp_sec()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->timestamp_sec(), output);
  }

  // optional string module_name = 2;
  if (has_module_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->module_name().data(), this->module_name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gnss_driver.pb.Header.module_name");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->module_name(), output);
  }

  // optional uint32 sequence_num = 3;
  if (has_sequence_num()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(3, this->sequence_num(), output);
  }

  // optional uint64 lidar_timestamp = 4;
  if (has_lidar_timestamp()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt64(4, this->lidar_timestamp(), output);
  }

  // optional uint64 camera_timestamp = 5;
  if (has_camera_timestamp()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt64(5, this->camera_timestamp(), output);
  }

  // optional uint64 radar_timestamp = 6;
  if (has_radar_timestamp()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt64(6, this->radar_timestamp(), output);
  }

  // optional uint32 version = 7 [default = 1];
  if (has_version()) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(7, this->version(), output);
  }

  // optional .gnss_driver.pb.StatusPb status = 8;
  if (has_status()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      8, *this->status_, output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gnss_driver.pb.Header)
}

::google::protobuf::uint8* Header::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:gnss_driver.pb.Header)
  // optional double timestamp_sec = 1;
  if (has_timestamp_sec()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->timestamp_sec(), target);
  }

  // optional string module_name = 2;
  if (has_module_name()) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->module_name().data(), this->module_name().length(),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gnss_driver.pb.Header.module_name");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        2, this->module_name(), target);
  }

  // optional uint32 sequence_num = 3;
  if (has_sequence_num()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(3, this->sequence_num(), target);
  }

  // optional uint64 lidar_timestamp = 4;
  if (has_lidar_timestamp()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt64ToArray(4, this->lidar_timestamp(), target);
  }

  // optional uint64 camera_timestamp = 5;
  if (has_camera_timestamp()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt64ToArray(5, this->camera_timestamp(), target);
  }

  // optional uint64 radar_timestamp = 6;
  if (has_radar_timestamp()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt64ToArray(6, this->radar_timestamp(), target);
  }

  // optional uint32 version = 7 [default = 1];
  if (has_version()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(7, this->version(), target);
  }

  // optional .gnss_driver.pb.StatusPb status = 8;
  if (has_status()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        8, *this->status_, false, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gnss_driver.pb.Header)
  return target;
}

size_t Header::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:gnss_driver.pb.Header)
  size_t total_size = 0;

  if (_has_bits_[0 / 32] & 255u) {
    // optional double timestamp_sec = 1;
    if (has_timestamp_sec()) {
      total_size += 1 + 8;
    }

    // optional string module_name = 2;
    if (has_module_name()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->module_name());
    }

    // optional uint32 sequence_num = 3;
    if (has_sequence_num()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->sequence_num());
    }

    // optional uint64 lidar_timestamp = 4;
    if (has_lidar_timestamp()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt64Size(
          this->lidar_timestamp());
    }

    // optional uint64 camera_timestamp = 5;
    if (has_camera_timestamp()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt64Size(
          this->camera_timestamp());
    }

    // optional uint64 radar_timestamp = 6;
    if (has_radar_timestamp()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt64Size(
          this->radar_timestamp());
    }

    // optional uint32 version = 7 [default = 1];
    if (has_version()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->version());
    }

    // optional .gnss_driver.pb.StatusPb status = 8;
    if (has_status()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          *this->status_);
    }

  }
  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Header::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:gnss_driver.pb.Header)
  if (GOOGLE_PREDICT_FALSE(&from == this)) MergeFromFail(__LINE__);
  const Header* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Header>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:gnss_driver.pb.Header)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:gnss_driver.pb.Header)
    UnsafeMergeFrom(*source);
  }
}

void Header::MergeFrom(const Header& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:gnss_driver.pb.Header)
  if (GOOGLE_PREDICT_TRUE(&from != this)) {
    UnsafeMergeFrom(from);
  } else {
    MergeFromFail(__LINE__);
  }
}

void Header::UnsafeMergeFrom(const Header& from) {
  GOOGLE_DCHECK(&from != this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_timestamp_sec()) {
      set_timestamp_sec(from.timestamp_sec());
    }
    if (from.has_module_name()) {
      set_has_module_name();
      module_name_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.module_name_);
    }
    if (from.has_sequence_num()) {
      set_sequence_num(from.sequence_num());
    }
    if (from.has_lidar_timestamp()) {
      set_lidar_timestamp(from.lidar_timestamp());
    }
    if (from.has_camera_timestamp()) {
      set_camera_timestamp(from.camera_timestamp());
    }
    if (from.has_radar_timestamp()) {
      set_radar_timestamp(from.radar_timestamp());
    }
    if (from.has_version()) {
      set_version(from.version());
    }
    if (from.has_status()) {
      mutable_status()->::gnss_driver::pb::StatusPb::MergeFrom(from.status());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::UnknownFieldSet::MergeToInternalMetdata(
      from.unknown_fields(), &_internal_metadata_);
  }
}

void Header::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:gnss_driver.pb.Header)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Header::CopyFrom(const Header& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:gnss_driver.pb.Header)
  if (&from == this) return;
  Clear();
  UnsafeMergeFrom(from);
}

bool Header::IsInitialized() const {

  return true;
}

void Header::Swap(Header* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Header::InternalSwap(Header* other) {
  std::swap(timestamp_sec_, other->timestamp_sec_);
  module_name_.Swap(&other->module_name_);
  std::swap(sequence_num_, other->sequence_num_);
  std::swap(lidar_timestamp_, other->lidar_timestamp_);
  std::swap(camera_timestamp_, other->camera_timestamp_);
  std::swap(radar_timestamp_, other->radar_timestamp_);
  std::swap(version_, other->version_);
  std::swap(status_, other->status_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Header::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Header_descriptor_;
  metadata.reflection = Header_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// Header

// optional double timestamp_sec = 1;
bool Header::has_timestamp_sec() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void Header::set_has_timestamp_sec() {
  _has_bits_[0] |= 0x00000001u;
}
void Header::clear_has_timestamp_sec() {
  _has_bits_[0] &= ~0x00000001u;
}
void Header::clear_timestamp_sec() {
  timestamp_sec_ = 0;
  clear_has_timestamp_sec();
}
double Header::timestamp_sec() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Header.timestamp_sec)
  return timestamp_sec_;
}
void Header::set_timestamp_sec(double value) {
  set_has_timestamp_sec();
  timestamp_sec_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.Header.timestamp_sec)
}

// optional string module_name = 2;
bool Header::has_module_name() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void Header::set_has_module_name() {
  _has_bits_[0] |= 0x00000002u;
}
void Header::clear_has_module_name() {
  _has_bits_[0] &= ~0x00000002u;
}
void Header::clear_module_name() {
  module_name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_module_name();
}
const ::std::string& Header::module_name() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Header.module_name)
  return module_name_.GetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
void Header::set_module_name(const ::std::string& value) {
  set_has_module_name();
  module_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gnss_driver.pb.Header.module_name)
}
void Header::set_module_name(const char* value) {
  set_has_module_name();
  module_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gnss_driver.pb.Header.module_name)
}
void Header::set_module_name(const char* value, size_t size) {
  set_has_module_name();
  module_name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gnss_driver.pb.Header.module_name)
}
::std::string* Header::mutable_module_name() {
  set_has_module_name();
  // @@protoc_insertion_point(field_mutable:gnss_driver.pb.Header.module_name)
  return module_name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
::std::string* Header::release_module_name() {
  // @@protoc_insertion_point(field_release:gnss_driver.pb.Header.module_name)
  clear_has_module_name();
  return module_name_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
void Header::set_allocated_module_name(::std::string* module_name) {
  if (module_name != NULL) {
    set_has_module_name();
  } else {
    clear_has_module_name();
  }
  module_name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), module_name);
  // @@protoc_insertion_point(field_set_allocated:gnss_driver.pb.Header.module_name)
}

// optional uint32 sequence_num = 3;
bool Header::has_sequence_num() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void Header::set_has_sequence_num() {
  _has_bits_[0] |= 0x00000004u;
}
void Header::clear_has_sequence_num() {
  _has_bits_[0] &= ~0x00000004u;
}
void Header::clear_sequence_num() {
  sequence_num_ = 0u;
  clear_has_sequence_num();
}
::google::protobuf::uint32 Header::sequence_num() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Header.sequence_num)
  return sequence_num_;
}
void Header::set_sequence_num(::google::protobuf::uint32 value) {
  set_has_sequence_num();
  sequence_num_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.Header.sequence_num)
}

// optional uint64 lidar_timestamp = 4;
bool Header::has_lidar_timestamp() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
void Header::set_has_lidar_timestamp() {
  _has_bits_[0] |= 0x00000008u;
}
void Header::clear_has_lidar_timestamp() {
  _has_bits_[0] &= ~0x00000008u;
}
void Header::clear_lidar_timestamp() {
  lidar_timestamp_ = GOOGLE_ULONGLONG(0);
  clear_has_lidar_timestamp();
}
::google::protobuf::uint64 Header::lidar_timestamp() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Header.lidar_timestamp)
  return lidar_timestamp_;
}
void Header::set_lidar_timestamp(::google::protobuf::uint64 value) {
  set_has_lidar_timestamp();
  lidar_timestamp_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.Header.lidar_timestamp)
}

// optional uint64 camera_timestamp = 5;
bool Header::has_camera_timestamp() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
void Header::set_has_camera_timestamp() {
  _has_bits_[0] |= 0x00000010u;
}
void Header::clear_has_camera_timestamp() {
  _has_bits_[0] &= ~0x00000010u;
}
void Header::clear_camera_timestamp() {
  camera_timestamp_ = GOOGLE_ULONGLONG(0);
  clear_has_camera_timestamp();
}
::google::protobuf::uint64 Header::camera_timestamp() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Header.camera_timestamp)
  return camera_timestamp_;
}
void Header::set_camera_timestamp(::google::protobuf::uint64 value) {
  set_has_camera_timestamp();
  camera_timestamp_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.Header.camera_timestamp)
}

// optional uint64 radar_timestamp = 6;
bool Header::has_radar_timestamp() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
void Header::set_has_radar_timestamp() {
  _has_bits_[0] |= 0x00000020u;
}
void Header::clear_has_radar_timestamp() {
  _has_bits_[0] &= ~0x00000020u;
}
void Header::clear_radar_timestamp() {
  radar_timestamp_ = GOOGLE_ULONGLONG(0);
  clear_has_radar_timestamp();
}
::google::protobuf::uint64 Header::radar_timestamp() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Header.radar_timestamp)
  return radar_timestamp_;
}
void Header::set_radar_timestamp(::google::protobuf::uint64 value) {
  set_has_radar_timestamp();
  radar_timestamp_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.Header.radar_timestamp)
}

// optional uint32 version = 7 [default = 1];
bool Header::has_version() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
void Header::set_has_version() {
  _has_bits_[0] |= 0x00000040u;
}
void Header::clear_has_version() {
  _has_bits_[0] &= ~0x00000040u;
}
void Header::clear_version() {
  version_ = 1u;
  clear_has_version();
}
::google::protobuf::uint32 Header::version() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Header.version)
  return version_;
}
void Header::set_version(::google::protobuf::uint32 value) {
  set_has_version();
  version_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.Header.version)
}

// optional .gnss_driver.pb.StatusPb status = 8;
bool Header::has_status() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
void Header::set_has_status() {
  _has_bits_[0] |= 0x00000080u;
}
void Header::clear_has_status() {
  _has_bits_[0] &= ~0x00000080u;
}
void Header::clear_status() {
  if (status_ != NULL) status_->::gnss_driver::pb::StatusPb::Clear();
  clear_has_status();
}
const ::gnss_driver::pb::StatusPb& Header::status() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Header.status)
  return status_ != NULL ? *status_
                         : *::gnss_driver::pb::StatusPb::internal_default_instance();
}
::gnss_driver::pb::StatusPb* Header::mutable_status() {
  set_has_status();
  if (status_ == NULL) {
    status_ = new ::gnss_driver::pb::StatusPb;
  }
  // @@protoc_insertion_point(field_mutable:gnss_driver.pb.Header.status)
  return status_;
}
::gnss_driver::pb::StatusPb* Header::release_status() {
  // @@protoc_insertion_point(field_release:gnss_driver.pb.Header.status)
  clear_has_status();
  ::gnss_driver::pb::StatusPb* temp = status_;
  status_ = NULL;
  return temp;
}
void Header::set_allocated_status(::gnss_driver::pb::StatusPb* status) {
  delete status_;
  status_ = status;
  if (status) {
    set_has_status();
  } else {
    clear_has_status();
  }
  // @@protoc_insertion_point(field_set_allocated:gnss_driver.pb.Header.status)
}

inline const Header* Header::internal_default_instance() {
  return &Header_default_instance_.get();
}
#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace pb
}  // namespace gnss_driver

// @@protoc_insertion_point(global_scope)
