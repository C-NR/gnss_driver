// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: imu.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "imu.pb.h"

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

const ::google::protobuf::Descriptor* Imu_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Imu_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_imu_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_imu_2eproto() {
  protobuf_AddDesc_imu_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "imu.proto");
  GOOGLE_CHECK(file != NULL);
  Imu_descriptor_ = file->message_type(0);
  static const int Imu_offsets_[5] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, header_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, measurement_time_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, measurement_span_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, linear_acceleration_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, angular_velocity_),
  };
  Imu_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      Imu_descriptor_,
      Imu::internal_default_instance(),
      Imu_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, _has_bits_),
      -1,
      -1,
      sizeof(Imu),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Imu, _internal_metadata_));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_imu_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      Imu_descriptor_, Imu::internal_default_instance());
}

}  // namespace

void protobuf_ShutdownFile_imu_2eproto() {
  Imu_default_instance_.Shutdown();
  delete Imu_reflection_;
}

void protobuf_InitDefaults_imu_2eproto_impl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::gnss_driver::pb::protobuf_InitDefaults_header_2eproto();
  ::gnss_driver::pb::protobuf_InitDefaults_geometry_2eproto();
  Imu_default_instance_.DefaultConstruct();
  Imu_default_instance_.get_mutable()->InitAsDefaultInstance();
}

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_InitDefaults_imu_2eproto_once_);
void protobuf_InitDefaults_imu_2eproto() {
  ::google::protobuf::GoogleOnceInit(&protobuf_InitDefaults_imu_2eproto_once_,
                 &protobuf_InitDefaults_imu_2eproto_impl);
}
void protobuf_AddDesc_imu_2eproto_impl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  protobuf_InitDefaults_imu_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\timu.proto\022\016gnss_driver.pb\032\014header.prot"
    "o\032\016geometry.proto\"\315\001\n\003Imu\022&\n\006header\030\001 \001("
    "\0132\026.gnss_driver.pb.Header\022\030\n\020measurement"
    "_time\030\002 \001(\001\022\033\n\020measurement_span\030\003 \001(\002:\0010"
    "\0224\n\023linear_acceleration\030\004 \001(\0132\027.gnss_dri"
    "ver.pb.Point3D\0221\n\020angular_velocity\030\005 \001(\013"
    "2\027.gnss_driver.pb.Point3D", 265);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "imu.proto", &protobuf_RegisterTypes);
  ::gnss_driver::pb::protobuf_AddDesc_header_2eproto();
  ::gnss_driver::pb::protobuf_AddDesc_geometry_2eproto();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_imu_2eproto);
}

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AddDesc_imu_2eproto_once_);
void protobuf_AddDesc_imu_2eproto() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AddDesc_imu_2eproto_once_,
                 &protobuf_AddDesc_imu_2eproto_impl);
}
// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_imu_2eproto {
  StaticDescriptorInitializer_imu_2eproto() {
    protobuf_AddDesc_imu_2eproto();
  }
} static_descriptor_initializer_imu_2eproto_;

namespace {

static void MergeFromFail(int line) GOOGLE_ATTRIBUTE_COLD GOOGLE_ATTRIBUTE_NORETURN;
static void MergeFromFail(int line) {
  ::google::protobuf::internal::MergeFromFail(__FILE__, line);
}

}  // namespace


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Imu::kHeaderFieldNumber;
const int Imu::kMeasurementTimeFieldNumber;
const int Imu::kMeasurementSpanFieldNumber;
const int Imu::kLinearAccelerationFieldNumber;
const int Imu::kAngularVelocityFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Imu::Imu()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (this != internal_default_instance()) protobuf_InitDefaults_imu_2eproto();
  SharedCtor();
  // @@protoc_insertion_point(constructor:gnss_driver.pb.Imu)
}

void Imu::InitAsDefaultInstance() {
  header_ = const_cast< ::gnss_driver::pb::Header*>(
      ::gnss_driver::pb::Header::internal_default_instance());
  linear_acceleration_ = const_cast< ::gnss_driver::pb::Point3D*>(
      ::gnss_driver::pb::Point3D::internal_default_instance());
  angular_velocity_ = const_cast< ::gnss_driver::pb::Point3D*>(
      ::gnss_driver::pb::Point3D::internal_default_instance());
}

Imu::Imu(const Imu& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  UnsafeMergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:gnss_driver.pb.Imu)
}

void Imu::SharedCtor() {
  _cached_size_ = 0;
  header_ = NULL;
  linear_acceleration_ = NULL;
  angular_velocity_ = NULL;
  ::memset(&measurement_time_, 0, reinterpret_cast<char*>(&measurement_span_) -
    reinterpret_cast<char*>(&measurement_time_) + sizeof(measurement_span_));
}

Imu::~Imu() {
  // @@protoc_insertion_point(destructor:gnss_driver.pb.Imu)
  SharedDtor();
}

void Imu::SharedDtor() {
  if (this != &Imu_default_instance_.get()) {
    delete header_;
    delete linear_acceleration_;
    delete angular_velocity_;
  }
}

void Imu::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Imu::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Imu_descriptor_;
}

const Imu& Imu::default_instance() {
  protobuf_InitDefaults_imu_2eproto();
  return *internal_default_instance();
}

::google::protobuf::internal::ExplicitlyConstructed<Imu> Imu_default_instance_;

Imu* Imu::New(::google::protobuf::Arena* arena) const {
  Imu* n = new Imu;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Imu::Clear() {
// @@protoc_insertion_point(message_clear_start:gnss_driver.pb.Imu)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(Imu, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<Imu*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&(first), 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  if (_has_bits_[0 / 32] & 31u) {
    ZR_(measurement_time_, measurement_span_);
    if (has_header()) {
      if (header_ != NULL) header_->::gnss_driver::pb::Header::Clear();
    }
    if (has_linear_acceleration()) {
      if (linear_acceleration_ != NULL) linear_acceleration_->::gnss_driver::pb::Point3D::Clear();
    }
    if (has_angular_velocity()) {
      if (angular_velocity_ != NULL) angular_velocity_->::gnss_driver::pb::Point3D::Clear();
    }
  }

#undef ZR_HELPER_
#undef ZR_

  _has_bits_.Clear();
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool Imu::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gnss_driver.pb.Imu)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .gnss_driver.pb.Header header = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_header()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_measurement_time;
        break;
      }

      // optional double measurement_time = 2;
      case 2: {
        if (tag == 17) {
         parse_measurement_time:
          set_has_measurement_time();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &measurement_time_)));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(29)) goto parse_measurement_span;
        break;
      }

      // optional float measurement_span = 3 [default = 0];
      case 3: {
        if (tag == 29) {
         parse_measurement_span:
          set_has_measurement_span();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &measurement_span_)));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(34)) goto parse_linear_acceleration;
        break;
      }

      // optional .gnss_driver.pb.Point3D linear_acceleration = 4;
      case 4: {
        if (tag == 34) {
         parse_linear_acceleration:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_linear_acceleration()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(42)) goto parse_angular_velocity;
        break;
      }

      // optional .gnss_driver.pb.Point3D angular_velocity = 5;
      case 5: {
        if (tag == 42) {
         parse_angular_velocity:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_angular_velocity()));
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
  // @@protoc_insertion_point(parse_success:gnss_driver.pb.Imu)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gnss_driver.pb.Imu)
  return false;
#undef DO_
}

void Imu::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gnss_driver.pb.Imu)
  // optional .gnss_driver.pb.Header header = 1;
  if (has_header()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, *this->header_, output);
  }

  // optional double measurement_time = 2;
  if (has_measurement_time()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->measurement_time(), output);
  }

  // optional float measurement_span = 3 [default = 0];
  if (has_measurement_span()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->measurement_span(), output);
  }

  // optional .gnss_driver.pb.Point3D linear_acceleration = 4;
  if (has_linear_acceleration()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      4, *this->linear_acceleration_, output);
  }

  // optional .gnss_driver.pb.Point3D angular_velocity = 5;
  if (has_angular_velocity()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      5, *this->angular_velocity_, output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gnss_driver.pb.Imu)
}

::google::protobuf::uint8* Imu::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:gnss_driver.pb.Imu)
  // optional .gnss_driver.pb.Header header = 1;
  if (has_header()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        1, *this->header_, false, target);
  }

  // optional double measurement_time = 2;
  if (has_measurement_time()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->measurement_time(), target);
  }

  // optional float measurement_span = 3 [default = 0];
  if (has_measurement_span()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->measurement_span(), target);
  }

  // optional .gnss_driver.pb.Point3D linear_acceleration = 4;
  if (has_linear_acceleration()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        4, *this->linear_acceleration_, false, target);
  }

  // optional .gnss_driver.pb.Point3D angular_velocity = 5;
  if (has_angular_velocity()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        5, *this->angular_velocity_, false, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gnss_driver.pb.Imu)
  return target;
}

size_t Imu::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:gnss_driver.pb.Imu)
  size_t total_size = 0;

  if (_has_bits_[0 / 32] & 31u) {
    // optional .gnss_driver.pb.Header header = 1;
    if (has_header()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          *this->header_);
    }

    // optional double measurement_time = 2;
    if (has_measurement_time()) {
      total_size += 1 + 8;
    }

    // optional float measurement_span = 3 [default = 0];
    if (has_measurement_span()) {
      total_size += 1 + 4;
    }

    // optional .gnss_driver.pb.Point3D linear_acceleration = 4;
    if (has_linear_acceleration()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          *this->linear_acceleration_);
    }

    // optional .gnss_driver.pb.Point3D angular_velocity = 5;
    if (has_angular_velocity()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          *this->angular_velocity_);
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

void Imu::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:gnss_driver.pb.Imu)
  if (GOOGLE_PREDICT_FALSE(&from == this)) MergeFromFail(__LINE__);
  const Imu* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Imu>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:gnss_driver.pb.Imu)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:gnss_driver.pb.Imu)
    UnsafeMergeFrom(*source);
  }
}

void Imu::MergeFrom(const Imu& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:gnss_driver.pb.Imu)
  if (GOOGLE_PREDICT_TRUE(&from != this)) {
    UnsafeMergeFrom(from);
  } else {
    MergeFromFail(__LINE__);
  }
}

void Imu::UnsafeMergeFrom(const Imu& from) {
  GOOGLE_DCHECK(&from != this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_header()) {
      mutable_header()->::gnss_driver::pb::Header::MergeFrom(from.header());
    }
    if (from.has_measurement_time()) {
      set_measurement_time(from.measurement_time());
    }
    if (from.has_measurement_span()) {
      set_measurement_span(from.measurement_span());
    }
    if (from.has_linear_acceleration()) {
      mutable_linear_acceleration()->::gnss_driver::pb::Point3D::MergeFrom(from.linear_acceleration());
    }
    if (from.has_angular_velocity()) {
      mutable_angular_velocity()->::gnss_driver::pb::Point3D::MergeFrom(from.angular_velocity());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::UnknownFieldSet::MergeToInternalMetdata(
      from.unknown_fields(), &_internal_metadata_);
  }
}

void Imu::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:gnss_driver.pb.Imu)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Imu::CopyFrom(const Imu& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:gnss_driver.pb.Imu)
  if (&from == this) return;
  Clear();
  UnsafeMergeFrom(from);
}

bool Imu::IsInitialized() const {

  return true;
}

void Imu::Swap(Imu* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Imu::InternalSwap(Imu* other) {
  std::swap(header_, other->header_);
  std::swap(measurement_time_, other->measurement_time_);
  std::swap(measurement_span_, other->measurement_span_);
  std::swap(linear_acceleration_, other->linear_acceleration_);
  std::swap(angular_velocity_, other->angular_velocity_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Imu::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Imu_descriptor_;
  metadata.reflection = Imu_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// Imu

// optional .gnss_driver.pb.Header header = 1;
bool Imu::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void Imu::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
void Imu::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
void Imu::clear_header() {
  if (header_ != NULL) header_->::gnss_driver::pb::Header::Clear();
  clear_has_header();
}
const ::gnss_driver::pb::Header& Imu::header() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Imu.header)
  return header_ != NULL ? *header_
                         : *::gnss_driver::pb::Header::internal_default_instance();
}
::gnss_driver::pb::Header* Imu::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    header_ = new ::gnss_driver::pb::Header;
  }
  // @@protoc_insertion_point(field_mutable:gnss_driver.pb.Imu.header)
  return header_;
}
::gnss_driver::pb::Header* Imu::release_header() {
  // @@protoc_insertion_point(field_release:gnss_driver.pb.Imu.header)
  clear_has_header();
  ::gnss_driver::pb::Header* temp = header_;
  header_ = NULL;
  return temp;
}
void Imu::set_allocated_header(::gnss_driver::pb::Header* header) {
  delete header_;
  header_ = header;
  if (header) {
    set_has_header();
  } else {
    clear_has_header();
  }
  // @@protoc_insertion_point(field_set_allocated:gnss_driver.pb.Imu.header)
}

// optional double measurement_time = 2;
bool Imu::has_measurement_time() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void Imu::set_has_measurement_time() {
  _has_bits_[0] |= 0x00000002u;
}
void Imu::clear_has_measurement_time() {
  _has_bits_[0] &= ~0x00000002u;
}
void Imu::clear_measurement_time() {
  measurement_time_ = 0;
  clear_has_measurement_time();
}
double Imu::measurement_time() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Imu.measurement_time)
  return measurement_time_;
}
void Imu::set_measurement_time(double value) {
  set_has_measurement_time();
  measurement_time_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.Imu.measurement_time)
}

// optional float measurement_span = 3 [default = 0];
bool Imu::has_measurement_span() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void Imu::set_has_measurement_span() {
  _has_bits_[0] |= 0x00000004u;
}
void Imu::clear_has_measurement_span() {
  _has_bits_[0] &= ~0x00000004u;
}
void Imu::clear_measurement_span() {
  measurement_span_ = 0;
  clear_has_measurement_span();
}
float Imu::measurement_span() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Imu.measurement_span)
  return measurement_span_;
}
void Imu::set_measurement_span(float value) {
  set_has_measurement_span();
  measurement_span_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.Imu.measurement_span)
}

// optional .gnss_driver.pb.Point3D linear_acceleration = 4;
bool Imu::has_linear_acceleration() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
void Imu::set_has_linear_acceleration() {
  _has_bits_[0] |= 0x00000008u;
}
void Imu::clear_has_linear_acceleration() {
  _has_bits_[0] &= ~0x00000008u;
}
void Imu::clear_linear_acceleration() {
  if (linear_acceleration_ != NULL) linear_acceleration_->::gnss_driver::pb::Point3D::Clear();
  clear_has_linear_acceleration();
}
const ::gnss_driver::pb::Point3D& Imu::linear_acceleration() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Imu.linear_acceleration)
  return linear_acceleration_ != NULL ? *linear_acceleration_
                         : *::gnss_driver::pb::Point3D::internal_default_instance();
}
::gnss_driver::pb::Point3D* Imu::mutable_linear_acceleration() {
  set_has_linear_acceleration();
  if (linear_acceleration_ == NULL) {
    linear_acceleration_ = new ::gnss_driver::pb::Point3D;
  }
  // @@protoc_insertion_point(field_mutable:gnss_driver.pb.Imu.linear_acceleration)
  return linear_acceleration_;
}
::gnss_driver::pb::Point3D* Imu::release_linear_acceleration() {
  // @@protoc_insertion_point(field_release:gnss_driver.pb.Imu.linear_acceleration)
  clear_has_linear_acceleration();
  ::gnss_driver::pb::Point3D* temp = linear_acceleration_;
  linear_acceleration_ = NULL;
  return temp;
}
void Imu::set_allocated_linear_acceleration(::gnss_driver::pb::Point3D* linear_acceleration) {
  delete linear_acceleration_;
  linear_acceleration_ = linear_acceleration;
  if (linear_acceleration) {
    set_has_linear_acceleration();
  } else {
    clear_has_linear_acceleration();
  }
  // @@protoc_insertion_point(field_set_allocated:gnss_driver.pb.Imu.linear_acceleration)
}

// optional .gnss_driver.pb.Point3D angular_velocity = 5;
bool Imu::has_angular_velocity() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
void Imu::set_has_angular_velocity() {
  _has_bits_[0] |= 0x00000010u;
}
void Imu::clear_has_angular_velocity() {
  _has_bits_[0] &= ~0x00000010u;
}
void Imu::clear_angular_velocity() {
  if (angular_velocity_ != NULL) angular_velocity_->::gnss_driver::pb::Point3D::Clear();
  clear_has_angular_velocity();
}
const ::gnss_driver::pb::Point3D& Imu::angular_velocity() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.Imu.angular_velocity)
  return angular_velocity_ != NULL ? *angular_velocity_
                         : *::gnss_driver::pb::Point3D::internal_default_instance();
}
::gnss_driver::pb::Point3D* Imu::mutable_angular_velocity() {
  set_has_angular_velocity();
  if (angular_velocity_ == NULL) {
    angular_velocity_ = new ::gnss_driver::pb::Point3D;
  }
  // @@protoc_insertion_point(field_mutable:gnss_driver.pb.Imu.angular_velocity)
  return angular_velocity_;
}
::gnss_driver::pb::Point3D* Imu::release_angular_velocity() {
  // @@protoc_insertion_point(field_release:gnss_driver.pb.Imu.angular_velocity)
  clear_has_angular_velocity();
  ::gnss_driver::pb::Point3D* temp = angular_velocity_;
  angular_velocity_ = NULL;
  return temp;
}
void Imu::set_allocated_angular_velocity(::gnss_driver::pb::Point3D* angular_velocity) {
  delete angular_velocity_;
  angular_velocity_ = angular_velocity;
  if (angular_velocity) {
    set_has_angular_velocity();
  } else {
    clear_has_angular_velocity();
  }
  // @@protoc_insertion_point(field_set_allocated:gnss_driver.pb.Imu.angular_velocity)
}

inline const Imu* Imu::internal_default_instance() {
  return &Imu_default_instance_.get();
}
#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace pb
}  // namespace gnss_driver

// @@protoc_insertion_point(global_scope)
