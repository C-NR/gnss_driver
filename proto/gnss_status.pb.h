// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: gnss_status.proto

#ifndef PROTOBUF_gnss_5fstatus_2eproto__INCLUDED
#define PROTOBUF_gnss_5fstatus_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3001000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3001000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "header.pb.h"
// @@protoc_insertion_point(includes)

namespace gnss_driver {
namespace pb {

// Internal implementation detail -- do not call these.
void protobuf_AddDesc_gnss_5fstatus_2eproto();
void protobuf_InitDefaults_gnss_5fstatus_2eproto();
void protobuf_AssignDesc_gnss_5fstatus_2eproto();
void protobuf_ShutdownFile_gnss_5fstatus_2eproto();

class GnssStatus;
class InsStatus;
class StreamStatus;

enum StreamStatus_Type {
  StreamStatus_Type_DISCONNECTED = 0,
  StreamStatus_Type_CONNECTED = 1
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
  InsStatus_Type_GOOD = 2
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

class StreamStatus : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gnss_driver.pb.StreamStatus) */ {
 public:
  StreamStatus();
  virtual ~StreamStatus();

  StreamStatus(const StreamStatus& from);

  inline StreamStatus& operator=(const StreamStatus& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const StreamStatus& default_instance();

  static const StreamStatus* internal_default_instance();

  void Swap(StreamStatus* other);

  // implements Message ----------------------------------------------

  inline StreamStatus* New() const { return New(NULL); }

  StreamStatus* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const StreamStatus& from);
  void MergeFrom(const StreamStatus& from);
  void Clear();
  bool IsInitialized() const;

  size_t ByteSizeLong() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(StreamStatus* other);
  void UnsafeMergeFrom(const StreamStatus& from);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

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

  // optional .gnss_driver.pb.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::gnss_driver::pb::Header& header() const;
  ::gnss_driver::pb::Header* mutable_header();
  ::gnss_driver::pb::Header* release_header();
  void set_allocated_header(::gnss_driver::pb::Header* header);

  // optional .gnss_driver.pb.StreamStatus.Type ins_stream_type = 2 [default = DISCONNECTED];
  bool has_ins_stream_type() const;
  void clear_ins_stream_type();
  static const int kInsStreamTypeFieldNumber = 2;
  ::gnss_driver::pb::StreamStatus_Type ins_stream_type() const;
  void set_ins_stream_type(::gnss_driver::pb::StreamStatus_Type value);

  // optional .gnss_driver.pb.StreamStatus.Type rtk_stream_in_type = 3 [default = DISCONNECTED];
  bool has_rtk_stream_in_type() const;
  void clear_rtk_stream_in_type();
  static const int kRtkStreamInTypeFieldNumber = 3;
  ::gnss_driver::pb::StreamStatus_Type rtk_stream_in_type() const;
  void set_rtk_stream_in_type(::gnss_driver::pb::StreamStatus_Type value);

  // optional .gnss_driver.pb.StreamStatus.Type rtk_stream_out_type = 4 [default = DISCONNECTED];
  bool has_rtk_stream_out_type() const;
  void clear_rtk_stream_out_type();
  static const int kRtkStreamOutTypeFieldNumber = 4;
  ::gnss_driver::pb::StreamStatus_Type rtk_stream_out_type() const;
  void set_rtk_stream_out_type(::gnss_driver::pb::StreamStatus_Type value);

  // @@protoc_insertion_point(class_scope:gnss_driver.pb.StreamStatus)
 private:
  inline void set_has_header();
  inline void clear_has_header();
  inline void set_has_ins_stream_type();
  inline void clear_has_ins_stream_type();
  inline void set_has_rtk_stream_in_type();
  inline void clear_has_rtk_stream_in_type();
  inline void set_has_rtk_stream_out_type();
  inline void clear_has_rtk_stream_out_type();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::gnss_driver::pb::Header* header_;
  int ins_stream_type_;
  int rtk_stream_in_type_;
  int rtk_stream_out_type_;
  friend void  protobuf_InitDefaults_gnss_5fstatus_2eproto_impl();
  friend void  protobuf_AddDesc_gnss_5fstatus_2eproto_impl();
  friend void protobuf_AssignDesc_gnss_5fstatus_2eproto();
  friend void protobuf_ShutdownFile_gnss_5fstatus_2eproto();

  void InitAsDefaultInstance();
};
extern ::google::protobuf::internal::ExplicitlyConstructed<StreamStatus> StreamStatus_default_instance_;

// -------------------------------------------------------------------

class InsStatus : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gnss_driver.pb.InsStatus) */ {
 public:
  InsStatus();
  virtual ~InsStatus();

  InsStatus(const InsStatus& from);

  inline InsStatus& operator=(const InsStatus& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const InsStatus& default_instance();

  static const InsStatus* internal_default_instance();

  void Swap(InsStatus* other);

  // implements Message ----------------------------------------------

  inline InsStatus* New() const { return New(NULL); }

  InsStatus* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const InsStatus& from);
  void MergeFrom(const InsStatus& from);
  void Clear();
  bool IsInitialized() const;

  size_t ByteSizeLong() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(InsStatus* other);
  void UnsafeMergeFrom(const InsStatus& from);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

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

  // optional .gnss_driver.pb.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::gnss_driver::pb::Header& header() const;
  ::gnss_driver::pb::Header* mutable_header();
  ::gnss_driver::pb::Header* release_header();
  void set_allocated_header(::gnss_driver::pb::Header* header);

  // optional .gnss_driver.pb.InsStatus.Type type = 2 [default = INVALID];
  bool has_type() const;
  void clear_type();
  static const int kTypeFieldNumber = 2;
  ::gnss_driver::pb::InsStatus_Type type() const;
  void set_type(::gnss_driver::pb::InsStatus_Type value);

  // @@protoc_insertion_point(class_scope:gnss_driver.pb.InsStatus)
 private:
  inline void set_has_header();
  inline void clear_has_header();
  inline void set_has_type();
  inline void clear_has_type();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::gnss_driver::pb::Header* header_;
  int type_;
  friend void  protobuf_InitDefaults_gnss_5fstatus_2eproto_impl();
  friend void  protobuf_AddDesc_gnss_5fstatus_2eproto_impl();
  friend void protobuf_AssignDesc_gnss_5fstatus_2eproto();
  friend void protobuf_ShutdownFile_gnss_5fstatus_2eproto();

  void InitAsDefaultInstance();
};
extern ::google::protobuf::internal::ExplicitlyConstructed<InsStatus> InsStatus_default_instance_;

// -------------------------------------------------------------------

class GnssStatus : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gnss_driver.pb.GnssStatus) */ {
 public:
  GnssStatus();
  virtual ~GnssStatus();

  GnssStatus(const GnssStatus& from);

  inline GnssStatus& operator=(const GnssStatus& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const GnssStatus& default_instance();

  static const GnssStatus* internal_default_instance();

  void Swap(GnssStatus* other);

  // implements Message ----------------------------------------------

  inline GnssStatus* New() const { return New(NULL); }

  GnssStatus* New(::google::protobuf::Arena* arena) const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const GnssStatus& from);
  void MergeFrom(const GnssStatus& from);
  void Clear();
  bool IsInitialized() const;

  size_t ByteSizeLong() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const {
    return InternalSerializeWithCachedSizesToArray(false, output);
  }
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(GnssStatus* other);
  void UnsafeMergeFrom(const GnssStatus& from);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return _internal_metadata_.arena();
  }
  inline void* MaybeArenaPtr() const {
    return _internal_metadata_.raw_arena_ptr();
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional .gnss_driver.pb.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::gnss_driver::pb::Header& header() const;
  ::gnss_driver::pb::Header* mutable_header();
  ::gnss_driver::pb::Header* release_header();
  void set_allocated_header(::gnss_driver::pb::Header* header);

  // optional bool solution_completed = 2 [default = false];
  bool has_solution_completed() const;
  void clear_solution_completed();
  static const int kSolutionCompletedFieldNumber = 2;
  bool solution_completed() const;
  void set_solution_completed(bool value);

  // optional uint32 solution_status = 3 [default = 0];
  bool has_solution_status() const;
  void clear_solution_status();
  static const int kSolutionStatusFieldNumber = 3;
  ::google::protobuf::uint32 solution_status() const;
  void set_solution_status(::google::protobuf::uint32 value);

  // optional uint32 position_type = 4 [default = 0];
  bool has_position_type() const;
  void clear_position_type();
  static const int kPositionTypeFieldNumber = 4;
  ::google::protobuf::uint32 position_type() const;
  void set_position_type(::google::protobuf::uint32 value);

  // optional int32 num_sats = 5 [default = 0];
  bool has_num_sats() const;
  void clear_num_sats();
  static const int kNumSatsFieldNumber = 5;
  ::google::protobuf::int32 num_sats() const;
  void set_num_sats(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:gnss_driver.pb.GnssStatus)
 private:
  inline void set_has_header();
  inline void clear_has_header();
  inline void set_has_solution_completed();
  inline void clear_has_solution_completed();
  inline void set_has_solution_status();
  inline void clear_has_solution_status();
  inline void set_has_position_type();
  inline void clear_has_position_type();
  inline void set_has_num_sats();
  inline void clear_has_num_sats();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::gnss_driver::pb::Header* header_;
  bool solution_completed_;
  ::google::protobuf::uint32 solution_status_;
  ::google::protobuf::uint32 position_type_;
  ::google::protobuf::int32 num_sats_;
  friend void  protobuf_InitDefaults_gnss_5fstatus_2eproto_impl();
  friend void  protobuf_AddDesc_gnss_5fstatus_2eproto_impl();
  friend void protobuf_AssignDesc_gnss_5fstatus_2eproto();
  friend void protobuf_ShutdownFile_gnss_5fstatus_2eproto();

  void InitAsDefaultInstance();
};
extern ::google::protobuf::internal::ExplicitlyConstructed<GnssStatus> GnssStatus_default_instance_;

// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// StreamStatus

// optional .gnss_driver.pb.Header header = 1;
inline bool StreamStatus::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void StreamStatus::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
inline void StreamStatus::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void StreamStatus::clear_header() {
  if (header_ != NULL) header_->::gnss_driver::pb::Header::Clear();
  clear_has_header();
}
inline const ::gnss_driver::pb::Header& StreamStatus::header() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.StreamStatus.header)
  return header_ != NULL ? *header_
                         : *::gnss_driver::pb::Header::internal_default_instance();
}
inline ::gnss_driver::pb::Header* StreamStatus::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    header_ = new ::gnss_driver::pb::Header;
  }
  // @@protoc_insertion_point(field_mutable:gnss_driver.pb.StreamStatus.header)
  return header_;
}
inline ::gnss_driver::pb::Header* StreamStatus::release_header() {
  // @@protoc_insertion_point(field_release:gnss_driver.pb.StreamStatus.header)
  clear_has_header();
  ::gnss_driver::pb::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline void StreamStatus::set_allocated_header(::gnss_driver::pb::Header* header) {
  delete header_;
  header_ = header;
  if (header) {
    set_has_header();
  } else {
    clear_has_header();
  }
  // @@protoc_insertion_point(field_set_allocated:gnss_driver.pb.StreamStatus.header)
}

// optional .gnss_driver.pb.StreamStatus.Type ins_stream_type = 2 [default = DISCONNECTED];
inline bool StreamStatus::has_ins_stream_type() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void StreamStatus::set_has_ins_stream_type() {
  _has_bits_[0] |= 0x00000002u;
}
inline void StreamStatus::clear_has_ins_stream_type() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void StreamStatus::clear_ins_stream_type() {
  ins_stream_type_ = 0;
  clear_has_ins_stream_type();
}
inline ::gnss_driver::pb::StreamStatus_Type StreamStatus::ins_stream_type() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.StreamStatus.ins_stream_type)
  return static_cast< ::gnss_driver::pb::StreamStatus_Type >(ins_stream_type_);
}
inline void StreamStatus::set_ins_stream_type(::gnss_driver::pb::StreamStatus_Type value) {
  assert(::gnss_driver::pb::StreamStatus_Type_IsValid(value));
  set_has_ins_stream_type();
  ins_stream_type_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.StreamStatus.ins_stream_type)
}

// optional .gnss_driver.pb.StreamStatus.Type rtk_stream_in_type = 3 [default = DISCONNECTED];
inline bool StreamStatus::has_rtk_stream_in_type() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void StreamStatus::set_has_rtk_stream_in_type() {
  _has_bits_[0] |= 0x00000004u;
}
inline void StreamStatus::clear_has_rtk_stream_in_type() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void StreamStatus::clear_rtk_stream_in_type() {
  rtk_stream_in_type_ = 0;
  clear_has_rtk_stream_in_type();
}
inline ::gnss_driver::pb::StreamStatus_Type StreamStatus::rtk_stream_in_type() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.StreamStatus.rtk_stream_in_type)
  return static_cast< ::gnss_driver::pb::StreamStatus_Type >(rtk_stream_in_type_);
}
inline void StreamStatus::set_rtk_stream_in_type(::gnss_driver::pb::StreamStatus_Type value) {
  assert(::gnss_driver::pb::StreamStatus_Type_IsValid(value));
  set_has_rtk_stream_in_type();
  rtk_stream_in_type_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.StreamStatus.rtk_stream_in_type)
}

// optional .gnss_driver.pb.StreamStatus.Type rtk_stream_out_type = 4 [default = DISCONNECTED];
inline bool StreamStatus::has_rtk_stream_out_type() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void StreamStatus::set_has_rtk_stream_out_type() {
  _has_bits_[0] |= 0x00000008u;
}
inline void StreamStatus::clear_has_rtk_stream_out_type() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void StreamStatus::clear_rtk_stream_out_type() {
  rtk_stream_out_type_ = 0;
  clear_has_rtk_stream_out_type();
}
inline ::gnss_driver::pb::StreamStatus_Type StreamStatus::rtk_stream_out_type() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.StreamStatus.rtk_stream_out_type)
  return static_cast< ::gnss_driver::pb::StreamStatus_Type >(rtk_stream_out_type_);
}
inline void StreamStatus::set_rtk_stream_out_type(::gnss_driver::pb::StreamStatus_Type value) {
  assert(::gnss_driver::pb::StreamStatus_Type_IsValid(value));
  set_has_rtk_stream_out_type();
  rtk_stream_out_type_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.StreamStatus.rtk_stream_out_type)
}

inline const StreamStatus* StreamStatus::internal_default_instance() {
  return &StreamStatus_default_instance_.get();
}
// -------------------------------------------------------------------

// InsStatus

// optional .gnss_driver.pb.Header header = 1;
inline bool InsStatus::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void InsStatus::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
inline void InsStatus::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void InsStatus::clear_header() {
  if (header_ != NULL) header_->::gnss_driver::pb::Header::Clear();
  clear_has_header();
}
inline const ::gnss_driver::pb::Header& InsStatus::header() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.InsStatus.header)
  return header_ != NULL ? *header_
                         : *::gnss_driver::pb::Header::internal_default_instance();
}
inline ::gnss_driver::pb::Header* InsStatus::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    header_ = new ::gnss_driver::pb::Header;
  }
  // @@protoc_insertion_point(field_mutable:gnss_driver.pb.InsStatus.header)
  return header_;
}
inline ::gnss_driver::pb::Header* InsStatus::release_header() {
  // @@protoc_insertion_point(field_release:gnss_driver.pb.InsStatus.header)
  clear_has_header();
  ::gnss_driver::pb::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline void InsStatus::set_allocated_header(::gnss_driver::pb::Header* header) {
  delete header_;
  header_ = header;
  if (header) {
    set_has_header();
  } else {
    clear_has_header();
  }
  // @@protoc_insertion_point(field_set_allocated:gnss_driver.pb.InsStatus.header)
}

// optional .gnss_driver.pb.InsStatus.Type type = 2 [default = INVALID];
inline bool InsStatus::has_type() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void InsStatus::set_has_type() {
  _has_bits_[0] |= 0x00000002u;
}
inline void InsStatus::clear_has_type() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void InsStatus::clear_type() {
  type_ = 0;
  clear_has_type();
}
inline ::gnss_driver::pb::InsStatus_Type InsStatus::type() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.InsStatus.type)
  return static_cast< ::gnss_driver::pb::InsStatus_Type >(type_);
}
inline void InsStatus::set_type(::gnss_driver::pb::InsStatus_Type value) {
  assert(::gnss_driver::pb::InsStatus_Type_IsValid(value));
  set_has_type();
  type_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.InsStatus.type)
}

inline const InsStatus* InsStatus::internal_default_instance() {
  return &InsStatus_default_instance_.get();
}
// -------------------------------------------------------------------

// GnssStatus

// optional .gnss_driver.pb.Header header = 1;
inline bool GnssStatus::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void GnssStatus::set_has_header() {
  _has_bits_[0] |= 0x00000001u;
}
inline void GnssStatus::clear_has_header() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void GnssStatus::clear_header() {
  if (header_ != NULL) header_->::gnss_driver::pb::Header::Clear();
  clear_has_header();
}
inline const ::gnss_driver::pb::Header& GnssStatus::header() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.GnssStatus.header)
  return header_ != NULL ? *header_
                         : *::gnss_driver::pb::Header::internal_default_instance();
}
inline ::gnss_driver::pb::Header* GnssStatus::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    header_ = new ::gnss_driver::pb::Header;
  }
  // @@protoc_insertion_point(field_mutable:gnss_driver.pb.GnssStatus.header)
  return header_;
}
inline ::gnss_driver::pb::Header* GnssStatus::release_header() {
  // @@protoc_insertion_point(field_release:gnss_driver.pb.GnssStatus.header)
  clear_has_header();
  ::gnss_driver::pb::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline void GnssStatus::set_allocated_header(::gnss_driver::pb::Header* header) {
  delete header_;
  header_ = header;
  if (header) {
    set_has_header();
  } else {
    clear_has_header();
  }
  // @@protoc_insertion_point(field_set_allocated:gnss_driver.pb.GnssStatus.header)
}

// optional bool solution_completed = 2 [default = false];
inline bool GnssStatus::has_solution_completed() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void GnssStatus::set_has_solution_completed() {
  _has_bits_[0] |= 0x00000002u;
}
inline void GnssStatus::clear_has_solution_completed() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void GnssStatus::clear_solution_completed() {
  solution_completed_ = false;
  clear_has_solution_completed();
}
inline bool GnssStatus::solution_completed() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.GnssStatus.solution_completed)
  return solution_completed_;
}
inline void GnssStatus::set_solution_completed(bool value) {
  set_has_solution_completed();
  solution_completed_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.GnssStatus.solution_completed)
}

// optional uint32 solution_status = 3 [default = 0];
inline bool GnssStatus::has_solution_status() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void GnssStatus::set_has_solution_status() {
  _has_bits_[0] |= 0x00000004u;
}
inline void GnssStatus::clear_has_solution_status() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void GnssStatus::clear_solution_status() {
  solution_status_ = 0u;
  clear_has_solution_status();
}
inline ::google::protobuf::uint32 GnssStatus::solution_status() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.GnssStatus.solution_status)
  return solution_status_;
}
inline void GnssStatus::set_solution_status(::google::protobuf::uint32 value) {
  set_has_solution_status();
  solution_status_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.GnssStatus.solution_status)
}

// optional uint32 position_type = 4 [default = 0];
inline bool GnssStatus::has_position_type() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void GnssStatus::set_has_position_type() {
  _has_bits_[0] |= 0x00000008u;
}
inline void GnssStatus::clear_has_position_type() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void GnssStatus::clear_position_type() {
  position_type_ = 0u;
  clear_has_position_type();
}
inline ::google::protobuf::uint32 GnssStatus::position_type() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.GnssStatus.position_type)
  return position_type_;
}
inline void GnssStatus::set_position_type(::google::protobuf::uint32 value) {
  set_has_position_type();
  position_type_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.GnssStatus.position_type)
}

// optional int32 num_sats = 5 [default = 0];
inline bool GnssStatus::has_num_sats() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void GnssStatus::set_has_num_sats() {
  _has_bits_[0] |= 0x00000010u;
}
inline void GnssStatus::clear_has_num_sats() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void GnssStatus::clear_num_sats() {
  num_sats_ = 0;
  clear_has_num_sats();
}
inline ::google::protobuf::int32 GnssStatus::num_sats() const {
  // @@protoc_insertion_point(field_get:gnss_driver.pb.GnssStatus.num_sats)
  return num_sats_;
}
inline void GnssStatus::set_num_sats(::google::protobuf::int32 value) {
  set_has_num_sats();
  num_sats_ = value;
  // @@protoc_insertion_point(field_set:gnss_driver.pb.GnssStatus.num_sats)
}

inline const GnssStatus* GnssStatus::internal_default_instance() {
  return &GnssStatus_default_instance_.get();
}
#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace pb
}  // namespace gnss_driver

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::gnss_driver::pb::StreamStatus_Type> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::gnss_driver::pb::StreamStatus_Type>() {
  return ::gnss_driver::pb::StreamStatus_Type_descriptor();
}
template <> struct is_proto_enum< ::gnss_driver::pb::InsStatus_Type> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::gnss_driver::pb::InsStatus_Type>() {
  return ::gnss_driver::pb::InsStatus_Type_descriptor();
}

}  // namespace protobuf
}  // namespace google
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_gnss_5fstatus_2eproto__INCLUDED
