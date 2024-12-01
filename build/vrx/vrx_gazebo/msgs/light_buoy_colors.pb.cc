// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: light_buoy_colors.proto

#include "light_buoy_colors.pb.h"

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

namespace light_buoy_colors_msgs {
namespace msgs {
class LightBuoyColorsDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<LightBuoyColors>
      _instance;
} _LightBuoyColors_default_instance_;
}  // namespace msgs
}  // namespace light_buoy_colors_msgs
namespace protobuf_light_5fbuoy_5fcolors_2eproto {
static void InitDefaultsLightBuoyColors() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::light_buoy_colors_msgs::msgs::_LightBuoyColors_default_instance_;
    new (ptr) ::light_buoy_colors_msgs::msgs::LightBuoyColors();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::light_buoy_colors_msgs::msgs::LightBuoyColors::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_LightBuoyColors =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsLightBuoyColors}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_LightBuoyColors.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::light_buoy_colors_msgs::msgs::LightBuoyColors, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::light_buoy_colors_msgs::msgs::LightBuoyColors, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::light_buoy_colors_msgs::msgs::LightBuoyColors, color_1_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::light_buoy_colors_msgs::msgs::LightBuoyColors, color_2_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::light_buoy_colors_msgs::msgs::LightBuoyColors, color_3_),
  0,
  1,
  2,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::light_buoy_colors_msgs::msgs::LightBuoyColors)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::light_buoy_colors_msgs::msgs::_LightBuoyColors_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "light_buoy_colors.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n\027light_buoy_colors.proto\022\033light_buoy_co"
      "lors_msgs.msgs\"D\n\017LightBuoyColors\022\017\n\007col"
      "or_1\030\001 \002(\t\022\017\n\007color_2\030\002 \002(\t\022\017\n\007color_3\030\003"
      " \002(\t"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 124);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "light_buoy_colors.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_light_5fbuoy_5fcolors_2eproto
namespace light_buoy_colors_msgs {
namespace msgs {

// ===================================================================

void LightBuoyColors::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int LightBuoyColors::kColor1FieldNumber;
const int LightBuoyColors::kColor2FieldNumber;
const int LightBuoyColors::kColor3FieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

LightBuoyColors::LightBuoyColors()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_light_5fbuoy_5fcolors_2eproto::scc_info_LightBuoyColors.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:light_buoy_colors_msgs.msgs.LightBuoyColors)
}
LightBuoyColors::LightBuoyColors(const LightBuoyColors& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  color_1_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_color_1()) {
    color_1_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.color_1_);
  }
  color_2_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_color_2()) {
    color_2_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.color_2_);
  }
  color_3_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_color_3()) {
    color_3_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.color_3_);
  }
  // @@protoc_insertion_point(copy_constructor:light_buoy_colors_msgs.msgs.LightBuoyColors)
}

void LightBuoyColors::SharedCtor() {
  color_1_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  color_2_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  color_3_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

LightBuoyColors::~LightBuoyColors() {
  // @@protoc_insertion_point(destructor:light_buoy_colors_msgs.msgs.LightBuoyColors)
  SharedDtor();
}

void LightBuoyColors::SharedDtor() {
  color_1_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  color_2_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  color_3_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

void LightBuoyColors::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* LightBuoyColors::descriptor() {
  ::protobuf_light_5fbuoy_5fcolors_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_light_5fbuoy_5fcolors_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const LightBuoyColors& LightBuoyColors::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_light_5fbuoy_5fcolors_2eproto::scc_info_LightBuoyColors.base);
  return *internal_default_instance();
}


void LightBuoyColors::Clear() {
// @@protoc_insertion_point(message_clear_start:light_buoy_colors_msgs.msgs.LightBuoyColors)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      color_1_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000002u) {
      color_2_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000004u) {
      color_3_.ClearNonDefaultToEmptyNoArena();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool LightBuoyColors::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:light_buoy_colors_msgs.msgs.LightBuoyColors)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string color_1 = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_color_1()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->color_1().data(), static_cast<int>(this->color_1().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "light_buoy_colors_msgs.msgs.LightBuoyColors.color_1");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required string color_2 = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_color_2()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->color_2().data(), static_cast<int>(this->color_2().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "light_buoy_colors_msgs.msgs.LightBuoyColors.color_2");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required string color_3 = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_color_3()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->color_3().data(), static_cast<int>(this->color_3().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "light_buoy_colors_msgs.msgs.LightBuoyColors.color_3");
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
  // @@protoc_insertion_point(parse_success:light_buoy_colors_msgs.msgs.LightBuoyColors)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:light_buoy_colors_msgs.msgs.LightBuoyColors)
  return false;
#undef DO_
}

void LightBuoyColors::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:light_buoy_colors_msgs.msgs.LightBuoyColors)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required string color_1 = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->color_1().data(), static_cast<int>(this->color_1().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "light_buoy_colors_msgs.msgs.LightBuoyColors.color_1");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->color_1(), output);
  }

  // required string color_2 = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->color_2().data(), static_cast<int>(this->color_2().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "light_buoy_colors_msgs.msgs.LightBuoyColors.color_2");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->color_2(), output);
  }

  // required string color_3 = 3;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->color_3().data(), static_cast<int>(this->color_3().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "light_buoy_colors_msgs.msgs.LightBuoyColors.color_3");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      3, this->color_3(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:light_buoy_colors_msgs.msgs.LightBuoyColors)
}

::google::protobuf::uint8* LightBuoyColors::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:light_buoy_colors_msgs.msgs.LightBuoyColors)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required string color_1 = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->color_1().data(), static_cast<int>(this->color_1().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "light_buoy_colors_msgs.msgs.LightBuoyColors.color_1");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->color_1(), target);
  }

  // required string color_2 = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->color_2().data(), static_cast<int>(this->color_2().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "light_buoy_colors_msgs.msgs.LightBuoyColors.color_2");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        2, this->color_2(), target);
  }

  // required string color_3 = 3;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->color_3().data(), static_cast<int>(this->color_3().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "light_buoy_colors_msgs.msgs.LightBuoyColors.color_3");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        3, this->color_3(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:light_buoy_colors_msgs.msgs.LightBuoyColors)
  return target;
}

size_t LightBuoyColors::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:light_buoy_colors_msgs.msgs.LightBuoyColors)
  size_t total_size = 0;

  if (has_color_1()) {
    // required string color_1 = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->color_1());
  }

  if (has_color_2()) {
    // required string color_2 = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->color_2());
  }

  if (has_color_3()) {
    // required string color_3 = 3;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->color_3());
  }

  return total_size;
}
size_t LightBuoyColors::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:light_buoy_colors_msgs.msgs.LightBuoyColors)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (((_has_bits_[0] & 0x00000007) ^ 0x00000007) == 0) {  // All required fields are present.
    // required string color_1 = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->color_1());

    // required string color_2 = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->color_2());

    // required string color_3 = 3;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->color_3());

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void LightBuoyColors::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:light_buoy_colors_msgs.msgs.LightBuoyColors)
  GOOGLE_DCHECK_NE(&from, this);
  const LightBuoyColors* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const LightBuoyColors>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:light_buoy_colors_msgs.msgs.LightBuoyColors)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:light_buoy_colors_msgs.msgs.LightBuoyColors)
    MergeFrom(*source);
  }
}

void LightBuoyColors::MergeFrom(const LightBuoyColors& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:light_buoy_colors_msgs.msgs.LightBuoyColors)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_color_1();
      color_1_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.color_1_);
    }
    if (cached_has_bits & 0x00000002u) {
      set_has_color_2();
      color_2_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.color_2_);
    }
    if (cached_has_bits & 0x00000004u) {
      set_has_color_3();
      color_3_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.color_3_);
    }
  }
}

void LightBuoyColors::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:light_buoy_colors_msgs.msgs.LightBuoyColors)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LightBuoyColors::CopyFrom(const LightBuoyColors& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:light_buoy_colors_msgs.msgs.LightBuoyColors)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LightBuoyColors::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000007) != 0x00000007) return false;
  return true;
}

void LightBuoyColors::Swap(LightBuoyColors* other) {
  if (other == this) return;
  InternalSwap(other);
}
void LightBuoyColors::InternalSwap(LightBuoyColors* other) {
  using std::swap;
  color_1_.Swap(&other->color_1_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  color_2_.Swap(&other->color_2_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  color_3_.Swap(&other->color_3_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata LightBuoyColors::GetMetadata() const {
  protobuf_light_5fbuoy_5fcolors_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_light_5fbuoy_5fcolors_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace msgs
}  // namespace light_buoy_colors_msgs
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::light_buoy_colors_msgs::msgs::LightBuoyColors* Arena::CreateMaybeMessage< ::light_buoy_colors_msgs::msgs::LightBuoyColors >(Arena* arena) {
  return Arena::CreateInternal< ::light_buoy_colors_msgs::msgs::LightBuoyColors >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)