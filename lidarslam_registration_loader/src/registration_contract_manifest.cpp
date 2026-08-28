// Copyright 2026 Sasaki
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "lidarslam_registration_loader/registration_plugin_loader.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace lidarslam
{
namespace plugins
{
namespace registration
{
namespace shell
{
namespace
{

namespace fs = std::filesystem;

LoadFailure failure(const LoadFailureCode code, const std::string & message)
{
  return LoadFailure{code, message};
}

// Small self-contained SHA-256 implementation.  The loader deliberately does
// not gain an OpenSSL ABI dependency merely to validate installed sidecars.
class Sha256 final
{
public:
  Sha256()
  : state_{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
      0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U} {}

  void update(const std::uint8_t * data, const std::size_t length)
  {
    for (std::size_t index = 0U; index < length; ++index) {
      block_[block_length_++] = data[index];
      if (block_length_ == block_.size()) {
        transform(block_.data());
        bit_length_ += 512U;
        block_length_ = 0U;
      }
    }
  }

  std::string final()
  {
    const std::uint64_t message_bits = bit_length_ + block_length_ * 8U;
    block_[block_length_++] = 0x80U;
    if (block_length_ > 56U) {
      while (block_length_ < 64U) {
        block_[block_length_++] = 0U;
      }
      transform(block_.data());
      block_length_ = 0U;
    }
    while (block_length_ < 56U) {
      block_[block_length_++] = 0U;
    }
    for (int shift = 7; shift >= 0; --shift) {
      block_[block_length_++] = static_cast<std::uint8_t>(
        (message_bits >> (static_cast<unsigned int>(shift) * 8U)) & 0xffU);
    }
    transform(block_.data());

    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (const std::uint32_t word : state_) {
      output << std::setw(8) << word;
    }
    return output.str();
  }

private:
  static constexpr std::array<std::uint32_t, 64> kRoundConstants{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U,
    0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U,
    0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
    0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U,
    0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U,
    0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
    0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
    0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U,
    0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
    0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U,
    0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

  static std::uint32_t rotr(const std::uint32_t value, const unsigned int shift)
  {
    return (value >> shift) | (value << (32U - shift));
  }

  void transform(const std::uint8_t * block)
  {
    std::array<std::uint32_t, 64> schedule{};
    for (std::size_t index = 0U; index < 16U; ++index) {
      schedule[index] =
        (static_cast<std::uint32_t>(block[index * 4U]) << 24U) |
        (static_cast<std::uint32_t>(block[index * 4U + 1U]) << 16U) |
        (static_cast<std::uint32_t>(block[index * 4U + 2U]) << 8U) |
        static_cast<std::uint32_t>(block[index * 4U + 3U]);
    }
    for (std::size_t index = 16U; index < 64U; ++index) {
      const std::uint32_t s0 = rotr(schedule[index - 15U], 7U) ^
        rotr(schedule[index - 15U], 18U) ^ (schedule[index - 15U] >> 3U);
      const std::uint32_t s1 = rotr(schedule[index - 2U], 17U) ^
        rotr(schedule[index - 2U], 19U) ^ (schedule[index - 2U] >> 10U);
      schedule[index] = schedule[index - 16U] + s0 + schedule[index - 7U] + s1;
    }

    std::uint32_t a = state_[0];
    std::uint32_t b = state_[1];
    std::uint32_t c = state_[2];
    std::uint32_t d = state_[3];
    std::uint32_t e = state_[4];
    std::uint32_t f = state_[5];
    std::uint32_t g = state_[6];
    std::uint32_t h = state_[7];
    for (std::size_t index = 0U; index < 64U; ++index) {
      const std::uint32_t s1 = rotr(e, 6U) ^ rotr(e, 11U) ^ rotr(e, 25U);
      const std::uint32_t choice = (e & f) ^ ((~e) & g);
      const std::uint32_t temporary1 = h + s1 + choice + kRoundConstants[index] +
        schedule[index];
      const std::uint32_t s0 = rotr(a, 2U) ^ rotr(a, 13U) ^ rotr(a, 22U);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temporary2 = s0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temporary1;
      d = c;
      c = b;
      b = a;
      a = temporary1 + temporary2;
    }
    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<std::uint32_t, 8> state_;
  std::array<std::uint8_t, 64> block_{};
  std::size_t block_length_{0U};
  std::uint64_t bit_length_{0U};
};

std::string sha256Bytes(const std::string & bytes)
{
  Sha256 hash;
  hash.update(reinterpret_cast<const std::uint8_t *>(bytes.data()), bytes.size());
  return hash.final();
}

bool readFile(const fs::path & path, std::string * bytes, std::string * error)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    if (error != nullptr) {
      *error = "cannot open '" + path.string() + "'";
    }
    return false;
  }
  std::ostringstream stream;
  stream << input.rdbuf();
  if (!input.good() && !input.eof()) {
    if (error != nullptr) {
      *error = "cannot read '" + path.string() + "'";
    }
    return false;
  }
  *bytes = stream.str();
  return true;
}

std::string sha256File(const fs::path & path, std::string * error)
{
  std::string bytes;
  if (!readFile(path, &bytes, error)) {
    return {};
  }
  return sha256Bytes(bytes);
}

bool isHexSha256(const std::string & value)
{
  return value.size() == 64U && std::all_of(value.begin(), value.end(), [](const char c) {
             return std::isxdigit(static_cast<unsigned char>(c)) != 0;
    });
}

std::string sanitizeClassId(const std::string & class_id)
{
  std::string result;
  result.reserve(class_id.size());
  for (const unsigned char character : class_id) {
    result.push_back(
      (std::isalnum(character) != 0 || character == '_' || character == '-') ?
      static_cast<char>(character) : '_');
  }
  return result;
}

struct JsonScalar
{
  bool string_value{false};
  std::string value;
};

void skipWhitespace(const std::string & input, std::size_t * position)
{
  while (*position < input.size() &&
    std::isspace(static_cast<unsigned char>(input[*position])) != 0)
  {
    ++(*position);
  }
}

bool parseJsonString(
  const std::string & input, std::size_t * position, std::string * value)
{
  if (*position >= input.size() || input[*position] != '"') {
    return false;
  }
  ++(*position);
  std::string result;
  while (*position < input.size()) {
    const char character = input[(*position)++];
    if (character == '"') {
      *value = std::move(result);
      return true;
    }
    if (character == '\\' ||
      static_cast<unsigned char>(character) < static_cast<unsigned char>(0x20))
    {
      // Sidecar strings are generated from constrained identifiers.  Reject
      // escapes instead of accepting multiple spellings of one identity.
      return false;
    }
    result.push_back(character);
  }
  return false;
}

bool parseJsonObject(const std::string & input, std::map<std::string, JsonScalar> * values)
{
  std::size_t position = 0U;
  skipWhitespace(input, &position);
  if (position >= input.size() || input[position++] != '{') {
    return false;
  }
  skipWhitespace(input, &position);
  if (position < input.size() && input[position] == '}') {
    ++position;
    skipWhitespace(input, &position);
    return position == input.size();
  }
  while (position < input.size()) {
    std::string key;
    if (!parseJsonString(input, &position, &key)) {
      return false;
    }
    if (values->find(key) != values->end()) {
      return false;
    }
    skipWhitespace(input, &position);
    if (position >= input.size() || input[position++] != ':') {
      return false;
    }
    skipWhitespace(input, &position);
    JsonScalar scalar;
    if (position < input.size() && input[position] == '"') {
      scalar.string_value = true;
      if (!parseJsonString(input, &position, &scalar.value)) {
        return false;
      }
    } else {
      const std::size_t start = position;
      while (position < input.size() &&
        (std::isdigit(static_cast<unsigned char>(input[position])) != 0 ||
        input[position] == '-'))
      {
        ++position;
      }
      if (start == position) {
        return false;
      }
      scalar.value = input.substr(start, position - start);
    }
    values->emplace(std::move(key), std::move(scalar));
    skipWhitespace(input, &position);
    if (position >= input.size()) {
      return false;
    }
    if (input[position] == '}') {
      ++position;
      skipWhitespace(input, &position);
      return position == input.size();
    }
    if (input[position++] != ',') {
      return false;
    }
    skipWhitespace(input, &position);
  }
  return false;
}

bool unsignedValue(
  const std::map<std::string, JsonScalar> & values, const char * key, std::uint64_t * output)
{
  const auto iterator = values.find(key);
  if (iterator == values.end() || iterator->second.string_value || iterator->second.value.empty() ||
    iterator->second.value.front() == '-')
  {
    return false;
  }
  try {
    std::size_t consumed = 0U;
    const std::uint64_t value = std::stoull(iterator->second.value, &consumed, 10);
    if (consumed != iterator->second.value.size()) {
      return false;
    }
    *output = value;
    return true;
  } catch (...) {
    return false;
  }
}

bool stringValue(
  const std::map<std::string, JsonScalar> & values, const char * key, std::string * output)
{
  const auto iterator = values.find(key);
  if (iterator == values.end() || !iterator->second.string_value) {
    return false;
  }
  *output = iterator->second.value;
  return true;
}

std::string canonicalManifestIdentity(const RegistrationContractManifest & manifest)
{
  std::ostringstream identity;
  identity << "schema=" << manifest.schema <<
    "|schema_version=" << manifest.schema_version <<
    "|class_id=" << manifest.class_id <<
    "|plugin_xml_sha256=" << manifest.plugin_xml_sha256 <<
    "|plugin_xml_size_bytes=" << manifest.plugin_xml_size_bytes <<
    "|dso_sha256=" << manifest.dso_sha256 <<
    "|dso_size_bytes=" << manifest.dso_size_bytes <<
    "|abi_epoch=" << manifest.abi_epoch <<
    "|toolchain_tag=" << manifest.toolchain_tag <<
    "|interface_contract_sha256=" << manifest.interface_contract_sha256 <<
    "|api_min_major=" << manifest.api_min.major <<
    "|api_min_minor=" << manifest.api_min.minor <<
    "|api_max_major=" << manifest.api_max.major <<
    "|api_max_minor=" << manifest.api_max.minor <<
    "|required_capability_bits=" << manifest.required_capability_bits <<
    "|optional_capability_bits=" << manifest.optional_capability_bits <<
    "|target_policy=" << static_cast<int>(manifest.target_policy) <<
    "|correspondence_metric=" << static_cast<int>(manifest.correspondence_metric) <<
    "|thread_model=" << static_cast<int>(manifest.thread_model) <<
    "|cancellation_model=" << static_cast<int>(manifest.cancellation_model) <<
    "|config_schema_id=" << manifest.config_schema_id <<
    "|config_schema_version=" << manifest.config_schema_version <<
    "|config_schema_sha256=" << manifest.config_schema_sha256;
  return identity.str();
}

bool allKnownKeys(const std::map<std::string, JsonScalar> & values)
{
  static const std::set<std::string> keys{
    "schema", "schema_version", "class_id", "plugin_xml_sha256", "plugin_xml_size_bytes",
    "dso_sha256", "dso_size_bytes", "abi_epoch", "toolchain_tag", "interface_contract_sha256",
    "api_min_major", "api_min_minor", "api_max_major", "api_max_minor",
    "required_capability_bits", "optional_capability_bits", "target_policy",
    "correspondence_metric", "thread_model", "cancellation_model", "config_schema_id",
    "config_schema_version",
    "config_schema_sha256", "manifest_sha256"};
  for (const auto & entry : values) {
    if (keys.find(entry.first) == keys.end()) {
      return false;
    }
  }
  return values.size() == keys.size();
}

bool descriptorMatchesManifest(
  const RegistrationRuntimeDescriptor & descriptor,
  const RegistrationContractManifest & manifest)
{
  // The installed sidecar and the post-instantiation descriptor have
  // deliberately distinct schema identities: the former describes the
  // immutable XML/DSO contract, while the latter describes the provider's
  // runtime descriptor payload.  Their bound identity fields below must
  // match exactly; comparing the schema labels themselves would reject every
  // valid provider by construction.
  return descriptor.class_id == manifest.class_id &&
         descriptor.api_min.major == manifest.api_min.major &&
         descriptor.api_min.minor == manifest.api_min.minor &&
         descriptor.api_max.major == manifest.api_max.major &&
         descriptor.api_max.minor == manifest.api_max.minor &&
         descriptor.required_capability_bits == manifest.required_capability_bits &&
         descriptor.optional_capability_bits == manifest.optional_capability_bits &&
         descriptor.target_policy == manifest.target_policy &&
         descriptor.correspondence_metric == manifest.correspondence_metric &&
         descriptor.thread_model == manifest.thread_model &&
         descriptor.cancellation_model == manifest.cancellation_model &&
         descriptor.abi_epoch == manifest.abi_epoch &&
         descriptor.toolchain_tag == manifest.toolchain_tag &&
         descriptor.config_schema_id == manifest.config_schema_id &&
         descriptor.config_schema_version == manifest.config_schema_version &&
         descriptor.config_schema_sha256 == manifest.config_schema_sha256 &&
         descriptor.interface_contract_sha256 == manifest.interface_contract_sha256;
}

bool descriptorMatchesExpected(
  const RegistrationRuntimeDescriptor & descriptor,
  const RegistrationRuntimeDescriptor & expected)
{
  return descriptor.schema == expected.schema &&
         descriptor.schema_version == expected.schema_version &&
         descriptor.class_id == expected.class_id &&
         descriptor.api_min.major == expected.api_min.major &&
         descriptor.api_min.minor == expected.api_min.minor &&
         descriptor.api_max.major == expected.api_max.major &&
         descriptor.api_max.minor == expected.api_max.minor &&
         descriptor.required_capability_bits == expected.required_capability_bits &&
         descriptor.optional_capability_bits == expected.optional_capability_bits &&
         descriptor.target_policy == expected.target_policy &&
         descriptor.correspondence_metric == expected.correspondence_metric &&
         descriptor.thread_model == expected.thread_model &&
         descriptor.cancellation_model == expected.cancellation_model &&
         descriptor.abi_epoch == expected.abi_epoch &&
         descriptor.toolchain_tag == expected.toolchain_tag &&
         descriptor.config_schema_id == expected.config_schema_id &&
         descriptor.config_schema_version == expected.config_schema_version &&
         descriptor.config_schema_sha256 == expected.config_schema_sha256 &&
         descriptor.interface_contract_sha256 == expected.interface_contract_sha256;
}

}  // namespace

std::string registrationContractSidecarPath(
  const std::string & plugin_manifest_path, const std::string & class_id)
{
  // Keep the sidecar adjacent to the path returned by pluginlib.  XML
  // resources may be symlink-installed; following the XML symlink here would
  // incorrectly look beside the source-tree target instead of beside the
  // installed resource and would make a clean install unverifiable.
  const fs::path base = fs::path(plugin_manifest_path).lexically_normal();
  return (base.parent_path() /
         (base.filename().string() + "." + sanitizeClassId(class_id) + ".contract.json")).string();
}

std::string registrationContractManifestDigest(const RegistrationContractManifest & manifest)
{
  return sha256Bytes(canonicalManifestIdentity(manifest));
}

std::string registrationContractFileSha256(const std::string & path)
{
  std::string error;
  return sha256File(fs::path(path), &error);
}

LoadFailure readAndValidateRegistrationContractManifest(
  const std::string & class_id,
  const std::string & library_path,
  const std::string & plugin_manifest_path,
  RegistrationContractManifest * manifest)
{
  if (manifest == nullptr) {
    return failure(LoadFailureCode::kInvalidRequest, "contract manifest output is null");
  }
  if (class_id.empty()) {
    return failure(LoadFailureCode::kContractManifestInvalid,
              "contract manifest class ID is empty");
  }
  // Keep this public validation entry point fail-closed even when callers do
  // not go through RegistrationPluginLoader::load().  In particular, the
  // sidecar must never bless a relative, symlinked, missing, or non-regular
  // DSO/XML path.
  const LoadFailure provenance = validateExternalDsoProvenance(
    class_id, library_path, plugin_manifest_path);
  if (!provenance.ok()) {
    return provenance;
  }
  const fs::path sidecar(registrationContractSidecarPath(plugin_manifest_path, class_id));
  std::error_code status_error;
  const fs::file_status sidecar_status = fs::symlink_status(sidecar, status_error);
  if (status_error || !fs::exists(sidecar_status)) {
    return failure(
      LoadFailureCode::kContractManifestMissing,
      "registration contract sidecar is missing: '" + sidecar.string() + "'");
  }
  if (fs::is_symlink(sidecar_status) || !fs::is_regular_file(sidecar_status)) {
    return failure(
      LoadFailureCode::kContractManifestInvalid,
      "registration contract sidecar must be a regular non-symlink file: '" + sidecar.string() +
              "'");
  }

  std::string sidecar_bytes;
  std::string read_error;
  if (!readFile(sidecar, &sidecar_bytes, &read_error) || sidecar_bytes.size() > 65536U) {
    return failure(LoadFailureCode::kContractManifestInvalid, read_error.empty() ?
      "registration contract sidecar is too large" : read_error);
  }
  std::map<std::string, JsonScalar> values;
  if (!parseJsonObject(sidecar_bytes, &values) || !allKnownKeys(values)) {
    return failure(
      LoadFailureCode::kContractManifestInvalid,
      "registration contract sidecar is not the strict v1 object: '" + sidecar.string() + "'");
  }

  RegistrationContractManifest parsed;
  std::uint64_t number = 0U;
  if (!stringValue(values, "schema", &parsed.schema) ||
    !unsignedValue(values, "schema_version",
            &number) || number > std::numeric_limits<std::uint32_t>::max())
  {
    return failure(LoadFailureCode::kContractManifestInvalid,
              "contract schema identity is malformed");
  }
  parsed.schema_version = static_cast<std::uint32_t>(number);
  if (!stringValue(values, "class_id", &parsed.class_id) ||
    !stringValue(values, "plugin_xml_sha256", &parsed.plugin_xml_sha256) ||
    !unsignedValue(values, "plugin_xml_size_bytes", &parsed.plugin_xml_size_bytes) ||
    !stringValue(values, "dso_sha256", &parsed.dso_sha256) ||
    !unsignedValue(values, "dso_size_bytes", &parsed.dso_size_bytes) ||
    !stringValue(values, "abi_epoch", &parsed.abi_epoch) ||
    !stringValue(values, "toolchain_tag", &parsed.toolchain_tag) ||
    !stringValue(values, "interface_contract_sha256", &parsed.interface_contract_sha256) ||
    !unsignedValue(values, "api_min_major",
            &number) || number > std::numeric_limits<std::uint16_t>::max())
  {
    return failure(LoadFailureCode::kContractManifestInvalid,
              "contract provenance identity is malformed");
  }
  parsed.api_min.major = static_cast<std::uint16_t>(number);
  if (!unsignedValue(values, "api_min_minor",
  &number) || number > std::numeric_limits<std::uint16_t>::max())
  {
    return failure(LoadFailureCode::kContractManifestInvalid, "contract API minimum is malformed");
  }
  parsed.api_min.minor = static_cast<std::uint16_t>(number);
  if (!unsignedValue(values, "api_max_major",
  &number) || number > std::numeric_limits<std::uint16_t>::max())
  {
    return failure(LoadFailureCode::kContractManifestInvalid, "contract API maximum is malformed");
  }
  parsed.api_max.major = static_cast<std::uint16_t>(number);
  if (!unsignedValue(values, "api_max_minor",
  &number) || number > std::numeric_limits<std::uint16_t>::max())
  {
    return failure(LoadFailureCode::kContractManifestInvalid, "contract API maximum is malformed");
  }
  parsed.api_max.minor = static_cast<std::uint16_t>(number);
  if (!unsignedValue(values, "required_capability_bits", &parsed.required_capability_bits) ||
    !unsignedValue(values, "optional_capability_bits", &parsed.optional_capability_bits) ||
    !unsignedValue(values, "target_policy", &number) || number > 2U)
  {
    return failure(LoadFailureCode::kContractManifestInvalid,
              "contract capability identity is malformed");
  }
  parsed.target_policy = static_cast<TargetPolicy>(number);
  if (!unsignedValue(values, "correspondence_metric", &number) || number > 2U) {
    return failure(LoadFailureCode::kContractManifestInvalid,
              "contract correspondence metric is malformed");
  }
  parsed.correspondence_metric = static_cast<CorrespondenceMetric>(number);
  if (!unsignedValue(values, "thread_model", &number) || number > 1U) {
    return failure(LoadFailureCode::kContractManifestInvalid, "contract thread model is malformed");
  }
  parsed.thread_model = static_cast<ThreadModel>(number);
  if (!unsignedValue(values, "cancellation_model", &number) || number > 1U) {
    return failure(LoadFailureCode::kContractManifestInvalid,
              "contract cancellation model is malformed");
  }
  parsed.cancellation_model = static_cast<CancellationModel>(number);
  if (!stringValue(values, "config_schema_id", &parsed.config_schema_id) ||
    !unsignedValue(values, "config_schema_version", &number) ||
    number > std::numeric_limits<std::uint32_t>::max())
  {
    return failure(LoadFailureCode::kContractManifestInvalid,
              "contract config schema identity is malformed");
  }
  parsed.config_schema_version = static_cast<std::uint32_t>(number);
  if (!stringValue(values, "config_schema_sha256", &parsed.config_schema_sha256) ||
    !stringValue(values, "manifest_sha256", &parsed.manifest_sha256))
  {
    return failure(LoadFailureCode::kContractManifestInvalid,
              "contract digest fields are malformed");
  }

  const std::uint64_t known_capabilities = 0xffU;
  if (parsed.schema != "lidarslam-registration-contract-manifest-v1" ||
    parsed.schema_version != 1U || parsed.class_id != class_id ||
    parsed.api_min.major != parsed.api_max.major ||
    parsed.api_min.minor > parsed.api_max.minor ||
    (parsed.required_capability_bits & ~known_capabilities) != 0U ||
    (parsed.optional_capability_bits & ~known_capabilities) != 0U ||
    (parsed.required_capability_bits & parsed.optional_capability_bits) != 0U ||
    !isHexSha256(parsed.plugin_xml_sha256) || !isHexSha256(parsed.dso_sha256) ||
    !isHexSha256(parsed.interface_contract_sha256) || !isHexSha256(parsed.config_schema_sha256) ||
    !isHexSha256(parsed.manifest_sha256) || parsed.abi_epoch.empty() ||
    parsed.toolchain_tag.empty() ||
    parsed.config_schema_id.empty() || parsed.config_schema_version == 0U)
  {
    return failure(LoadFailureCode::kContractManifestInvalid,
              "contract manifest identity is inconsistent");
  }

  const fs::path xml(plugin_manifest_path);
  const fs::path dso(library_path);
  std::error_code xml_size_error;
  std::error_code dso_size_error;
  const std::uint64_t xml_size = fs::file_size(xml, xml_size_error);
  const std::uint64_t dso_size = fs::file_size(dso, dso_size_error);
  std::string hash_error;
  const std::string xml_hash = sha256File(xml, &hash_error);
  if (xml_size_error || xml_size != parsed.plugin_xml_size_bytes ||
    xml_hash != parsed.plugin_xml_sha256)
  {
    return failure(LoadFailureCode::kContractManifestInvalid,
      "plugin XML size/SHA256 does not match the installed contract sidecar");
  }
  hash_error.clear();
  const std::string dso_hash = sha256File(dso, &hash_error);
  if (dso_size_error || dso_size != parsed.dso_size_bytes || dso_hash != parsed.dso_sha256) {
    return failure(LoadFailureCode::kContractManifestInvalid,
      "plugin DSO size/SHA256 does not match the installed contract sidecar");
  }
  if (parsed.abi_epoch != kRegistrationAbiEpoch ||
    parsed.toolchain_tag != registrationToolchainTag() ||
    parsed.interface_contract_sha256 != kRegistrationInterfaceContractSha256)
  {
    return failure(LoadFailureCode::kAbiMismatch,
      "registration contract ABI epoch/toolchain/interface identity "
      "is incompatible with this host");
  }
  if (parsed.api_min.major != kHostApiVersion.major ||
    parsed.api_max.major != kHostApiVersion.major ||
    kHostApiVersion.minor < parsed.api_min.minor || kHostApiVersion.minor > parsed.api_max.minor)
  {
    return failure(LoadFailureCode::kApiMismatch,
      "registration contract API range is incompatible with the host API");
  }
  if (registrationContractManifestDigest(parsed) != parsed.manifest_sha256) {
    return failure(LoadFailureCode::kContractManifestInvalid,
      "registration contract manifest digest is stale or forged");
  }
  *manifest = std::move(parsed);
  return LoadFailure{};
}

LoadFailure validateRegistrationRuntimeDescriptor(
  const std::string & requested_class_id,
  const RegistrationRuntimeDescriptor & descriptor,
  const RegistrationContractManifest * manifest,
  const RegistrationRuntimeDescriptor * expected)
{
  if (!descriptor.logicallyComplete()) {
    return failure(LoadFailureCode::kDescriptorMismatch,
      "registration plugin returned an incomplete runtime descriptor");
  }
  const std::uint64_t known_capabilities = 0xffU;
  if (descriptor.api_min.major != descriptor.api_max.major ||
    descriptor.api_min.minor > descriptor.api_max.minor ||
    (descriptor.required_capability_bits & ~known_capabilities) != 0U ||
    (descriptor.optional_capability_bits & ~known_capabilities) != 0U ||
    (descriptor.required_capability_bits & descriptor.optional_capability_bits) != 0U ||
    descriptor.abi_epoch != kRegistrationAbiEpoch ||
    descriptor.toolchain_tag != registrationToolchainTag() ||
    descriptor.interface_contract_sha256 != kRegistrationInterfaceContractSha256 ||
    !isHexSha256(descriptor.config_schema_sha256) ||
    !isHexSha256(descriptor.interface_contract_sha256))
  {
    return failure(LoadFailureCode::kDescriptorMismatch,
      "registration runtime descriptor contains a non-canonical ABI, API, "
      "capability, or schema identity");
  }
  if (descriptor.class_id != requested_class_id) {
    return failure(LoadFailureCode::kDescriptorMismatch,
      "registration runtime descriptor class ID does not match the selected class");
  }
  if (manifest != nullptr && !descriptorMatchesManifest(descriptor, *manifest)) {
    return failure(LoadFailureCode::kDescriptorMismatch,
      "registration runtime descriptor does not exactly match its preflight sidecar");
  }
  if (expected != nullptr && !descriptorMatchesExpected(descriptor, *expected)) {
    return failure(LoadFailureCode::kDescriptorMismatch,
      "registration runtime descriptor does not match the host preflight identity");
  }
  return LoadFailure{};
}

}  // namespace shell
}  // namespace registration
}  // namespace plugins
}  // namespace lidarslam
