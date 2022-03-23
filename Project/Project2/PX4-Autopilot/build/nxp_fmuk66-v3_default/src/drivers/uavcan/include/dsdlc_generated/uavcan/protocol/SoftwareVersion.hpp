/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/ds/Documents/g16/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/protocol/SoftwareVersion.uavcan
 */

#ifndef UAVCAN_PROTOCOL_SOFTWAREVERSION_HPP_INCLUDED
#define UAVCAN_PROTOCOL_SOFTWAREVERSION_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Nested type.
# Generic software version information.
#

#
# Primary version numbers.
# If both fields are set to zero, the version is considered unknown.
#
uint8 major
uint8 minor

#
# This mask indicates which optional fields (see below) are set.
#
uint8 OPTIONAL_FIELD_FLAG_VCS_COMMIT = 1
uint8 OPTIONAL_FIELD_FLAG_IMAGE_CRC  = 2
uint8 optional_field_flags

#
# VCS commit hash or revision number, e.g. git short commit hash. Optional.
#
uint32 vcs_commit

#
# The value of an arbitrary hash function applied to the firmware image.
# This field is used to detect whether the firmware running on the node is EXACTLY THE SAME
# as a certain specific revision. This field provides the absolute identity guarantee, unlike
# the version fields above, which can be the same for different builds of the firmware.
#
# The exact hash function and the methods of its application are implementation defined.
# However, implementations are recommended to adhere to the following guidelines,
# fully or partially:
#
#   - The hash function should be CRC-64-WE, the same that is used for computing DSDL signatures.
#
#   - The hash function should be applied to the entire application image padded to 8 bytes.
#
#   - If the computed image CRC is stored within the firmware image itself, the value of
#     the hash function becomes ill-defined, because it becomes recursively dependent on itself.
#     In order to circumvent this issue, while computing or checking the CRC, its value stored
#     within the image should be zeroed out.
#
uint64 image_crc
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.SoftwareVersion
saturated uint8 major
saturated uint8 minor
saturated uint8 optional_field_flags
saturated uint32 vcs_commit
saturated uint64 image_crc
******************************************************************************/

#undef major
#undef minor
#undef optional_field_flags
#undef vcs_commit
#undef image_crc
#undef OPTIONAL_FIELD_FLAG_VCS_COMMIT
#undef OPTIONAL_FIELD_FLAG_IMAGE_CRC

namespace uavcan
{
namespace protocol
{

template <int _tmpl>
struct UAVCAN_EXPORT SoftwareVersion_
{
    typedef const SoftwareVersion_<_tmpl>& ParameterType;
    typedef SoftwareVersion_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > OPTIONAL_FIELD_FLAG_VCS_COMMIT;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > OPTIONAL_FIELD_FLAG_IMAGE_CRC;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > major;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > minor;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > optional_field_flags;
        typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > vcs_commit;
        typedef ::uavcan::IntegerSpec< 64, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > image_crc;
    };

    enum
    {
        MinBitLen
            = FieldTypes::major::MinBitLen
            + FieldTypes::minor::MinBitLen
            + FieldTypes::optional_field_flags::MinBitLen
            + FieldTypes::vcs_commit::MinBitLen
            + FieldTypes::image_crc::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::major::MaxBitLen
            + FieldTypes::minor::MaxBitLen
            + FieldTypes::optional_field_flags::MaxBitLen
            + FieldTypes::vcs_commit::MaxBitLen
            + FieldTypes::image_crc::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::OPTIONAL_FIELD_FLAG_VCS_COMMIT >::Type OPTIONAL_FIELD_FLAG_VCS_COMMIT; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::OPTIONAL_FIELD_FLAG_IMAGE_CRC >::Type OPTIONAL_FIELD_FLAG_IMAGE_CRC; // 2

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::major >::Type major;
    typename ::uavcan::StorageType< typename FieldTypes::minor >::Type minor;
    typename ::uavcan::StorageType< typename FieldTypes::optional_field_flags >::Type optional_field_flags;
    typename ::uavcan::StorageType< typename FieldTypes::vcs_commit >::Type vcs_commit;
    typename ::uavcan::StorageType< typename FieldTypes::image_crc >::Type image_crc;

    SoftwareVersion_()
        : major()
        , minor()
        , optional_field_flags()
        , vcs_commit()
        , image_crc()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<120 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.SoftwareVersion";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool SoftwareVersion_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        major == rhs.major &&
        minor == rhs.minor &&
        optional_field_flags == rhs.optional_field_flags &&
        vcs_commit == rhs.vcs_commit &&
        image_crc == rhs.image_crc;
}

template <int _tmpl>
bool SoftwareVersion_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(major, rhs.major) &&
        ::uavcan::areClose(minor, rhs.minor) &&
        ::uavcan::areClose(optional_field_flags, rhs.optional_field_flags) &&
        ::uavcan::areClose(vcs_commit, rhs.vcs_commit) &&
        ::uavcan::areClose(image_crc, rhs.image_crc);
}

template <int _tmpl>
int SoftwareVersion_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::major::encode(self.major, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::minor::encode(self.minor, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::optional_field_flags::encode(self.optional_field_flags, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::vcs_commit::encode(self.vcs_commit, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::image_crc::encode(self.image_crc, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int SoftwareVersion_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::major::decode(self.major, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::minor::decode(self.minor, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::optional_field_flags::decode(self.optional_field_flags, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::vcs_commit::decode(self.vcs_commit, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::image_crc::decode(self.image_crc, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature SoftwareVersion_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xDD46FD376527FEA1ULL);

    FieldTypes::major::extendDataTypeSignature(signature);
    FieldTypes::minor::extendDataTypeSignature(signature);
    FieldTypes::optional_field_flags::extendDataTypeSignature(signature);
    FieldTypes::vcs_commit::extendDataTypeSignature(signature);
    FieldTypes::image_crc::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename SoftwareVersion_<_tmpl>::ConstantTypes::OPTIONAL_FIELD_FLAG_VCS_COMMIT >::Type
    SoftwareVersion_<_tmpl>::OPTIONAL_FIELD_FLAG_VCS_COMMIT = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename SoftwareVersion_<_tmpl>::ConstantTypes::OPTIONAL_FIELD_FLAG_IMAGE_CRC >::Type
    SoftwareVersion_<_tmpl>::OPTIONAL_FIELD_FLAG_IMAGE_CRC = 2U; // 2

/*
 * Final typedef
 */
typedef SoftwareVersion_<0> SoftwareVersion;

// No default registration

} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::SoftwareVersion >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::SoftwareVersion::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::SoftwareVersion >::stream(Stream& s, ::uavcan::protocol::SoftwareVersion::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "major: ";
    YamlStreamer< ::uavcan::protocol::SoftwareVersion::FieldTypes::major >::stream(s, obj.major, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "minor: ";
    YamlStreamer< ::uavcan::protocol::SoftwareVersion::FieldTypes::minor >::stream(s, obj.minor, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "optional_field_flags: ";
    YamlStreamer< ::uavcan::protocol::SoftwareVersion::FieldTypes::optional_field_flags >::stream(s, obj.optional_field_flags, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "vcs_commit: ";
    YamlStreamer< ::uavcan::protocol::SoftwareVersion::FieldTypes::vcs_commit >::stream(s, obj.vcs_commit, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "image_crc: ";
    YamlStreamer< ::uavcan::protocol::SoftwareVersion::FieldTypes::image_crc >::stream(s, obj.image_crc, level + 1);
}

}

namespace uavcan
{
namespace protocol
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::SoftwareVersion::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::SoftwareVersion >::stream(s, obj, 0);
    return s;
}

} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_SOFTWAREVERSION_HPP_INCLUDED