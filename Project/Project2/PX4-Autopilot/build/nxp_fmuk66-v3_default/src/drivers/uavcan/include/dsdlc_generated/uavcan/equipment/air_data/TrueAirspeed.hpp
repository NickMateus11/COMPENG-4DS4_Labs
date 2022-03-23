/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/ds/Documents/g16/PX4-Autopilot/src/drivers/uavcan/libuavcan/dsdl/uavcan/equipment/air_data/1020.TrueAirspeed.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_AIR_DATA_TRUEAIRSPEED_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_AIR_DATA_TRUEAIRSPEED_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# TAS.
#

float16 true_airspeed           # m/s
float16 true_airspeed_variance  # (m/s)^2
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.air_data.TrueAirspeed
saturated float16 true_airspeed
saturated float16 true_airspeed_variance
******************************************************************************/

#undef true_airspeed
#undef true_airspeed_variance

namespace uavcan
{
namespace equipment
{
namespace air_data
{

template <int _tmpl>
struct UAVCAN_EXPORT TrueAirspeed_
{
    typedef const TrueAirspeed_<_tmpl>& ParameterType;
    typedef TrueAirspeed_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > true_airspeed;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > true_airspeed_variance;
    };

    enum
    {
        MinBitLen
            = FieldTypes::true_airspeed::MinBitLen
            + FieldTypes::true_airspeed_variance::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::true_airspeed::MaxBitLen
            + FieldTypes::true_airspeed_variance::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::true_airspeed >::Type true_airspeed;
    typename ::uavcan::StorageType< typename FieldTypes::true_airspeed_variance >::Type true_airspeed_variance;

    TrueAirspeed_()
        : true_airspeed()
        , true_airspeed_variance()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<32 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1020 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.air_data.TrueAirspeed";
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
bool TrueAirspeed_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        true_airspeed == rhs.true_airspeed &&
        true_airspeed_variance == rhs.true_airspeed_variance;
}

template <int _tmpl>
bool TrueAirspeed_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(true_airspeed, rhs.true_airspeed) &&
        ::uavcan::areClose(true_airspeed_variance, rhs.true_airspeed_variance);
}

template <int _tmpl>
int TrueAirspeed_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::true_airspeed::encode(self.true_airspeed, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::true_airspeed_variance::encode(self.true_airspeed_variance, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int TrueAirspeed_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::true_airspeed::decode(self.true_airspeed, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::true_airspeed_variance::decode(self.true_airspeed_variance, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature TrueAirspeed_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x306F69E0A591AFAAULL);

    FieldTypes::true_airspeed::extendDataTypeSignature(signature);
    FieldTypes::true_airspeed_variance::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef TrueAirspeed_<0> TrueAirspeed;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::air_data::TrueAirspeed > _uavcan_gdtr_registrator_TrueAirspeed;

}

} // Namespace air_data
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::air_data::TrueAirspeed >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::air_data::TrueAirspeed::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::air_data::TrueAirspeed >::stream(Stream& s, ::uavcan::equipment::air_data::TrueAirspeed::ParameterType obj, const int level)
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
    s << "true_airspeed: ";
    YamlStreamer< ::uavcan::equipment::air_data::TrueAirspeed::FieldTypes::true_airspeed >::stream(s, obj.true_airspeed, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "true_airspeed_variance: ";
    YamlStreamer< ::uavcan::equipment::air_data::TrueAirspeed::FieldTypes::true_airspeed_variance >::stream(s, obj.true_airspeed_variance, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace air_data
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::air_data::TrueAirspeed::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::air_data::TrueAirspeed >::stream(s, obj, 0);
    return s;
}

} // Namespace air_data
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_AIR_DATA_TRUEAIRSPEED_HPP_INCLUDED