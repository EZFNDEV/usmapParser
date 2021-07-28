#include <windows.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// TODO: Linoodle support or custom implementation...

#define USMAP_MAGIC 0x30C4

/* Enums */
enum class Version {
    INITIAL,

    LATEST_PLUS_ONE,
    LATEST = LATEST_PLUS_ONE - 1
};

enum class ECompressionMethod {
    None,
    Oodle,
    Brotli,

    Unknown = 0xFF
};

enum class EPropertyType : uint8_t {
    ByteProperty,
    BoolProperty,
    IntProperty,
    FloatProperty,
    ObjectProperty,
    NameProperty,
    DelegateProperty,
    DoubleProperty,
    ArrayProperty,
    StructProperty,
    StrProperty,
    TextProperty,
    InterfaceProperty,
    MulticastDelegateProperty,
    WeakObjectProperty, //
    LazyObjectProperty, // When deserialized, these 3 properties will be SoftObjects
    AssetObjectProperty, //
    SoftObjectProperty,
    UInt64Property,
    UInt32Property,
    UInt16Property,
    Int64Property,
    Int16Property,
    Int8Property,
    MapProperty,
    SetProperty,
    EnumProperty,
    FieldPathProperty,

    Unknown = 0xFF
};

namespace Oodle {
    // Typedefs
    typedef int8_t OO_S8;
    typedef uint8_t OO_U8;
    typedef int16_t OO_S16;
    typedef uint16_t OO_U16;
    typedef int32_t OO_S32;
    typedef uint32_t OO_U32;
    typedef int64_t OO_S64;
    typedef uint64_t OO_U64;
    typedef float OO_F32;
    typedef double OO_F64;
    typedef intptr_t OO_SINTa;
    typedef uintptr_t OO_UINTa;
    typedef int32_t OO_BOOL;

    typedef enum OodleLZ_FuzzSafe
    {
        OodleLZ_FuzzSafe_No = 0,
        OodleLZ_FuzzSafe_Yes = 1
    } OodleLZ_FuzzSafe;
    /* OodleLZ_FuzzSafe (deprecated)

        About fuzz safety:

        Fuzz Safe decodes will not crash on corrupt data.  They may or may not return failure, and produce garbage output.

        Fuzz safe decodes will not read out of bounds.  They won't put data on the stack or previously in memory
        into the output buffer.

        As of Oodle 2.9.0 all compressors supported are fuzzsafe, so OodleLZ_FuzzSafe_Yes should always be used and this
        enum is deprecated.

    */

    typedef enum OodleLZ_CheckCRC
    {
        OodleLZ_CheckCRC_No = 0,
        OodleLZ_CheckCRC_Yes = 1,
        OodleLZ_CheckCRC_Force32 = 0x40000000
    } OodleLZ_CheckCRC;
    /* Bool enum for the LZ decoder - should it check CRC before decoding or not?

        NOTE : the CRC's in the LZH decompress checks are the CRC's of the *compressed* bytes.  This allows checking the CRc
        prior to decompression, so corrupted data cannot be fed to the compressor.

        To use OodleLZ_CheckCRC_Yes, the compressed data must have been made with $(OodleLZ_CompressOptions:sendQuantumCRCs) set to true.

        If you want a CRC of the raw bytes, there is one optionally stored in the $OodleLZ_SeekTable and can be confirmed with
        $OodleLZ_CheckSeekTableCRCs
    */

    // Default verbosity selection of 0 will not even log when it sees corruption
    typedef enum OodleLZ_Verbosity
    {
        OodleLZ_Verbosity_None = 0,
        OodleLZ_Verbosity_Minimal = 1,
        OodleLZ_Verbosity_Some = 2,
        OodleLZ_Verbosity_Lots = 3,
        OodleLZ_Verbosity_Force32 = 0x40000000
    } OodleLZ_Verbosity;
    /* Verbosity of LZ functions
        LZ functions print information to the function set by $OodleCore_Plugins_SetPrintf
        or $OodleXLog_Printf if using OodleX.
    */

    typedef enum OodleDecompressCallbackRet
    {
        OodleDecompressCallbackRet_Continue = 0,
        OodleDecompressCallbackRet_Cancel = 1,
        OodleDecompressCallbackRet_Invalid = 2,
        OodleDecompressCallbackRet_Force32 = 0x40000000
    } OodleDecompressCallbackRet;
    /* Return value for $OodleDecompressCallback
        return OodleDecompressCallbackRet_Cancel to abort the in-progress decompression
    */

    typedef enum OodleLZ_Decode_ThreadPhase
    {
        OodleLZ_Decode_ThreadPhase1 = 1,
        OodleLZ_Decode_ThreadPhase2 = 2,
        OodleLZ_Decode_ThreadPhaseAll = 3,
        OodleLZ_Decode_Unthreaded = OodleLZ_Decode_ThreadPhaseAll
    } OodleLZ_Decode_ThreadPhase;
    /* ThreadPhase for threaded Oodle decode

        Check $OodleLZ_Compressor_CanDecodeThreadPhased
        (currently only used by Kraken)

        See $OodleLZ_About_ThreadPhasedDecode

    */

    // Decompress returns raw (decompressed) len received
    // Decompress returns 0 (OODLELZ_FAILED) if it detects corruption
    typedef OO_SINTa(__fastcall* OodleLZ_DecompressFunc) (const void* compBuf, OO_SINTa compBufSize, void* rawBuf, OO_SINTa rawLen,
        OodleLZ_FuzzSafe fuzzSafe,
        OodleLZ_CheckCRC checkCRC,
        OodleLZ_Verbosity verbosity,
        void* decBufBase,
        OO_SINTa decBufSize,
        OodleDecompressCallbackRet* fpCallback,
        void* callbackUserData,
        void* decoderMemory,
        OO_SINTa decoderMemorySize,
        OodleLZ_Decode_ThreadPhase threadPhase
        );
    /* Decompress a some data from memory to memory, synchronously.

        $:compBuf       pointer to compressed data
        $:compBufSize   number of compressed bytes available (must be greater or equal to the number consumed)
        $:rawBuf        pointer to output uncompressed data into
        $:rawLen        number of uncompressed bytes to output
        $:fuzzSafe      (optional) should the decode fail if it contains non-fuzz safe codecs?
        $:checkCRC      (optional) if data could be corrupted and you want to know about it, pass OodleLZ_CheckCRC_Yes
        $:verbosity     (optional) if not OodleLZ_Verbosity_None, logs some info
        $:decBufBase    (optional) if not NULL, provides preceding data to prime the dictionary; must be contiguous with rawBuf, the data between the pointers _dictionaryBase_ and _rawBuf_ is used as the preconditioning data.   The exact same precondition must be passed to encoder and decoder.  The decBufBase must be a reset point.
        $:decBufSize    (optional) size of decode buffer starting at decBufBase, if 0, _rawLen_ is assumed
        $:fpCallback    (optional) OodleDecompressCallback to call incrementally as decode proceeds
        $:callbackUserData (optional) passed as userData to fpCallback
        $:decoderMemory (optional) pre-allocated memory for the Decoder, of size _decoderMemorySize_
        $:decoderMemorySize (optional) size of the buffer at _decoderMemory_; must be at least $OodleLZDecoder_MemorySizeNeeded bytes to be used
        $:threadPhase   (optional) for threaded decode; see $OodleLZ_About_ThreadPhasedDecode (default OodleLZ_Decode_Unthreaded)
        $:return        the number of decompressed bytes output, $OODLELZ_FAILED (0) if none can be decompressed

        Decodes data encoded with any $OodleLZ_Compressor.

        Note : _rawLen_ must be the actual number of bytes to output, the same as the number that were encoded with the corresponding
        OodleLZ_Compress size.  You must store this somewhere in your own header and pass it in to this call.  _compBufSize_ does NOT
        need to be the exact number of compressed bytes, is the number of bytes available in the buffer, it must be greater or equal to
        the actual compressed length.

        Note that the new compressors (Kraken,Mermaid,Selkie,BitKnit) are all fuzz safe and you can use OodleLZ_FuzzSafe_Yes
        with them and no padding of the decode target buffer.

        If checkCRC is OodleLZ_CheckCRC_Yes, then corrupt data will be detected and the decode aborted.
        If checkCRC is OodleLZ_CheckCRC_No, then corruption might result in invalid data, but no detection of any error (garbage in, garbage out).

        If corruption is possible, _fuzzSafe_ is No and _checkCRC_ is OodleLZ_CheckCRC_No, $OodleLZ_GetDecodeBufferSize must be used to allocate
        _rawBuf_ large enough to prevent overrun.

        $OodleLZ_GetDecodeBufferSize should always be used to ensure _rawBuf_ is large enough, even when corruption is not
        possible (when fuzzSafe is No).

        _compBuf_ and _rawBuf_ are allowed to overlap for "in place" decoding, but then _rawBuf_ must be allocated to
        the size given by $OodleLZ_GetInPlaceDecodeBufferSize , and the compressed data must be at the end of that buffer.

        An easy way to take the next step to parallel decoding is with $OodleXLZ_Decompress_MakeSeekTable_Wide_Async (in the Oodle2 Ext lib)

        NOTE : the return value is the *total* number of decompressed bytes output so far.  If rawBuf is > decBufBase, that means
        the initial inset of (rawBuf - decBufBase) is included!  (eg. you won't just get _rawLen_)

        If _decBufBase_ is provided, the backup distance from _rawBuf_ must be a multiple of $OODLELZ_BLOCK_LEN

        About fuzz safety:

        OodleLZ_Decompress is guaranteed not to crash even if the data is corrupted when _fuzzSafe_ is set to OodleLZ_FuzzSafe_Yes.
        When _fuzzSafe_ is Yes, the target buffer (_rawBuf_ and _rawLen_) will never be overrun.  Note that corrupted day might not
        be detected (the return value might indicate success).

        Fuzz Safe decodes will not crash on corrupt data.  They may or may not return failure, and produce garbage output.

        Fuzz safe decodes will not read out of bounds.  They won't put data on the stack or previously in memory
        into the output buffer.

        Fuzz safe decodes will not output more than the uncompressed size. (eg. the output buffer does not need to
        be padded like OodleLZ_GetDecodeBufferSize)

        If you ask for a fuzz safe decode and the compressor doesn't satisfy OodleLZ_Compressor_CanDecodeFuzzSafe
        then it will return failure.

        The _fuzzSafe_ argument should always be OodleLZ_FuzzSafe_Yes as of Oodle 2.9.0 ; older compressors did not
        support fuzz safety but they now all do.

        Use of OodleLZ_FuzzSafe_No is deprecated.

    */

    static bool Initialized = false;
    static HMODULE OodleLibary = LoadLibraryA("oo2core_5_win64.dll");
    static OodleLZ_DecompressFunc OodleLZ_Decompress;

    void Initialize() {
        if (Initialized) return;
        OodleLZ_Decompress = (OodleLZ_DecompressFunc)GetProcAddress(OodleLibary, "OodleLZ_Decompress");

        Initialized = true;
    }

    void Decompress(const void* compBuf, OO_SINTa compBufSize, void* rawBuf, OO_SINTa rawLen,
        OodleLZ_FuzzSafe fuzzSafe = OodleLZ_FuzzSafe_Yes,
        OodleLZ_CheckCRC checkCRC = OodleLZ_CheckCRC_No,
        OodleLZ_Verbosity verbosity = OodleLZ_Verbosity_None,
        void* decBufBase = NULL,
        OO_SINTa decBufSize = 0,
        OodleDecompressCallbackRet* fpCallback = NULL,
        void* callbackUserData = NULL,
        void* decoderMemory = NULL,
        OO_SINTa decoderMemorySize = 0,
        OodleLZ_Decode_ThreadPhase threadPhase = OodleLZ_Decode_Unthreaded
    ) {
        Initialize();

        OO_SINTa DecompSize = OodleLZ_Decompress(compBuf, compBufSize, rawBuf, rawLen, fuzzSafe, checkCRC, verbosity, decBufBase, decBufSize, fpCallback, callbackUserData, decoderMemory, decoderMemorySize, threadPhase);
    }
}

namespace usmapParser {
    static uint8_t* usmapData;
    static std::vector<std::string> Names;

    struct Enum {
        std::string Name;
        std::vector<std::string> ValueNames;
    };
    static std::vector<Enum> Enums;

    class FPropertyTag {
        public:
            FPropertyTag* InnerType;
            FPropertyTag* ValueType;

            EPropertyType Type;
            std::string StructType;
            std::string EnumName;
            uint32_t EndPosition = 0;
        public:
            FPropertyTag() {};

            FPropertyTag(uint8_t* data) {
                Type = (EPropertyType)data[0];
                EndPosition += 1;

                FPropertyTag tempType;
                FPropertyTag* innerType = new FPropertyTag();
                FPropertyTag* valueType = new FPropertyTag();

                uint32_t NameIdx;

                switch (Type) {
                    case EPropertyType::EnumProperty:
                        tempType = FPropertyTag(data + 1);
                        innerType->EndPosition = tempType.EndPosition;
                        innerType->EnumName = tempType.EnumName;
                        innerType->InnerType = tempType.InnerType;
                        innerType->StructType = tempType.StructType;
                        innerType->Type = tempType.Type;
                        innerType->ValueType = tempType.ValueType;

                        NameIdx = *(uint32_t*)&data[innerType->EndPosition+1];
                        EndPosition += 4;
                        EnumName = Names[NameIdx];
                        break;
                    case EPropertyType::StructProperty:
                        NameIdx = *(uint32_t*)&data[1];
                        EndPosition += 4;
                        StructType = Names[NameIdx];
                        break;
                    case EPropertyType::SetProperty:
                    case EPropertyType::ArrayProperty:
                        tempType = FPropertyTag(data + 1);
                        innerType->EndPosition = tempType.EndPosition;
                        innerType->EnumName = tempType.EnumName;
                        innerType->InnerType = tempType.InnerType;
                        innerType->StructType = tempType.StructType;
                        innerType->Type = tempType.Type;
                        innerType->ValueType = tempType.ValueType;
                        break;
                    case EPropertyType::MapProperty:
                        tempType = FPropertyTag(data + 1);
                        innerType->EndPosition = tempType.EndPosition;
                        innerType->EnumName = tempType.EnumName;
                        innerType->InnerType = tempType.InnerType;
                        innerType->StructType = tempType.StructType;
                        innerType->Type = tempType.Type;
                        innerType->ValueType = tempType.ValueType;

                        tempType = FPropertyTag(data + innerType->EndPosition + 1);
                        valueType->EndPosition = tempType.EndPosition;
                        valueType->EnumName = tempType.EnumName;
                        valueType->InnerType = tempType.InnerType;
                        valueType->StructType = tempType.StructType;
                        valueType->Type = tempType.Type;
                        valueType->ValueType = tempType.ValueType;
                        break;
                }

                InnerType = innerType;
                ValueType = valueType;

                EndPosition += innerType->EndPosition + valueType->EndPosition;
            }
    };

    struct Property {
        std::string Name;
        uint16_t SchemaIdx;
        uint8_t ArraySize;
        FPropertyTag Data;

    };

    struct Schema {
        std::vector<Property> Properties;
        std::string Name;
        std::string SuperType;
        uint16_t PropCount;
    };
    static std::vector<Schema> Schemas;

    void LoadByPath(std::string path) {
        Names.clear();
        Enums.clear();
        Schemas.clear();

        std::ifstream file(path, std::ifstream::ios_base::binary);

        file.seekg(0, file.end);
        size_t length = file.tellg();
        file.seekg(0, file.beg);

        usmapData = new uint8_t[length];
        file.read((char*)&usmapData[0], length);
    }

    void LoadByNetCL() {
        // TODO: Make a list of netcls or use my own api
    }

    void LoadByBuildVersion() {

    }

    void Parse() {
        if (usmapData[0] != 0xC4 || usmapData[1] != 0x30) { // Magic
            printf(".usmap file has an invalid magic constant\n");
            // throw InvalidMagicException(".usmap file has an invalid magic constant");
        }

        if (usmapData[2] != (uint8_t)Version::LATEST) {
            // throw InvalidVersionException(".usmap file has invalid version %d", (int)Version);
        }

        uint32_t compressedSize = *(uint32_t*)&usmapData[4];
        uint32_t decompressedSize = *(uint32_t*)&usmapData[8];

        /* if (CompressedInputStream.size() - CompressedInputStream.tell() < CompSize) {
             throw ArchiveCorruptedException("There is not enough data in the .usmap file");
         }*/

        uint8_t* decompressedData = new uint8_t[decompressedSize];

        switch ((ECompressionMethod)usmapData[3]) { // Method
            case ECompressionMethod::None:
                if (compressedSize != decompressedSize) {
                    // throw DecompressionException("No compression: Compression size must be equal to decompression size");
                }
                decompressedData = usmapData + 7;
            case ECompressionMethod::Oodle:
            {
                Oodle::Decompress(usmapData + 12, compressedSize, decompressedData, decompressedSize);
            }
            /*case ECompressionMethod::Brotli:
            {
                
            }*/
            // default:
                // throw DecompressionException("Unknown compression method: %d", (int)Method);
        }

        // Names
        uint32_t arrayLength = *(uint32_t*)&decompressedData[0];
        int pos = 4;
        for (uint32_t i = 0; i < (arrayLength); i++) {
            uint8_t NameSize = decompressedData[pos];

            char* Name = new char[NameSize + 1];
            memcpy(Name, &decompressedData[pos + 1], NameSize);
            Name[NameSize] = '\0';

            Names.emplace_back(std::string(Name));
            pos += NameSize + 1;
        }

        // Enums
        arrayLength = *(uint32_t*)&decompressedData[pos];
        pos += 4;
        for (uint32_t i = 0; i < (arrayLength); i++) {
            uint32_t NameIdx = *(uint32_t*)&decompressedData[pos];

            uint8_t EnumValuesCount = *(uint8_t*)&decompressedData[pos + 4];
            std::vector<std::string> EnumValueNames;
            for (uint32_t j = 0; j < EnumValuesCount; j++) {
                uint32_t EnumValueNameIdx = *(uint32_t*)&decompressedData[pos + 5 + j * 4];
                EnumValueNames.emplace_back(Names[EnumValueNameIdx]);
            }

            Enum FNEnum;
            FNEnum.Name = Names[NameIdx];
            FNEnum.ValueNames = EnumValueNames;

            Enums.emplace_back(FNEnum);

            pos += 5 + EnumValuesCount * 4;
        }

        // Schemas
        arrayLength = *(uint32_t*)&decompressedData[pos];
        pos += 4;
        for (uint32_t i = 0; i < (arrayLength); i++) {
            uint32_t NameIdx = *(uint32_t*)&decompressedData[pos];

            uint32_t SuperIdx = *(uint32_t*)&decompressedData[pos + 4];
            uint16_t PropCount = *(uint16_t*)&decompressedData[pos + 8];
            uint16_t SerializablePropCount = *(uint16_t*)&decompressedData[pos + 10];

            pos += 12;

            std::vector<Property> Properties;
            for (uint16_t j = 0; j < SerializablePropCount; ++j) {
                uint16_t SchemaIdx = *(uint16_t*)&decompressedData[pos];
                uint8_t ArraySize = *(uint8_t*)&decompressedData[pos + 2];
                uint32_t PropNameIdx = *(uint32_t*)&decompressedData[pos + 3];

                Property property;
                property.Name = Names[PropNameIdx];
                property.SchemaIdx = SchemaIdx;
                property.ArraySize = ArraySize;
                FPropertyTag tag = FPropertyTag(decompressedData + pos + 7);
                property.Data = tag;
                pos += tag.EndPosition + 7;
                Properties.emplace_back(property);
            }

            Schema FNSchema;
            FNSchema.Properties = Properties;
            FNSchema.Name = Names[NameIdx];
            FNSchema.PropCount = PropCount;
            if (SuperIdx < Names.size() - 1) {
                FNSchema.SuperType = Names[SuperIdx];
            }
            Schemas.emplace_back(FNSchema);
        }
    }
}

/* Some utils */

std::string getNativeDataTypeFromEPropertyType(EPropertyType Type) {
    switch (Type) {
        case EPropertyType::ByteProperty:
            return "byte";
        case EPropertyType::BoolProperty:
            return "bool";
        case EPropertyType::IntProperty:
            return "int";
        case EPropertyType::FloatProperty:
            return "float";
        /*case EPropertyType::ObjectProperty:
            return "ObjectProperty";
        case EPropertyType::NameProperty:
            return "NameProperty";
        case EPropertyType::DelegateProperty:
            return "DelegateProperty";*/
        case EPropertyType::DoubleProperty:
            return "double";
        /*case EPropertyType::ArrayProperty:
            return "ArrayProperty";
        case EPropertyType::StructProperty:
            return "StructProperty";
        case EPropertyType::StrProperty:
            return "StrProperty";
        case EPropertyType::TextProperty:
            return "TextProperty";
        case EPropertyType::InterfaceProperty:
            return "InterfaceProperty";
        case EPropertyType::MulticastDelegateProperty:
            return "MulticastDelegateProperty";
        case EPropertyType::WeakObjectProperty:
            return "WeakObjectProperty";
        case EPropertyType::LazyObjectProperty:
            return "LazyObjectProperty";
        case EPropertyType::AssetObjectProperty:
            return "AssetObjectProperty";
        case EPropertyType::SoftObjectProperty:
            return "SoftObjectProperty";*/
        case EPropertyType::UInt64Property:
            return "uint64_t";
        case EPropertyType::UInt32Property:
            return "uint32_t";
        case EPropertyType::UInt16Property:
            return "uint16_t";
        case EPropertyType::Int64Property:
            return "int64_t";
        case EPropertyType::Int16Property:
            return "int16_t";
        case EPropertyType::Int8Property:
            return "int8_t";
        /*case EPropertyType::MapProperty:
            return "MapProperty";
        case EPropertyType::SetProperty:
            return "SetProperty";
        case EPropertyType::EnumProperty:
            return "EnumProperty";
        case EPropertyType::FieldPathProperty:
            return "FieldPathProperty";*/
    }

    return "Unknown";
}

std::string getNameFromEPropertyType(EPropertyType Type) {
    switch (Type) {
        case EPropertyType::ByteProperty:
            return "ByteProperty";
        case EPropertyType::BoolProperty:
            return "BoolProperty";
        case EPropertyType::IntProperty:
            return "IntProperty";
        case EPropertyType::FloatProperty:
            return "FloatProperty";
        case EPropertyType::ObjectProperty:
            return "ObjectProperty";
        case EPropertyType::NameProperty:
            return "NameProperty";
        case EPropertyType::DelegateProperty:
            return "DelegateProperty";
        case EPropertyType::DoubleProperty:
            return "DoubleProperty";
        case EPropertyType::ArrayProperty:
            return "ArrayProperty";
        case EPropertyType::StructProperty:
            return "StructProperty";
        case EPropertyType::StrProperty:
            return "StrProperty";
        case EPropertyType::TextProperty:
            return "TextProperty";
        case EPropertyType::InterfaceProperty:
            return "InterfaceProperty";
        case EPropertyType::MulticastDelegateProperty:
            return "MulticastDelegateProperty";
        case EPropertyType::WeakObjectProperty:
            return "WeakObjectProperty";
        case EPropertyType::LazyObjectProperty:
            return "LazyObjectProperty";
        case EPropertyType::AssetObjectProperty:
            return "AssetObjectProperty";
        case EPropertyType::SoftObjectProperty:
            return "SoftObjectProperty";
        case EPropertyType::UInt64Property:
            return "UInt64Property";
        case EPropertyType::UInt32Property:
            return "UInt32Property";
        case EPropertyType::UInt16Property:
            return "UInt16Property";
        case EPropertyType::Int64Property:
            return "Int64Property";
        case EPropertyType::Int16Property:
            return "Int16Property";
        case EPropertyType::Int8Property:
            return "Int8Property";
        case EPropertyType::MapProperty:
            return "MapProperty";
        case EPropertyType::SetProperty:
            return "SetProperty";
        case EPropertyType::EnumProperty:
            return "EnumProperty";
        case EPropertyType::FieldPathProperty:
            return "FieldPathProperty";
    }

    return "Unknown";
}

std::string getUE4NameFromPropertyTag(usmapParser::FPropertyTag PropertyTag) {
    std::string dataType = getNativeDataTypeFromEPropertyType(PropertyTag.Type);
    if (dataType == "Unknown") {
        switch (PropertyTag.Type) {
            case EPropertyType::ObjectProperty:
                dataType = "class UClass*";
                break;
            case EPropertyType::NameProperty:
                dataType = "struct FName";
                break;
            case EPropertyType::DelegateProperty:
                dataType = "struct FDelegate";
                break;
            case EPropertyType::ArrayProperty:
                dataType = "TArray<" + getUE4NameFromPropertyTag(*PropertyTag.InnerType) + ">";
                break;
            case EPropertyType::StructProperty:
                dataType = "struct " + PropertyTag.StructType;
                break;
            case EPropertyType::StrProperty:
                dataType = "struct FString";
                break;
            case EPropertyType::TextProperty:
                dataType = "struct FText";
                break;
            /*case EPropertyType::InterfaceProperty:
                return "InterfaceProperty";
            */case EPropertyType::MulticastDelegateProperty:
                dataType = "struct FScriptMulticastDelegate";
                break;
            /*case EPropertyType::WeakObjectProperty:
                return "WeakObjectProperty";
            case EPropertyType::LazyObjectProperty:
                return "LazyObjectProperty";
            case EPropertyType::AssetObjectProperty:
                return "AssetObjectProperty";
            */
            case EPropertyType::SoftObjectProperty:
                dataType = "struct TSoftClassPtr<UObject>";
                break;
             /*case EPropertyType::MapProperty:
                    return "MapProperty";
             */
            case EPropertyType::SetProperty:
                // dataType = "struct TSet<" + getUE4NameFromPropertyTag(*PropertyTag.InnerType) + ">";
                break;
            case EPropertyType::EnumProperty:
                dataType = PropertyTag.EnumName;
                break;
            /*case EPropertyType::FieldPathProperty:
                return "FieldPathProperty";
            */
            default:
                // printf("Can't get Name from Nnt implemented Type: %s\n", getNameFromEPropertyType(PropertyTag.Type).c_str());
                break;
        }
    }

    return dataType;
}