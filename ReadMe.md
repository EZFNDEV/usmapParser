# usmapParser
You can download Fortnite mappings at https://benbot.app/api/v1/mappings

## Usage
NOTE: You need oo2core_5_win64.dll in your dir
1. Include "usmapParser.hpp"
2. Load a usmap (only oodle compressed are supported atm) using `usmapParser::LoadByPath(YOUR_PATH);`
3. Parse the usmap using `usmapParser::Parse();`
4. You can now access Names, Enums and Schemas

# Names, Enums, Schemas datatype
static std::vector<std::string> Names;
static std::vector<Enum> Enums;
static std::vector<Schema> Schemas;

# Structs

class FPropertyTag {
    public:
        FPropertyTag* InnerType;
        FPropertyTag* ValueType;

        EPropertyType Type;
        std::string StructType;
        std::string EnumName;
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

struct Enum {
    std::string Name;
    std::vector<std::string> ValueNames;
};