#include <iostream>
#include "usmapParser.hpp"
#include <list>

void loadMappings() {
	std::cout << "Parsing ++Fortnite+Release-17.21-CL-16967001-Windows_oo.usmap" << std::endl;
	usmapParser::LoadByPath("C:\\Users\\Joel\\Downloads\\++Fortnite+Release-17.21-CL-16967001-Windows_oo.usmap");
	usmapParser::Parse();
}

void createHeaders() {
	loadMappings();

	// Get the default classes
	std::string UE4classes = R"(
// Standart UE4 classes
struct FString {};
struct FText {};
struct FName {};
struct FDelegate {};
struct FScriptMulticastDelegate {};

// TODO: TArray, TSoftClassPtr
)";

	std::string enums;
	std::string structs = "#include \"Enums.h\"\n\n" + UE4classes + "\n\n// FortniteGame classes\n\n";
	std::string classes = "#include \"Structs.h\"\n";

	std::vector<std::string> UObjects;
	std::vector<std::string> AActors;

	for (int i = 0; i < usmapParser::Schemas.size(); i++) {
		std::string properties;
		bool bIsClass = usmapParser::Schemas[i].SuperType != std::string();

		for (int j = 0; j < usmapParser::Schemas[i].Properties.size(); j++) {
			std::string dataType = getUE4NameFromPropertyTag(usmapParser::Schemas[i].Properties[j].Data);
			properties += "	" + dataType + " " + usmapParser::Schemas[i].Properties[j].Name + ";\n";
		}

		std::string prefix = bIsClass ? "" : "F";
		if (bIsClass) {
			if (usmapParser::Schemas[i].SuperType == "Object") {
				prefix = "U";
				UObjects.emplace_back(usmapParser::Schemas[i].Name);
			}
			else if (usmapParser::Schemas[i].SuperType == "Actor") {
				prefix = "A";
				AActors.emplace_back(usmapParser::Schemas[i].Name);
			}
			else { // Deeper scan
				bool found = false;
				for (int j = 0; j < UObjects.size(); j++) {
					if (UObjects[j] == usmapParser::Schemas[i].SuperType) {
						prefix = "U";
						UObjects.emplace_back(usmapParser::Schemas[i].Name);
						found = true;
						break;
					}
				}

				if (!found) {
					for (int j = 0; j < AActors.size(); j++) {
						if (AActors[j] == usmapParser::Schemas[i].SuperType) {
							prefix = "A";
							AActors.emplace_back(usmapParser::Schemas[i].Name);
							break;
						}
					}
				}
			}
		}

		if (bIsClass) {
			classes += "class " + prefix + usmapParser::Schemas[i].Name + " : public " + prefix + usmapParser::Schemas[i].SuperType + "\n";
			classes += "{\n";
			classes += "public:\n";
			classes += properties;
			classes += "\n	// NOTE: Functions are not included into usmaps sadly :(\n";
			classes += "};\n\n";
		}
		else {
			structs += "struct " + prefix + usmapParser::Schemas[i].Name + "\n";
			structs += "{\n";
			structs += properties;
			structs += "};\n\n";
		}
	}

	for (int i = 0; i < usmapParser::Enums.size(); i++) {
		enums += "enum " + usmapParser::Enums[i].Name + "\n";
		enums += "{\n";
		for (int j = 0; j < usmapParser::Enums[i].ValueNames.size(); j++) {
			enums += "	" + usmapParser::Enums[i].ValueNames[j] + (j != usmapParser::Enums[i].ValueNames.size() - 1 ? "," : "") + "\n";
		}
		enums += "};\n\n";
	}

	std::ofstream file("Classes.h", std::ifstream::ios_base::binary);
	file.write(classes.c_str(), classes.length());

	file = std::ofstream("Structs.h", std::ifstream::ios_base::binary);
	file.write(structs.c_str(), structs.length());

	file = std::ofstream("Enums.h", std::ifstream::ios_base::binary);
	file.write(enums.c_str(), enums.length());
}

bool hasName(std::vector<std::string> Names, std::string Name) {
	for (int i = 0; i < Names.size(); i++) {
		if (Names[i] == Name) return true;
	}

	return false;
}

int main() {
	createHeaders();
	return 0;
}