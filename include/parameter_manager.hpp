#pragma once
#include <iostream>
#include "toml.hpp"

class ParameterManager {
 private:
  toml::table data_;

 public:
  ParameterManager(std::string filename) { readParameterFile(filename); }

  bool checkExistanceTable(std::string table_name);
  void readParameterFile(std::string filename);

  int readIntData(std::string table, std::string key);
  bool readBoolData(std::string table, std::string key);
  float readFloatData(std::string table, std::string key);
  double readDoubleData(std::string table, std::string key);
  std::string readStringData(std::string table, std::string key);
};
