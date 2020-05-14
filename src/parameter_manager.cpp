#include "parameter_manager.hpp"

void ParameterManager::readParameterFile(std::string filename) {
  data_ = toml::parse(filename);
}

bool ParameterManager::checkExistanceTable(std::string table_name) {
  bool existence = data_.find(table_name) == data_.end() ? false : true;
  return existence;
}

int ParameterManager::readIntData(std::string table, std::string key) {
  const auto tab = toml::get<toml::Table>(data_.at(table));
  int readData = toml::get<int>(tab.at(key));

  return readData;
}

bool ParameterManager::readBoolData(std::string table, std::string key) {
  const auto tab = toml::get<toml::Table>(data_.at(table));
  bool readData = toml::get<bool>(tab.at(key));

  return readData;
}

float ParameterManager::readFloatData(std::string table, std::string key) {
  const auto tab = toml::get<toml::Table>(data_.at(table));
  float readData = toml::get<float>(tab.at(key));

  return readData;
}

double ParameterManager::readDoubleData(std::string table, std::string key) {
  const auto tab = toml::get<toml::Table>(data_.at(table));
  double readData = toml::get<double>(tab.at(key));

  return readData;
}

std::string ParameterManager::readStringData(std::string table,
                                             std::string key) {
  const auto tab = toml::get<toml::Table>(data_.at(table));
  std::string readData = toml::get<std::string>(tab.at(key));

  return readData;
}
