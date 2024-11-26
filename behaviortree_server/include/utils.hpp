#include <string>

std::string const BoolToString(bool b)
{
  return b ? "true" : "false";
}

std::string GetFullPath(const std::string& _file, const std::string& trees_folder_)
{
    std::string full_name = _file;
    const char* home = getenv("HOME");

    full_name = full_name.front() == '~' ? std::string(home) + full_name.substr(1, full_name.size() - 1) : full_name;
    full_name = full_name.front() == '/' ? full_name : (trees_folder_.back() == '/' ? trees_folder_ : trees_folder_ + "/") + full_name;

    std::size_t pos = full_name.find_last_of('.');
    if (pos == std::string::npos)
        full_name += ".xml";
    else if (full_name.substr(pos) != ".xml")
        full_name += ".xml";

    return full_name;
}

