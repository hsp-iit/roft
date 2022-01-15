#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include <iostream>
#include <regex>
#include <unordered_map>

#include <Eigen/Dense>

#include <libconfig.h++>

#include <tclap/CmdLine.h>


class ConfigParser
{
public:

    ConfigParser(const int& argc, char** argv, const std::string& file_path = "");

    template <class T>
    void operator()(const std::string& path, T& value)
    {
        std::string libconfig_path = std::regex_replace(path, std::regex("::"), ".");

        try
        {
            cfg_.lookupValue(libconfig_path, value);
        }
        catch(libconfig::SettingNotFoundException &e)
        {
            throw(std::runtime_error("ConfigParser::operator(). Error: cannot find the setting with name " + std::string(e.getPath())));
        }
    }


    template <class T>
    void operator()(const std::string& path, std::vector<T>& array)
    {
        std::string libconfig_path = std::regex_replace(path, std::regex("::"), ".");

        try
        {
            libconfig::Setting& parameters = cfg_.lookup(libconfig_path);

            if ((parameters.getType() == libconfig::Setting::TypeArray))
            {
                for (std::size_t i = 0; i < parameters.getLength(); i++)
                {
                    T value = parameters[i];
                    array.push_back(value);
                }
            }
            else
                throw(std::runtime_error("ConfigParser::operator(). Error: cannot find an array setting with name " + libconfig_path));
        }
        catch(libconfig::SettingNotFoundException &e)
        {
            throw(std::runtime_error("ConfigParser::operator(). Error: cannot find the setting with name " + std::string(e.getPath())));
        }
    }


    template <class T>
    void operator()(const std::string& path, Eigen::Matrix<T, Eigen::Dynamic, 1>& array)
    {
        std::vector<T> array_std;
        operator()<T>(path, array_std);

        Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>> map(array_std.data(), array_std.size());
        array = map;
    }


    struct ParameterInfo
    {
        std::string name;
        libconfig::Setting::Type type;
        libconfig::Setting::Type array_type;
        int array_length;
    };

private:

    void add_cli_argument(const ConfigParser::ParameterInfo& parameter_info, TCLAP::CmdLine& cli);

    void add_cli_arg_from(TCLAP::CmdLine& cli);

    void get_parameters_list(const libconfig::Setting& parent, std::vector<ParameterInfo>& list, const std::string& parent_name = "");

    std::string get_cfg_file_path(const int& argc, char** argv);

    void process_cli_args_bool(const std::unordered_map<std::string, std::unique_ptr<TCLAP::ValueArg<std::string>>>& list);

    void process_cli_args_array(const std::unordered_map<std::string, std::unique_ptr<TCLAP::ValueArg<std::string>>>& list, const libconfig::Setting::Type& type);

    template <class T>
    void process_cli_args(const std::unordered_map<std::string, std::unique_ptr<TCLAP::ValueArg<T>>>& list)
    {
        for (const auto& pair : list)
        {
            std::string libconfig_path = std::regex_replace(pair.first, std::regex("::"), ".");
            if(pair.second->isSet())
            {
                cfg_.lookup(libconfig_path) = pair.second->getValue();
            }
        }
    }

    template <class T>
    void process_cli_args_array(const std::unordered_map<std::string, std::unique_ptr<TCLAP::ValueArg<std::string>>>& list)
    {
        std::regex exp;
        std::string type_description;
        if (std::is_same<T, int>::value)
        {
            type_description = "int";
            exp = std::regex("^([+-]?([0-9]*))+(,[ ]{0,}([+-]?([0-9]*))+)*$");
        }
        else if (std::is_same<T, double>::value)
        {
            type_description = "double";
            exp = std::regex("^([+-]?([0-9]*[.])?[0-9])+(,[ ]{0,}([+-]?([0-9]*[.])?[0-9])+)*$");
        }


        for (const auto& pair : list)
        {
            std::string libconfig_path = std::regex_replace(pair.first, std::regex("::"), ".");
            if(pair.second->isSet())
            {
                std::smatch match;
                if(!std::regex_search(pair.second->getValue(), match, exp))
                    throw(std::runtime_error("Provided values for " + pair.first + " are not valid " + type_description + " numbers."));

                std::stringstream ss(pair.second->getValue());

                std::vector<T> array;
                T number;
                while(ss >> number)
                {
                    array.push_back(number);

                    if (ss.peek() == ',')
                        ss.ignore();
                }

                std::string libconfig_path = std::regex_replace(pair.first, std::regex("::"), ".");
                libconfig::Setting& parameters = cfg_.lookup(libconfig_path);

                if (array.size() != parameters.getLength())
                    throw(std::runtime_error("Expected " + std::to_string(parameters.getLength()) + " values for " + pair.first + "."));

                for (std::size_t i = 0; i < array.size(); i++)
                    parameters[i] = array.at(i);
            }
        }
    }

    libconfig::Config cfg_;

    std::vector<std::string> allowed_booleans_string_ = {"true", "false"};

    TCLAP::ValuesConstraint<std::string> allowed_booleans_;

    std::unique_ptr<TCLAP::ValueArg<std::string>> cli_arg_from_;

    const std::string default_path_;

    /* Plain types. */

    std::unordered_map<std::string, std::unique_ptr<TCLAP::ValueArg<std::string>>> cli_args_bool_;

    std::unordered_map<std::string, std::unique_ptr<TCLAP::ValueArg<double>>> cli_args_double_;

    std::unordered_map<std::string, std::unique_ptr<TCLAP::ValueArg<int>>> cli_args_int_;

    std::unordered_map<std::string, std::unique_ptr<TCLAP::ValueArg<std::string>>> cli_args_string_;

    /* Arrays. */

    std::unordered_map<std::string, std::unique_ptr<TCLAP::ValueArg<std::string>>> cli_args_int_array_;

    std::unordered_map<std::string, std::unique_ptr<TCLAP::ValueArg<std::string>>> cli_args_double_array_;
};

#endif /* CONFIG_PARSER_H */
