#include <ConfigParser.h>

#include <regex>
#include <sstream>
#include <vector>


ConfigParser::ConfigParser
(
    const int& argc,
    char** argv,
    const std::string& file_path
) :
    allowed_booleans_(allowed_booleans_string_),
    default_path_(file_path)
{
    std::string cfg_path = get_cfg_file_path(argc, argv);

    try
    {
        cfg_.readFile(cfg_path.c_str());
        const libconfig::Setting& root = cfg_.getRoot();

        std::vector<ParameterInfo> list;
        get_parameters_list(root, list);

        TCLAP::CmdLine cli("");
        add_cli_arg_from(cli);
        for (const auto& parameter : list)
            add_cli_argument(parameter, cli);
        cli.parse(argc, argv);

        process_cli_args_bool(cli_args_bool_);
        process_cli_args(cli_args_double_);
        process_cli_args(cli_args_int_);
        process_cli_args(cli_args_string_);
        process_cli_args_array<double>(cli_args_double_array_);
        process_cli_args_array<int>(cli_args_int_array_);
    }
    catch(const libconfig::FileIOException& e)
    {
        throw(std::runtime_error("ConfigParser::ctor. I/O error while reading " + cfg_path + "."));
    }
    catch(const libconfig::ParseException& e)
    {
        const std::string file_name(e.getFile());
        const std::string file_line(std::to_string(e.getLine()));
        const std::string file_error(e.getError());

        throw(std::runtime_error("ConfigParser::ctor. Parse error at " + file_name + ":" + file_line + " - " + file_error));
    }
}


void ConfigParser::add_cli_argument(const ConfigParser::ParameterInfo& parameter_info, TCLAP::CmdLine& cli)
{
    if (parameter_info.type == libconfig::Setting::TypeArray)
    {
        std::string description{"Specify the array as \"x_1, ..., x_" + std::to_string(parameter_info.array_length) + "\"."};
        std::string type_description{"array"};

        if (parameter_info.array_type == libconfig::Setting::TypeInt)
        {
            type_description += " of int";

            cli_args_int_array_[parameter_info.name] = std::make_unique<TCLAP::ValueArg<std::string>>("", parameter_info.name, description, false, "", type_description);
            cli.add(*cli_args_int_array_.at(parameter_info.name));
        }
        else if (parameter_info.array_type == libconfig::Setting::TypeFloat)
        {
            type_description += " of double";

            cli_args_double_array_[parameter_info.name] = std::make_unique<TCLAP::ValueArg<std::string>>("", parameter_info.name, description, false, "", type_description);
            cli.add(*cli_args_double_array_.at(parameter_info.name));
        }
    }
    else if (parameter_info.type == libconfig::Setting::TypeBoolean)
    {
        cli_args_bool_[parameter_info.name] = std::make_unique<TCLAP::ValueArg<std::string>>("", parameter_info.name, "", false, "", &allowed_booleans_);
        cli.add(*cli_args_bool_.at(parameter_info.name));
    }
    else if (parameter_info.type == libconfig::Setting::TypeFloat)
    {
        cli_args_double_[parameter_info.name] = std::make_unique<TCLAP::ValueArg<double>>("", parameter_info.name, "", false, 0.0, "double");
        cli.add(*cli_args_double_.at(parameter_info.name));
    }
    else if (parameter_info.type == libconfig::Setting::TypeInt)
    {
        cli_args_int_[parameter_info.name] = std::make_unique<TCLAP::ValueArg<int>>("", parameter_info.name, "", false, 0, "int");
        cli.add(*cli_args_int_.at(parameter_info.name));
    }
    else if (parameter_info.type == libconfig::Setting::TypeString)
    {
        cli_args_string_[parameter_info.name] = std::make_unique<TCLAP::ValueArg<std::string>>("", parameter_info.name, "", false, "", "string");
        cli.add(*cli_args_string_.at(parameter_info.name));
    }
}


void ConfigParser::add_cli_arg_from(TCLAP::CmdLine& cli)
{
    cli_arg_from_ = std::make_unique<TCLAP::ValueArg<std::string>>("", "from", "Path to the configuration file. It overrides the path provided to the ConfigParser constructor.", false, "", "string");
    cli.add(*cli_arg_from_);
}


void ConfigParser::get_parameters_list(const libconfig::Setting& parent, std::vector<ConfigParser::ParameterInfo>& list, const std::string& parent_name)
{
    for (const libconfig::Setting& item : parent)
    {
        std::string name = parent_name;

        if (!name.empty())
            name += "::";

        name += item.getName();

        if ((item.getLength() > 0) && (item.getType() != libconfig::Setting::TypeArray))
            get_parameters_list(item, list, name);
        else
        {
            if (item.getType() == libconfig::Setting::TypeArray)
            {
                libconfig::Setting& array_child = item[0];
                list.push_back(ConfigParser::ParameterInfo{name, item.getType(), array_child.getType(), item.getLength()});
            }
            else
                list.push_back(ConfigParser::ParameterInfo{name, item.getType()});
        }
    }
}


std::string ConfigParser::get_cfg_file_path(const int& argc, char** argv)
{
    std::string file_path = default_path_;

    TCLAP::CmdLine cli(" ", ' ', " ", false);
    TCLAP::ValueArg<std::string> arg("", "from", "Path to the configuration file. It overrides the path provided to the ConfigParser constructor.", false, "", "string");

    cli.add(arg);
    cli.setExceptionHandling(false);
    try
    {
        cli.parse(argc, argv);
    }
    catch (TCLAP::CmdLineParseException)
    {}

    if (arg.isSet())
        file_path = arg.getValue();

    return file_path;
}


void ConfigParser::process_cli_args_bool(const std::unordered_map<std::string, std::unique_ptr<TCLAP::ValueArg<std::string>>>& list)
{
    for (const auto& pair : list)
    {
        std::string libconfig_path = std::regex_replace(pair.first, std::regex("::"), ".");
        if(pair.second->isSet())
        {
            cfg_.lookup(libconfig_path) = (pair.second->getValue() == "true");
        }
    }
}
