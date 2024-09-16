#ifndef BEHAVIOR_TREE_ROS_DESERIALIZATION
#define BEHAVIOR_TREE_ROS_DESERIALIZATION

#include <functional>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>

#include "behavior_tree_ros/3rdparty/nlohmann/json.hpp"
#include "behavior_tree_ros/utils/UnorderedMap.hpp"

namespace BT_ROS
{
namespace deserialization
{
    using Json = nlohmann::json;
    using DeserializeFieldFunction = std::function<void(BT::ActionNodeBase&, const std::string&, const Json&)>;

    void deserializeFieldFromSignedInt(BT::ActionNodeBase& _node, const std::string& _port, const Json& _field)
    {
        const auto port_info = _node.getPortInfo(_port);
        
        bool match_not_found = false; 

        if(port_info.has_value() && !port_info.value().missingTypeInfo())
        {
            const std::type_info* type_info = port_info.value().type();
            
            if (*type_info == typeid(short)) // shall match int16_t
                _node.setOutput<short>(_port, _field.get<short>());
            else if(*type_info == typeid(int)) // shall match int32_t
                _node.setOutput<int>(_port, _field.get<int>());
            else if(*type_info == typeid(long)) // shall match int64_t
                _node.setOutput<long>(_port, _field.get<long>());
            else 
                match_not_found = true; //perform setOutput to max int, i.e. int64
            
        }
        else
            match_not_found = true;
        
        if(match_not_found)
        {
            // Don't have enough info (shall not happen in a good design tree)
            _node.setOutput<int64_t>(_port, _field.get<int64_t>()); // just jump to the highest int
        }

    
    }

    void deserializeFieldFromUnsignedInt(BT::ActionNodeBase& _node, const std::string& _port, const Json& _field)
    {
        const auto port_info = _node.getPortInfo(_port);
        
        bool match_not_found = false; 

        if(port_info.has_value() && !port_info.value().missingTypeInfo())
        {
            const std::type_info* type_info = port_info.value().type();

            // Retrieve the value as uint64_t (maximum unsigned size)
            uint64_t value = _field.get<uint64_t>();
            
            // Check for unsigned types
            if (*type_info == typeid(unsigned short) && value <= std::numeric_limits<unsigned short>::max()) // shall match uint16_t
                _node.setOutput<unsigned short>(_port, static_cast<unsigned short>(value));
            else if (*type_info == typeid(unsigned int) && value <= std::numeric_limits<unsigned int>::max()) // shall match uint32_t
                    _node.setOutput<unsigned int>(_port, static_cast<unsigned int>(value));
            else if (*type_info == typeid(unsigned long) && value <= std::numeric_limits<unsigned long>::max()) // shall match uint64_t
                    _node.setOutput<unsigned long>(_port, static_cast<unsigned long>(value));
            
            // Check for signed types
            else if (*type_info == typeid(short) && value <= static_cast<uint64_t>(std::numeric_limits<short>::max())) // signed short
                    _node.setOutput<short>(_port, static_cast<short>(value));
            else if (*type_info == typeid(int) && value <= static_cast<uint64_t>(std::numeric_limits<int>::max())) // signed int
                    _node.setOutput<int>(_port, static_cast<int>(value));

            else if (*type_info == typeid(long) && value <= static_cast<uint64_t>(std::numeric_limits<long>::max())) // signed long
                    _node.setOutput<long>(_port, static_cast<long>(value));

            else 
                match_not_found = true; //perform setOutput to max int, i.e. uint64
        }
        else
            match_not_found = true;
        
        if(match_not_found)
        {
            // Don't have enough info (shall not happen in a good design tree)
            _node.setOutput<uint64_t>(_port, _field.get<uint64_t>()); // just jump to the highest uint
        }
    
    }

    void deserializeFieldToDouble(BT::ActionNodeBase& _node, const std::string& _port, const Json& _field)
    {
        const auto port_info = _node.getPortInfo(_port);

        if(port_info.has_value() && !port_info.value().missingTypeInfo() && *(port_info.value().type()) == typeid(float))
        {
            _node.setOutput<float>(_port, _field.get<float>());
        }
        else 
            _node.setOutput(_port, _field.get<double>());
    }

    inline void deserializeFieldToBool(BT::ActionNodeBase& _node, const std::string& _port, const Json& _field)
    {
        _node.setOutput<bool>(_port, _field.get<bool>());
    }

    inline void deserializeFieldToString(BT::ActionNodeBase& _node, const std::string& _port, const Json& _field)
    {
        _node.setOutput<std::string>(_port, _field.get<std::string>());
    }

    inline void deserializeFieldToJson(BT::ActionNodeBase& _node, const std::string& _port, const Json& _field)
    {
        _node.setOutput<Json>(_port, _field);
    }

    static const Utils::UnorderedMap<Json::value_t, DeserializeFieldFunction> deserialize_field_map
    {
        { Json::value_t::boolean,         [] (auto& _node, const auto& _port, const auto& _field) { deserializeFieldToBool(_node, _port, _field);        }},
        { Json::value_t::string,          [] (auto& _node, const auto& _port, const auto& _field) { deserializeFieldToString(_node, _port, _field); }},
        { Json::value_t::number_integer,  [] (auto& _node, const auto& _port, const auto& _field) { deserializeFieldFromSignedInt(_node, _port, _field);     }},
        { Json::value_t::number_unsigned, [] (auto& _node, const auto& _port, const auto& _field) { deserializeFieldFromUnsignedInt(_node, _port, _field);    }},
        { Json::value_t::number_float,    [] (auto& _node, const auto& _port, const auto& _field) { deserializeFieldToDouble(_node, _port, _field);      }},
        { Json::value_t::object,          [] (auto& _node, const auto& _port, const auto& _field) { deserializeFieldToJson(_node, _port, _field);        }},
        { Json::value_t::array,           [] (auto& _node, const auto& _port, const auto& _field) { deserializeFieldToJson(_node, _port, _field);        }},
    };

    inline void deserializeField(BT::ActionNodeBase& _node, const std::string& _port, const Json& _field)
    {
        deserialize_field_map.at(_field.type())(_node, _port, _field);
    }

}
}

#endif
