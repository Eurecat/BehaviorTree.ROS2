#include "behavior_tree_ros/details/commons.hpp"

#include <ros/ros.h>
#include "yaml-cpp/yaml.h"

namespace BT_ROS
{
    void InitializeBlackboard(const std::string& abs_file_path, BT::Blackboard::Ptr blackboard_ptr, const bool sync_bb)
    {
        try 
        {
            // ROS_INFO("Initializing BB from YAML file %s", abs_file_path.c_str());
            YAML::Node config = YAML::LoadFile(abs_file_path);
            for(YAML::const_iterator it=config.begin();it!=config.end();++it)
            {
                const std::string& bb_key = it->first.as<std::string>();
                std::string bb_val = it->second.as<std::string>();
                const BT::Optional<std::string> bbentry_value_inferred_keyvalues = blackboard_ptr->replaceKeysWithStringValues(bb_val, true); // no effect if it has no key
                if(!bbentry_value_inferred_keyvalues)
                {
                    // but will complain if it has a reference to a wrong key
                    ROS_ERROR("Init. of BB key %s for value %s, value inference did not succeed: %s", 
                        bb_key.c_str(), 
                        bb_val.c_str(),
                        bbentry_value_inferred_keyvalues.error().c_str());
                    continue; // and skip this init
                }
                else
                    bb_val = bbentry_value_inferred_keyvalues.value();

                ROS_INFO("Init. BB key [\"%s\"] with value \"%s\"", bb_key.c_str(), bb_val.c_str());
                // use the string here and blackboard_ptr->set(...)
                blackboard_ptr->set(bb_key, bb_val, sync_bb);
            }
            ROS_INFO("Initialized BB with %ld entries from YAML file %s", blackboard_ptr->getKeys().size(), abs_file_path.c_str());
        }
        catch(const YAML::Exception& ex) 
        { 
            ROS_ERROR("Initializing. BB key from file '%s' did not succeed: %s", abs_file_path.c_str(), ex.what());
        }
    }
}
