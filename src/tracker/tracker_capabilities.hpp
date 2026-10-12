#ifndef TRACKER_CAPABILITIES_H
#define TRACKER_CAPABILITIES_H

/**
 * @brief The "visual_tracker" capability advert (de.cap/1) of de_tracker,
 * built as a pure function so it can be validated against
 * de_common's capValidateAdvert in a unit test.
 */

#include <string>
#include <vector>

#include "../de_common/helpers/json_nlohmann.hpp"

namespace de
{
namespace tracker
{

inline nlohmann::json buildVisualTrackerAdvert (const std::string& version, const std::vector<std::string>& ai_class_names)
{
  using Json_de = nlohmann::json;

  Json_de class_param = Json_de::object(
      {{"type", "string"}, {"required", true}, {"desc", "class to search for"}});
  if (!ai_class_names.empty()) {
    class_param = Json_de::object({{"type", "enum"},
                                   {"required", true},
                                   {"values", ai_class_names},
                                   {"desc", "class to search for"}});
  }

  const Json_de advert = Json_de::object(
      {{"schema", "de.cap/1"},
       {"ns", "visual_tracker"},
       {"module", "droneengage_visual_tracker"},
       {"ver", version},
       {"actions",
        Json_de::object(
            {{"start",
              Json_de::object(
                  {{"desc", "enable AI recognition and search for a class"},
                   {"params", Json_de::object({{"class", class_param}})}})},
             {"stop",
              Json_de::object({{"desc", "disable AI recognition"}})},
             {"lock",
              Json_de::object(
                  {{"desc", "hold a perceived track: by track id, or the first confirmed track of a class"},
                   {"params",
                    Json_de::object(
                        {{"trk", Json_de::object({{"type", "int"}, {"optional", true}, {"desc", "track id from the observation stream"}})},
                         {"class", Json_de::object({{"type", "string"}, {"optional", true}, {"desc", "class label; empty = any class"}})}})}})},
             {"unlock",
              Json_de::object({{"desc", "stop holding a track"}})},
             {"track",
              Json_de::object(
                  {{"desc", "track a region of the video frame"},
                   {"params",
                    Json_de::object(
                        {{"target",
                          Json_de::object(
                              {{"type", "object"},
                               {"required", true},
                               {"desc", "normalized box, x,y top-left in 0..1"},
                               {"fields",
                                Json_de::object(
                                    {{"x", Json_de::object({{"type", "number"}, {"required", true}, {"min", 0.0}, {"max", 1.0}})},
                                     {"y", Json_de::object({{"type", "number"}, {"required", true}, {"min", 0.0}, {"max", 1.0}})},
                                     {"w", Json_de::object({{"type", "number"}, {"required", true}, {"min", 0.01}, {"max", 1.0}})},
                                     {"h", Json_de::object({{"type", "number"}, {"required", true}, {"min", 0.01}, {"max", 1.0}})}})}})}})}})}})},
       {"state",
        Json_de::object(
            {{"tracking",
              Json_de::object(
                  {{"type", "enum"},
                   {"values", {"lost", "detected", "enabled", "stopped",
                               "config"}}})},
             {"ai",
              Json_de::object(
                  {{"type", "enum"},
                   {"values", {"lost", "detected", "enabled", "disabled"}}})},
             {"lock",
              Json_de::object(
                  {{"type", "string"},
                   {"desc", "none | trk:<id> | class:<label> | class:*"}})}})}});

  return advert;
}

}
}

#endif
