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
#include "../de_common/de_databus/cap_schema.hpp"

namespace de
{
namespace tracker
{

/**
 * @brief The de.cap namespace of a visual_tracker advert: one per cam_id,
 * so several trackers on one unit (e.g. RGB + IR) each own a namespace.
 * "visual_tracker_" + the cam_id lower-cased with every char outside
 * [a-z0-9_] mapped to '_' (the de.cap name grammar has no '.'), capped at
 * CAP_NAME_MAX_LEN. An empty cam_id gives the bare "visual_tracker".
 * (MP-0217: a mission track step pins the tracker whose cam_id matches
 * the confirming observation.)
 */
inline std::string visualTrackerNamespace (const std::string& cam_id)
{
  std::string ns = "visual_tracker";
  if (cam_id.empty()) return ns;
  ns += '_';
  for (const char c : cam_id)
  {
    if (ns.size() >= CAP_NAME_MAX_LEN) break;
    const char l = (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
    const bool ok = (l >= 'a' && l <= 'z') || (l >= '0' && l <= '9') || l == '_';
    ns += ok ? l : '_';
  }
  return ns;
}

inline nlohmann::json buildVisualTrackerAdvert (const std::string& version,
                                                const std::vector<std::string>& ai_class_names,
                                                const std::string& cam_id = "")
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

  const Json_de cam_param = Json_de::object(
      {{"type", "string"},
       {"optional", true},
       {"desc", "cam_id routing hint; must match this tracker when present"}});

  const Json_de advert = Json_de::object(
      {{"schema", "de.cap/1"},
       {"ns", visualTrackerNamespace(cam_id)},
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
                         {"class", Json_de::object({{"type", "string"}, {"optional", true}, {"desc", "class label; empty = any class"}})},
                         {"cam", cam_param}})}})},
             {"unlock",
              Json_de::object(
                  {{"desc", "stop holding a track"},
                   {"params", Json_de::object({{"cam", cam_param}})}})},
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
            {{"cam_id",
              Json_de::object(
                  {{"type", "string"},
                   {"default", cam_id},
                   {"desc", "cam_id of the stream this tracker holds locks on"}})},
             {"tracking",
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
