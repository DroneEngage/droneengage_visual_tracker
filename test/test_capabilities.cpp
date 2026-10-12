/**
 * @file test_capabilities.cpp
 * @brief the visual_tracker advert (incl. lock/unlock) conforms to de.cap/1
 *        and its lock params validate the way the planner will call them.
 */

#include <iostream>
#include <string>
#include <vector>

#include "../src/de_common/de_databus/cap_schema.hpp"
#include "../src/tracker/tracker_capabilities.hpp"

#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            std::cerr << "ASSERTION FAILED: " << #condition << " at line " << __LINE__ << std::endl; \
            return 1; \
        } \
    } while(0)

int main ()
{
    for (int with_classes = 0; with_classes < 2; ++with_classes)
    {
        std::vector<std::string> classes;
        if (with_classes) classes = { "person", "car" };

        Json_de advert = de::tracker::buildVisualTrackerAdvert("1.0.0", classes);

        std::string err;
        const bool ok = de::comm::capValidateAdvert(advert, err);
        if (!ok) std::cerr << "advert invalid: " << err << std::endl;
        ASSERT_TRUE(ok);

        ASSERT_TRUE(advert["ns"] == "visual_tracker");
        ASSERT_TRUE(advert["actions"].contains("lock"));
        ASSERT_TRUE(advert["actions"].contains("unlock"));
        ASSERT_TRUE(advert["actions"].contains("track"));
        ASSERT_TRUE(advert["state"].contains("lock"));

        // lock params: both optional, so {trk} and {class} each validate
        const Json_de& params = advert["actions"]["lock"]["params"];
        Json_de filled;
        ASSERT_TRUE(de::comm::capCheckParams(params, Json_de::parse(R"({"trk":7})"), filled).empty());
        ASSERT_TRUE(de::comm::capCheckParams(params, Json_de::parse(R"({"class":"person"})"), filled).empty());
        ASSERT_TRUE(!de::comm::capCheckParams(params, Json_de::parse(R"({"trk":"7"})"), filled).empty());
        ASSERT_TRUE(!de::comm::capCheckParams(params, Json_de::parse(R"({"bogus":1})"), filled).empty());
    }

    std::cout << "ALL TESTS PASSED" << std::endl;
    return 0;
}
