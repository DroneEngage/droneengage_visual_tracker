#ifndef TRACKER_FACADE_MODULE_H
#define TRACKER_FACADE_MODULE_H


#include "../helpers/json_nlohmann.hpp"

using Json_de = nlohmann::json;

#include "../de_common/de_facade_base.hpp"
#include "../de_common/messages.hpp"




namespace de
{
namespace tracker
{
    class CTracker_Facade : public de::comm::CFacade_Base
    {
        public:
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CTracker_Facade& getInstance()
            {
                static CTracker_Facade instance;

                return instance;
            }

            CTracker_Facade(CTracker_Facade const&)         = delete;
            void operator=(CTracker_Facade const&)          = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CTracker_Facade()
            {
            };

        public:
            
            ~CTracker_Facade ()
            {
                
            };
                

        public:
           
            void sendTrackingTargetsLocation(const std::string& target_party_id, const Json_de targets_location) const;
            void sendTrackingTargetStatus(const std::string& target_party_id, const int status) const;
        protected:

            
    };
}
}


#endif