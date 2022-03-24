#ifndef TRACKER_MAIN_H
#define TRACKER_MAIN_H

#include "./uavos_common/uavos_module.hpp"

namespace uavos
{
namespace tracker
{

    class CTrackerMain :public uavos::CMODULE
    {

        public:
            
            //https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
            static CTrackerMain& getInstance()
            {
                static CTrackerMain instance;

                return instance;
            }

            CTrackerMain(CTrackerMain const&)           = delete;
            void operator=(CTrackerMain const&)         = delete;

        
            // Note: Scott Meyers mentions in his Effective Modern
            //       C++ book, that deleted functions should generally
            //       be public as it results in better error messages
            //       due to the compilers behavior to check accessibility
            //       before deleted status

        private:

            CTrackerMain()
            {
                // Define module features
                m_module_features.push_back("TRK");

                m_module_class="computing";
            }

        public:
            
            ~CTrackerMain()
            {
                if (m_exit_thread == false)
                {
                    uninit();
                }
            };
                

        public:

            void init() override;
            void uninit() override;

        
        private:
            
            bool m_exit_thread;

    };

};
};


#endif