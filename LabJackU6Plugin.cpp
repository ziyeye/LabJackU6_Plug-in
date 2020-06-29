/*
 *  LabJack U6 Plugin for MWorks
 *
 *  Created by Mark Histed on 4/21/2010
 *    (based on Nidaq plugin code by Jon Hendry and John Maunsell)
 *
 */

#include "LabJackU6Device.h"


BEGIN_NAMESPACE_MW


class LabJackU6Plugin : public Plugin {
    void registerComponents(shared_ptr<ComponentRegistry> registry) MW_OVERRIDE {
        registry->registerFactory<StandardComponentFactory, LabJackU6Device>();
        mprintf("LabJackU6Plugin, registered LabJackU6Device factory");
    }
};


extern "C" Plugin* getPlugin(){
    return new LabJackU6Plugin();
}


END_NAMESPACE_MW
