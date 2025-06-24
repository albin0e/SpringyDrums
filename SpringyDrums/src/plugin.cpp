#include "plugin.hpp"

Plugin* pluginInstance;

void init(Plugin* p) {
    pluginInstance = p;
    
    // Add your modules here
    p->addModel(modelMyModule);
}
