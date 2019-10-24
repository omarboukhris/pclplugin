#include <string>
#include <stdio.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/core/ObjectFactory.h>

#define Q(x) #x
#define QUOTE(x) Q(x)

#ifndef PLUGIN_DATA_DIR
#define PLUGIN_DATA_DIR_ ""
#else
#define PLUGIN_DATA_DIR_ QUOTE(PLUGIN_DATA_DIR)
#endif

#ifndef PLUGIN_GIT_INFO
#define PLUGIN_GIT_INFO_ ""
#else
#define PLUGIN_GIT_INFO_ QUOTE(PLUGIN_GIT_INFO)
#endif

namespace sofa {

namespace PCLPlugin {

	//Here are just several convenient functions to help user to know what contains the plugin

	extern "C" {
        void initExternalModule();
        const char* getModuleName();
        const char* getModuleVersion();
        const char* getModuleLicense();
        const char* getModuleDescription();
        const char* getModuleComponentList();
	}
	
	void initExternalModule()
	{
		static bool first = true;
		if (first)
		{
            first = false;
            sofa::helper::system::DataRepository.addLastPath(std::string(PLUGIN_DATA_DIR_));
            sofa::helper::system::DataRepository.addLastPath(std::string(PLUGIN_DATA_DIR_) + "/data");
            sofa::helper::system::DataRepository.addLastPath(sofa::helper::system::SetDirectory::GetCurrentDir());
		}
	}

	const char* getModuleName()
	{
        return "PCLPlugin";
	}

	const char* getModuleVersion()
	{
        return "0.0";
	}

	const char* getModuleLicense()
	{
		return "LGPL";
	}

	const char* getModuleDescription()
    {
        std::ostringstream oss;
        oss << "<MODULE_DESCRIPTION>" << std::endl
            << "<GIT>" << PLUGIN_GIT_INFO_ ;

        char* desc = new char[oss.str().size()];
        strcpy(desc, oss.str().c_str());

        return desc;
	}

	const char* getModuleComponentList()
	{
        return "";
	}

} 

} 
