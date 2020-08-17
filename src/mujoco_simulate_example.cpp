#include "pam_mujoco/mujoco_base.hpp"

//-------------------------------- global -----------------------------------------------


int main()
{

    // initialize everything
    init();

    // request loadmodel if file given (otherwise drag-and-drop)
    /*if( argc>1 )
    {
        mju_strncpy(filename, argv[1], 1000);
        settings.loadrequest = 2;
	}*/

    std::string foo;
    run(&foo);
    
    return 0;

}
