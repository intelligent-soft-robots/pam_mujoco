
#include "pam_mujoco/mujoco_base.hpp"

using namespace mujoco;

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

    // start simulation thread
    std::thread simthread(simulate);

    // event loop
    while( !glfwWindowShouldClose(window) && !settings.exitrequest )
    {
        // start exclusive access (block simulation thread)
        mtx.lock();

        // load model (not on first pass, to show "loading" label)
        if( settings.loadrequest==1 )
            loadmodel();
        else if( settings.loadrequest>1 )
            settings.loadrequest = 1;

        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

        // render while simulation is running
        render(window);
    }

    // stop simulation thread
    settings.exitrequest = 1;
    simthread.join();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d); 
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // deactive MuJoCo
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 0;

}
