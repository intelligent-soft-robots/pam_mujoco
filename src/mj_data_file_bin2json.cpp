/**
 * @file
 * @brief Load a binary Mujoco state file and output its content as JSON.
 * @copyright 2022, Max Planck Gesellschaft.  All rights reserved.
 * @license BSD 3-clause
 */
#include <iostream>

#include <pam_mujoco/mj_state_tools.hpp>

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Invalid number of arguments." << std::endl;
        std::cerr << "Usage: " << argv[0] << " <input_file>" << std::endl;
        return 1;
    }
    std::string input_file = argv[1];

    pam_mujoco::print_state_file(input_file);

    return 0;
}
