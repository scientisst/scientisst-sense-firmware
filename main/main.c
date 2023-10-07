/** \file main.c
    \brief Main file for the Scientisst project.

    This file contains the main function for the Scientisst project.
*/

#include "scientisst.h"

/**
 *\brief Main function for the Scientisst project.
 *
 * This function initializes the Scientisst firmware and then deletes itself.
 */
void app_main(void)
{
    initScientisst();
    vTaskDelete(NULL);
}
