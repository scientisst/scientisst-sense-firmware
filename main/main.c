/**
 * \file main.c
 * \brief Entry point of the Scientisst application.
 *
 */

#include "sci_scientisst.h"

/**
 * \brief Starts the application's main logic.
 *
 * The function serves as the starting point for the Scientisst application. It first
 * initializes the required Scientisst firmware, setting up the environment for the application.
 * After the initial setup, it terminates itseslf.
 */
void app_main(void)
{
    // Initialize the Scientisst environment.
    initScientisst();

    // Task deletion indicating the end of the initialization process.
    vTaskDelete(NULL);
}
