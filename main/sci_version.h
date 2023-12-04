/**
 * \file version.h
 * \brief Firmware Version Management.
 *
 * This file contains a version string, which is directly linked to the commit hash to ensure accurate version tracking
 * during development and release cycles. The version information can be automatically updated by executing the
 * 'update_version.sh' script or integrating it within the git pre-commit hook, ensuring real-time versioning corresponding
 * to the repository state.
 */

#pragma once

// Represents a specific versioning scheme for BITalino boards, ensuring compatibility and feature tracking.
#define FIRMWARE_BITALINO_VERSION "BITalino_v5.1\n"

// Flag indicating whether the version can be incremented, providing a mechanism for version control logic.
#define VERSION_CAN_INCREMENT_FLAG 0

// The current firmware version, amalgamating semantic versioning with the commit hash for precise version tracking.
#define FIRMWARE_VERSION "4.0.0-d0eb4de"
