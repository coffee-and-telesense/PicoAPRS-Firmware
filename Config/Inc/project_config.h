#pragma once

#ifndef TARGET_MCU
#error "TARGET_MCU not defined! See CmakeLists.txt"
#endif
#include "target_config.h"
#include "feature_config.h"

// Global project settings
#define PROJECT_VERSION "1.0.0"
#define PROJECT_NAME "PicoAPRS"

//@todo: Include guards for feature combinations
// It might be important to include different guards for conflicting feature combinations and/or target configs
