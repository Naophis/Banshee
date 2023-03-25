#ifndef STATIC_FUNC_HPP
#define STATIC_FUNC_HPP

#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sdkconfig.h"

void mount();
void umount();
#endif