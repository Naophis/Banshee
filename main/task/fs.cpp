
#include "fs.hpp"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sdkconfig.h"

esp_vfs_fat_mount_config_t mount_config;
wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
bool mount_state = false;

void mount() {
  if (mount_state) {
    return;
  }
  mount_config.max_files = 8;
  mount_config.format_if_mount_failed = true;
  mount_config.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;
  const char *base_path = "/spiflash";

  printf("storage0: try mount\n");
  esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(base_path, "storage0",
                                                   &mount_config, &s_wl_handle);
  if (err != ESP_OK) {
    printf("storage0: Failed to mount FATFS (%s)\n", esp_err_to_name(err));
    return;
  } else {
    printf("storage0: mount OK\n");
    mount_state = true;
  }
}

void umount() {
  if (!mount_state) {
    return;
  }
  const char *base_path = "/spiflash";
  esp_vfs_fat_spiflash_unmount(base_path, s_wl_handle);
  mount_state = false;
}