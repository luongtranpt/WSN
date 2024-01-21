# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/esp/esp_idf/components/bootloader/subproject"
  "D:/tai_lieu_hoc_tap/2023-1/WSN/Firmware/Gate_V1/hel/build/bootloader"
  "D:/tai_lieu_hoc_tap/2023-1/WSN/Firmware/Gate_V1/hel/build/bootloader-prefix"
  "D:/tai_lieu_hoc_tap/2023-1/WSN/Firmware/Gate_V1/hel/build/bootloader-prefix/tmp"
  "D:/tai_lieu_hoc_tap/2023-1/WSN/Firmware/Gate_V1/hel/build/bootloader-prefix/src/bootloader-stamp"
  "D:/tai_lieu_hoc_tap/2023-1/WSN/Firmware/Gate_V1/hel/build/bootloader-prefix/src"
  "D:/tai_lieu_hoc_tap/2023-1/WSN/Firmware/Gate_V1/hel/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/tai_lieu_hoc_tap/2023-1/WSN/Firmware/Gate_V1/hel/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
