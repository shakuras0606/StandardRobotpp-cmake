set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Fixed absolute path so name resolution cannot pick up a different
# arm-none-eabi toolchain that may appear earlier on PATH (e.g. msys2).
set(ARM_TOOLCHAIN_DIR "D:/arm-gnu-toolchain" CACHE PATH "Arm GNU toolchain root directory")

set(CMAKE_C_COMPILER "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_ASM_COMPILER "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-gcc.exe")

set(CMAKE_OBJCOPY "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-objcopy.exe")
set(CMAKE_SIZE "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-size.exe")
set(CMAKE_AR "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-ar.exe")
set(CMAKE_NM "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-nm.exe")
set(CMAKE_READELF "${ARM_TOOLCHAIN_DIR}/bin/arm-none-eabi-readelf.exe")

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
