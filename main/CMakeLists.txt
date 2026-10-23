idf_component_register(SRCS "main.c" "../Includes/DysonUart.c" "../Includes/IRimitator.c" "../Includes/HardTimer.c"
                    INCLUDE_DIRS ".")
set(EXTRA_COMPONENT_DIRS ../)
# GCC 14 (IDF 5.5) made these errors; the mismatches are same-size pointers on the ESP32.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=incompatible-pointer-types)
