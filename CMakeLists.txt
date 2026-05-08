# Include from git submodule
set(quicksppi_requires driver)

if(DEFINED ENV{IDF_PATH} AND EXISTS "$ENV{IDF_PATH}/components/esp_driver_spi/CMakeLists.txt")
    list(APPEND quicksppi_requires esp_driver_spi)
endif()

if(DEFINED ENV{IDF_PATH} AND EXISTS "$ENV{IDF_PATH}/components/esp_driver_gpio/CMakeLists.txt")
    list(APPEND quicksppi_requires esp_driver_gpio)
endif()

idf_component_register(SRCS "src/QuickSPI.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES ${quicksppi_requires})

# Add compile definitions based on menuconfig selection
if(CONFIG_QUICKSPI_DRIVER_ESPIDF)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC QUICKSPI_DRIVER_ESPIDF)
elseif(CONFIG_QUICKSPI_DRIVER_ARDUINO)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC QUICKSPI_DRIVER_ARDUINO)
endif()
