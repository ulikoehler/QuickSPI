# Include from git submodule
idf_component_register(SRCS "src/QuickSPI.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver)
     
# Add compile definitions based on menuconfig selection
if(CONFIG_QUICKSPI_DRIVER_ESPIDF)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC QUICKSPI_DRIVER_ESPIDF)
elseif(CONFIG_QUICKSPI_DRIVER_ARDUINO)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC QUICKSPI_DRIVER_ARDUINO)
endif()
