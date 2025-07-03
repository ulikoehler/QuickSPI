# Include from git submodule
idf_component_register(SRCS "src/QuickSPI.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver)
