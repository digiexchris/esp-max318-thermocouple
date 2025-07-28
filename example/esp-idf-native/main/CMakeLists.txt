idf_component_register(SRCS "MAX31856_Example.cxx"
                    INCLUDE_DIRS "."
                    REQUIRES esp-max318-thermocouple
                    )

                    target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)