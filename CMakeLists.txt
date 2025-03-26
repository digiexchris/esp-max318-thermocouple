cmake_minimum_required(VERSION 3.10)

file(GLOB SOURCES src/max31856-espidf/*.cxx)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS include
    PRIV_INCLUDE_DIRS src
    REQUIRES ${pub_requires} driver
    PRIV_REQUIRES ${priv_requires}
)
