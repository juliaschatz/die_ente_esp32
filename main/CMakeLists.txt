file(GLOB SOURCES src/*.cpp)
set(COMPONENT_SRCS ${SOURCES})
set(COMPONENT_ADD_INCLUDEDIRS "src")
set(COMPONENT_ADD_INCLUDEDIRS "Eigen")

register_component()