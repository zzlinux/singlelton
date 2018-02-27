set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(protocol
        serialapp.cpp
        serialbase.cpp
        utility.cpp
        )