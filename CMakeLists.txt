cmake_minimum_required(VERSION 3.8)

project(task-scheduler)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(FetchContent)


# Add source files for library 1
set(LPT_SOURCES
        src/execqueue.cpp
        src/myqueue.cpp
        src/taskscheduler.cpp
)

# Add header files for library 1
set(LPT_HEADERS
        include/execqueue.h
        include/myqueue.h
)

add_executable(task-scheduler ${LPT_SOURCES} ${LPT_HEADERS})
target_include_directories(task-scheduler PUBLIC include)
target_link_libraries(task-scheduler PRIVATE fmt pthread)

