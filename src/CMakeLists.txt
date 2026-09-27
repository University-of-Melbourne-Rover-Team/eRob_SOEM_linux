cmake_minimum_required(VERSION 3.8)
project(urt_ecat_soem)


# project(urt_ecat_soem LANGUAGES C CXX)
find_package(Threads REQUIRED)

include_directories(/home/erobman/SOEM_ethercat/install/SOEM/include/soem)
link_directories(/home/erobman/SOEM_ethercat/install/SOEM/lib)

add_executable(main main.cpp)
target_link_libraries(main cia402 soem)
install(TARGETS main DESTINATION bin)
