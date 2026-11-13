# URT EtherCAT main-device
add_executable(main main.cpp)
target_link_libraries(main PRIVATE cia402 soem Threads::Threads)
target_compile_options(main PRIVATE -Wall -Wextra)
install(TARGETS main DESTINATION bin)
