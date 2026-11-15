# URT EtherCAT main-device
add_executable(main
    main.cpp
    ecat_master.cpp
    ecat_rt.cpp
    pdo.cpp
    rt_utils.cpp
)
target_link_libraries(main PRIVATE cia402 soem Threads::Threads)
target_compile_options(main PRIVATE -Wall -Wextra)
install(TARGETS main DESTINATION bin)
