# eRob CiA 402 control mode demos
set(EROB_DEMOS
    eRob_CSV
    eRob_PV
    eRob_CSP
    eRob_CSP_subscriber
    eRob_PP_subscriber
    eRob_PT
    eRob_CST
)

foreach(demo IN LISTS EROB_DEMOS)
    add_executable(${demo} ${demo}.cpp)
    target_link_libraries(${demo} PRIVATE cia402 soem Threads::Threads)
    install(TARGETS ${demo} DESTINATION bin)
endforeach()

# Command-line client that sends CSP target positions to eRob_CSP over a Unix socket
add_executable(CSP_command CSP_command.cpp)
install(TARGETS CSP_command DESTINATION bin)
