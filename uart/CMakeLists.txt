include_directories(inc/)

aux_source_directory(src/. DIR_LIB_UART)
add_library(uart ${DIR_LIB_UART})