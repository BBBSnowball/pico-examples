add_executable(rmii)

pico_generate_pio_header(rmii ${CMAKE_CURRENT_LIST_DIR}/rmii.pio)

target_sources(rmii PRIVATE
  rmii.c
  $ENV{PICO_EXTRAS_PATH}/src/rp2_common/lwip/lwip_arch.c
  lwip/src/core/inet_chksum.c
  lwip/src/core/def.c
  zlib/crc32.c)

target_include_directories(rmii PRIVATE
  lwip/src/include
  $ENV{PICO_EXTRAS_PATH}/src/rp2_common/lwip/include
  lwip/test/unit)

target_link_libraries(rmii PRIVATE pico_stdlib hardware_pio hardware_dma)
pico_add_extra_outputs(rmii)

# add url via pico_set_program_url
example_auto_set_url(rmii)
