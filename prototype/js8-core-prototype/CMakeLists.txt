cmake_minimum_required(VERSION 3.22)
project(js8_core_prototype LANGUAGES CXX)

get_filename_component(PROJECT_REPOSITORY "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(JS8CALL_SOURCE "${PROJECT_REPOSITORY}/JS8Call-improved-master")
set(JS8_REFERENCE_CORE_SHA256
  "b72787f6e6473919872103997ef5dc9847810ff3e7783124a3961a2da2a86bf1"
)
file(SHA256 "${JS8CALL_SOURCE}/JS8_Mode/JS8.cpp" JS8_REFERENCE_CORE_ACTUAL_SHA256)
if(NOT JS8_REFERENCE_CORE_ACTUAL_SHA256 STREQUAL JS8_REFERENCE_CORE_SHA256)
  message(FATAL_ERROR
    "Pinned JS8.cpp changed. Review upstream DSP and update the content pin.\n"
    "expected=${JS8_REFERENCE_CORE_SHA256}\n"
    "actual=${JS8_REFERENCE_CORE_ACTUAL_SHA256}"
  )
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(js8_proto_core STATIC
  src/js8_core.cpp
  src/audio_frontend.cpp
  src/aud1_protocol.cpp
)
target_include_directories(js8_proto_core PUBLIC src)
target_compile_options(js8_proto_core PRIVATE -Wall -Wextra -Wpedantic)

if(EMSCRIPTEN)
  add_executable(js8-prototype-wasm
    wasm/wasm_api.cpp
  )
  target_include_directories(js8-prototype-wasm PRIVATE wasm)
  target_link_libraries(js8-prototype-wasm PRIVATE js8_proto_core)
  target_compile_options(js8-prototype-wasm PRIVATE -Wall -Wextra -Wpedantic)
  target_link_options(js8-prototype-wasm PRIVATE
    --no-entry
    -sMODULARIZE=1
    -sEXPORT_NAME=createJs8Prototype
    -sENVIRONMENT=web,worker,node
    -sWASM_BIGINT=1
    -sALLOW_MEMORY_GROWTH=1
    -sFILESYSTEM=0
    "-sEXPORTED_FUNCTIONS=['_malloc','_free','_js8_proto_get_submode','_js8_proto_encode_frame','_js8_proto_modulate_frame48k','_js8_audio_create','_js8_audio_destroy','_js8_audio_reset','_js8_audio_push_ulaw','_js8_audio_finish','_js8_audio_copy_output','_js8_audio_snapshot','_js8_audio_next_window']"
    "-sEXPORTED_RUNTIME_METHODS=['cwrap']"
  )
  set_target_properties(js8-prototype-wasm PROPERTIES
    OUTPUT_NAME js8-prototype
    SUFFIX .js
  )
  return()
endif()

add_executable(js8-core-prototype
  src/main.cpp
)
target_link_libraries(js8-core-prototype PRIVATE js8_proto_core)
target_compile_options(js8-core-prototype PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-resampler-benchmark
  src/resampler_benchmark.cpp
)
target_link_libraries(js8-resampler-benchmark PRIVATE js8_proto_core)
target_compile_options(js8-resampler-benchmark PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-aud1-emitter-smoke
  firmware/aud1_rx_emitter.cpp
  firmware/aud1_rx_emitter_smoke.cpp
)
target_include_directories(js8-aud1-emitter-smoke PRIVATE firmware)
target_link_libraries(js8-aud1-emitter-smoke PRIVATE js8_proto_core)
target_compile_options(js8-aud1-emitter-smoke PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-aud1-tx-gate-smoke
  firmware/aud1_tx_gate.cpp
  firmware/aud1_tx_gate_smoke.cpp
)
target_include_directories(js8-aud1-tx-gate-smoke PRIVATE firmware)
target_link_libraries(js8-aud1-tx-gate-smoke PRIVATE js8_proto_core)
target_compile_options(js8-aud1-tx-gate-smoke PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-aud1-disconnect-smoke
  firmware/aud1_disconnect_smoke.cpp
)
target_include_directories(js8-aud1-disconnect-smoke PRIVATE ${PROJECT_REPOSITORY})
target_compile_options(js8-aud1-disconnect-smoke PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-aud1-ws-parser-smoke
  firmware/aud1_ws_parser_smoke.cpp
)
target_include_directories(js8-aud1-ws-parser-smoke PRIVATE ${PROJECT_REPOSITORY})
target_compile_options(js8-aud1-ws-parser-smoke PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-unattended-guard-smoke
  firmware/unattended_guard_smoke.cpp
)
target_include_directories(js8-unattended-guard-smoke PRIVATE ${PROJECT_REPOSITORY})
target_compile_options(js8-unattended-guard-smoke PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-session-smoke
  firmware/js8_session_smoke.cpp
)
target_include_directories(js8-session-smoke PRIVATE ${PROJECT_REPOSITORY})
target_compile_options(js8-session-smoke PRIVATE -Wall -Wextra -Wpedantic)

include(CheckIncludeFileCXX)
check_include_file_cxx("boost/crc.hpp" JS8_PROTO_HAVE_BOOST_CRC)
if(JS8_PROTO_HAVE_BOOST_CRC)
  target_compile_definitions(js8-core-prototype PRIVATE JS8_PROTO_HAVE_BOOST_CRC=1)
endif()

# Native reference decoder spike. This target deliberately lives only in the
# prototype: it compiles the DSP portion of upstream JS8.cpp without its Qt
# worker/UI layer. It is the executable specification for a later WASM port.
find_library(FFTW3F_LIBRARY fftw3f REQUIRED)

set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${GENERATED_DIR}")
execute_process(
  COMMAND "${CMAKE_COMMAND}"
    -DORIGINAL=${JS8CALL_SOURCE}/JS8_Mode/JS8.cpp
    -DWRAPPER=${CMAKE_CURRENT_SOURCE_DIR}/decoder_shim/decoder_wrapper.inc
    -DOUTPUT=${GENERATED_DIR}/js8_reference_core.cpp
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/extract_reference_core.cmake
  COMMAND_ERROR_IS_FATAL ANY
)

add_library(js8_reference_decoder STATIC
  ${GENERATED_DIR}/js8_reference_core.cpp
  ${JS8CALL_SOURCE}/JS8_Mode/FrequencyTracker.cpp
  decoder_shim/qt_shim.cpp
)
target_include_directories(js8_reference_decoder
  PUBLIC decoder_shim
  PRIVATE ${JS8CALL_SOURCE} ${JS8CALL_SOURCE}/JS8_Mode
)
target_link_libraries(js8_reference_decoder PRIVATE ${FFTW3F_LIBRARY})
target_compile_options(js8_reference_decoder PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-reference-decoder
  decoder_shim/decoder_main.cpp
)
target_link_libraries(js8-reference-decoder
  PRIVATE js8_reference_decoder ${FFTW3F_LIBRARY}
)
target_compile_options(js8-reference-decoder PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-stream-prototype
  src/stream_main.cpp
)
target_link_libraries(js8-stream-prototype
  PRIVATE js8_proto_core js8_reference_decoder ${FFTW3F_LIBRARY}
)
target_compile_options(js8-stream-prototype PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-golden-fixtures
  src/golden_main.cpp
)
target_link_libraries(js8-golden-fixtures
  PRIVATE js8_proto_core js8_reference_decoder ${FFTW3F_LIBRARY}
)
target_compile_options(js8-golden-fixtures PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-weak-signal
  src/weak_signal_main.cpp
)
target_link_libraries(js8-weak-signal
  PRIVATE js8_proto_core js8_reference_decoder ${FFTW3F_LIBRARY}
)
target_compile_options(js8-weak-signal PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-channel-matrix
  src/channel_matrix_main.cpp
)
target_link_libraries(js8-channel-matrix
  PRIVATE js8_proto_core js8_reference_decoder ${FFTW3F_LIBRARY}
)
target_compile_options(js8-channel-matrix PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-drift-replay
  src/drift_replay_main.cpp
)
target_link_libraries(js8-drift-replay
  PRIVATE js8_proto_core js8_reference_decoder ${FFTW3F_LIBRARY}
)
target_compile_options(js8-drift-replay PRIVATE -Wall -Wextra -Wpedantic)

add_executable(js8-capture-replay
  src/capture_replay_main.cpp
)
target_link_libraries(js8-capture-replay
  PRIVATE js8_proto_core js8_reference_decoder ${FFTW3F_LIBRARY}
)
target_compile_options(js8-capture-replay PRIVATE -Wall -Wextra -Wpedantic)
