file(READ "${ORIGINAL}" source)
set(marker "// Worker")
string(FIND "${source}" "${marker}" marker_position)
set(encoder_marker "// Public Interface - Encoding")
string(FIND "${source}" "${encoder_marker}" encoder_position)

if(marker_position EQUAL -1 OR encoder_position EQUAL -1)
  message(FATAL_ERROR "Expected section markers were not found in ${ORIGINAL}")
endif()

string(SUBSTRING "${source}" 0 ${marker_position} decoder_core)
string(SUBSTRING "${source}" ${encoder_position} -1 encoder)
file(READ "${WRAPPER}" wrapper)
set(combined "${decoder_core}\n${encoder}\n${wrapper}\n")

if(EXISTS "${OUTPUT}")
  file(READ "${OUTPUT}" previous)
  if(previous STREQUAL combined)
    return()
  endif()
endif()

file(WRITE "${OUTPUT}" "${combined}")
