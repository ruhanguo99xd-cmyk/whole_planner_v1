if(TARGET nlopt::nlopt)
  return()
endif()

find_path(INTEGRATED_MISSION_NLOPT_INCLUDE_DIR
  NAMES nlopt.h
)

find_library(INTEGRATED_MISSION_NLOPT_LIBRARY
  NAMES nlopt
)

if(NOT INTEGRATED_MISSION_NLOPT_INCLUDE_DIR OR NOT INTEGRATED_MISSION_NLOPT_LIBRARY)
  message(FATAL_ERROR
    "System nlopt not found. Install it with: "
    "sudo apt-get update && sudo apt-get install -y libnlopt-dev"
  )
endif()

add_library(nlopt::nlopt SHARED IMPORTED GLOBAL)
set_target_properties(nlopt::nlopt PROPERTIES
  IMPORTED_LOCATION "${INTEGRATED_MISSION_NLOPT_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${INTEGRATED_MISSION_NLOPT_INCLUDE_DIR}"
)

function(integrated_mission_link_nlopt target_name)
  target_include_directories(${target_name} PRIVATE "${INTEGRATED_MISSION_NLOPT_INCLUDE_DIR}")
  target_link_libraries(${target_name} nlopt::nlopt)
endfunction()
