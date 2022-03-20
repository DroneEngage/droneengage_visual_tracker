


set(OUTPUT_DIRECTORY "../bin")
file(MAKE_DIRECTORY ${OUTPUT_DIRECTORY})
SET(EXECUTABLE_OUTPUT_PATH ${OUTPUT_DIRECTORY})

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( tracker_main main.cpp tracker.cpp)
target_link_libraries(tracker_main  ${OpenCV_LIBS} )
