

file(GLOB folder_src "./*.cpp")
file(GLOB folder_helpers "./helpers/*.cpp")
set(files ${folder_src} ${folder_helpers})

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( tracker_main ${files})
target_link_libraries(tracker_main  Threads::Threads  ${OpenCV_LIBS} )
