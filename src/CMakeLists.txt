




file(GLOB folder_src "./*.cpp")
file(GLOB folder_common "./de_common/*.cpp")
file(GLOB folder_tracker "./tracker/*.cpp")
file(GLOB folder_helpers "./helpers/*.cpp")
set(files ${folder_src} ${folder_common} ${folder_tracker} ${folder_helpers})

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( de_tracker ${files})
target_link_libraries(de_tracker  Threads::Threads  ${OpenCV_LIBS} )
