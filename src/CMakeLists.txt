include_directories(${CMAKE_SOURCE_DIR}/src
		            ${CMAKE_CURRENT_BINARY_DIR}
		            ${CMAKE_SOURCE_DIR}/lib
				    ${CMAKE_SOURCE_DIR}/include
			   )

link_directories(${LIBRARY_OUTPUT_PATH})


# Actual ray tracer. Same as above though.
add_executable(subdivision 	tut1.cpp)
target_link_libraries(subdivision  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})