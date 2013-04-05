add_definitions("-DPROJECT_DATA_DIR=${CMAKE_SOURCE_DIR}/data")

include_directories(${CMAKE_SOURCE_DIR}/src
		            ${CMAKE_CURRENT_BINARY_DIR}
		            ${CMAKE_SOURCE_DIR}/lib
				    ${CMAKE_SOURCE_DIR}/include
			   )
link_directories(${LIBRARY_OUTPUT_PATH})

# Subdivision executable
add_executable(subdivision 	tut1.cpp)
target_link_libraries(subdivision  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

# Test for parser
add_executable(testParser tests/testParser.cpp
						  BezierPatch.cpp)
						  
# General rendering test
add_executable(testRender tests/testRender.cpp
						  BezierPatch.cpp)
target_link_libraries(testRender  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

# build the app which integrates openGL + subdivision
add_executable(dispApp tests/dispApp.cpp
						  BezierPatch.cpp)
target_link_libraries(dispApp  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
						  