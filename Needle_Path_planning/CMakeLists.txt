cmake_minimum_required(VERSION 3.16.3...3.19.7 FATAL_ERROR)

project(Needle_Path_planning)

#-----------------------------------------------------------------------------
# Extension meta-information
set(EXTENSION_HOMEPAGE "https://www.slicer.org/wiki/Documentation/Nightly/Extensions/Needle_Path_planning")
set(EXTENSION_CATEGORY "Path planning for brain surgery")
set(EXTENSION_CONTRIBUTORS "Dang The Hung (King's College London)")
set(EXTENSION_DESCRIPTION "This extension helps to find a path to insert a needle into the brain.")
set(EXTENSION_ICONURL "https://www.example.com/Slicer/Extensions/Needle_Path_planning.png")
set(EXTENSION_SCREENSHOTURLS "https://www.example.com/Slicer/Extensions/Needle_Path_planning/Screenshots/1.png")
set(EXTENSION_DEPENDS "NA") # Specified as a list or "NA" if no dependencies

#-----------------------------------------------------------------------------
# Extension dependencies
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

#-----------------------------------------------------------------------------
# Extension modules
add_subdirectory(Needle_Path_Planning)
## NEXT_MODULE

#-----------------------------------------------------------------------------
include(${Slicer_EXTENSION_GENERATE_CONFIG})
include(${Slicer_EXTENSION_CPACK})
