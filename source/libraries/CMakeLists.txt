file(GLOB directs "./*")

foreach(direct ${directs})
    get_filename_component(bext ${direct} EXT)
    if (${bext} MATCHES "txt")
    else()
        get_filename_component(bdirect ${direct} NAME_WE)
        add_subdirectory(${PROJECT_SOURCE_DIR}/libraries/${bdirect})
        message("${direct} ${bext}")
    endif()

endforeach()

#add_subdirectory(${PROJECT_SOURCE_DIR}/libraries/adcs)
#add_subdirectory(${PROJECT_SOURCE_DIR}/libraries/obc)
