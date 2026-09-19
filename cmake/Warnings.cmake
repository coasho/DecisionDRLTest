# fsim_set_warnings(<target>)
# Strict warnings for platform code only; third-party targets are left alone.
function(fsim_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            /W4
            /w14242 /w14254 /w14263 /w14265 /w14287 /w14296 /w14311
            /w14545 /w14546 /w14547 /w14549 /w14555 /w14619 /w14640
            /w14826 /w14905 /w14906 /w14928
            /external:W0
            $<$<BOOL:${FSIM_WARNINGS_AS_ERRORS}>:/WX>)
    else()
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
            -Wnon-virtual-dtor -Wold-style-cast -Wcast-align -Wunused
            -Woverloaded-virtual -Wnull-dereference -Wdouble-promotion
            $<$<BOOL:${FSIM_WARNINGS_AS_ERRORS}>:-Werror>)
    endif()
endfunction()
