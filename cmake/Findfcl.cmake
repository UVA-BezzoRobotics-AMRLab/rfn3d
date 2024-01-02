find_path(fcl_INCLUDE_DIRS
    NAME collision.h
    HINTS /usr/include /usr/local/include
    PATH_SUFFIXES fcl
)

find_library(fcl_LIBRARIES
    NAME libfcl.so
    HINTS /usr/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(fcl REQUIRED_VARS fcl_LIBRARIES fcl_INCLUDE_DIRS)
