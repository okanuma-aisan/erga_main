# Try to find fast_gicp library
find_path(FAST_GICP_INCLUDE_DIRS NAMES fast_gicp PATHS /usr/local/include
            /usr/include
            /home/sit/autoware_erga_humble2/install/fast_gicp/include/
            )

find_library(FAST_GICP_LIBRARIES NAMES fast_gicp PATHS /usr/local/include
            /usr/include
            /home/sit/autoware_erga_humble2/install/fast_gicp/lib/fast_gicp
            )
