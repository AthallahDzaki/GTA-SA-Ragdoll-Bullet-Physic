include(FetchContent)
FetchContent_Declare(plugin-sdk
  GIT_REPOSITORY https://github.com/AthallahDzaki/plugin-sdk
  GIT_SHALLOW 2147483647)

set(PSDK_BUILD_SA ON)

message("Fetching plugin-sdk from GitHub...")
FetchContent_MakeAvailable(plugin-sdk)

# Set PLUGIN_SDK_DIR to the location of the downloaded plugin-sdk
set(PLUGIN_SDK_DIR ${plugin-sdk_SOURCE_DIR})
message("PLUGIN_SDK_DIR set to ${PLUGIN_SDK_DIR}")
