find_package(Protobuf REQUIRED)

set(ProtobufIncludePath ${CMAKE_CURRENT_BINARY_DIR}
    CACHE INTERNAL "Path to generated protobuf files.")
message(ProtobufIncludePath ${ProtobufIncludePath})

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${ProtoFiles})
message(ProtoSources ${ProtoSources})

add_library(messages STATIC ${ProtoSources} ${ProtoHeaders})
message(ProtoSources ${PROTOBUF_LIBRARY})
target_link_libraries(messages ${PROTOBUF_LIBRARY})

target_include_directories(messages PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
