if(NOT DEFINED ROOT_DIR OR NOT DEFINED MAX_LINES)
  message(FATAL_ERROR "ROOT_DIR and MAX_LINES are required")
endif()

file(GLOB_RECURSE OWNED_SOURCES
  "${ROOT_DIR}/*.cpp"
  "${ROOT_DIR}/*.hpp"
  "${ROOT_DIR}/*.inc")

foreach(SOURCE_FILE IN LISTS OWNED_SOURCES)
  file(READ "${SOURCE_FILE}" SOURCE_TEXT)
  string(REGEX MATCHALL "\n" NEWLINES "${SOURCE_TEXT}")
  list(LENGTH NEWLINES LINE_COUNT)
  math(EXPR LINE_COUNT "${LINE_COUNT} + 1")
  if(LINE_COUNT GREATER MAX_LINES)
    message(FATAL_ERROR
      "${SOURCE_FILE} has ${LINE_COUNT} lines; the maximum is ${MAX_LINES}")
  endif()
endforeach()
