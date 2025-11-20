
if(NOT "H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps/numcpp-subbuild/numcpp-populate-prefix/src/numcpp-populate-stamp/numcpp-populate-gitinfo.txt" IS_NEWER_THAN "H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps/numcpp-subbuild/numcpp-populate-prefix/src/numcpp-populate-stamp/numcpp-populate-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: 'H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps/numcpp-subbuild/numcpp-populate-prefix/src/numcpp-populate-stamp/numcpp-populate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps/numcpp-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps/numcpp-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe"  clone --no-checkout --config "advice.detachedHead=false" "https://github.com/dpilger26/NumCpp" "numcpp-src"
    WORKING_DIRECTORY "H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/dpilger26/NumCpp'")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe"  checkout Version_2.14.2 --
  WORKING_DIRECTORY "H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps/numcpp-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'Version_2.14.2'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe"  submodule update --recursive --init 
    WORKING_DIRECTORY "H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps/numcpp-src"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps/numcpp-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps/numcpp-subbuild/numcpp-populate-prefix/src/numcpp-populate-stamp/numcpp-populate-gitinfo.txt"
    "H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps/numcpp-subbuild/numcpp-populate-prefix/src/numcpp-populate-stamp/numcpp-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'H:/ECE 420/ECE_420_final/lab4/lab4/app/.cxx/Debug/4f4du2c1/x86_64/_deps/numcpp-subbuild/numcpp-populate-prefix/src/numcpp-populate-stamp/numcpp-populate-gitclone-lastrun.txt'")
endif()

