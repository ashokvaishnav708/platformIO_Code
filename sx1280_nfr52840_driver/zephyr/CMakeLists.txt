cmake_minimum_required(VERSION 3.13.1)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(sx1280_nfr52840_driver)

FILE(GLOB app_sources ../src/*.c*)
target_sources(app PRIVATE ${app_sources})
