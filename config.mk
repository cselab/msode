CXX = gcc
LD  = g++
CXXFLAGS = -O3 -Wpedantic -std=c++14 -Wall

debug ?= 0

ifeq ($(debug),1)
CXXFLAGS += -DFAIL_ON_CONTRACT
endif

LDFLAGS = 