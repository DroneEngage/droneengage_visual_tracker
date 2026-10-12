CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g
TARGETS = test_lock_on test_capabilities

.PHONY: all clean test

all: $(TARGETS)

test_lock_on: test_lock_on.cpp ../src/tracker/lock_on.hpp
	$(CXX) $(CXXFLAGS) -o $@ test_lock_on.cpp

test_capabilities: test_capabilities.cpp ../src/tracker/tracker_capabilities.hpp ../src/de_common/de_databus/cap_schema.cpp
	$(CXX) $(CXXFLAGS) -I../src -o $@ test_capabilities.cpp ../src/de_common/de_databus/cap_schema.cpp

test: $(TARGETS)
	./test_lock_on
	./test_capabilities

clean:
	rm -f $(TARGETS)
