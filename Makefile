
CXX = g++
CC = gcc
CXXFLAGS = -I. -Wall -Wno-unknown-pragmas
CFLAGS = -I. -Wall

SRCS_COMMON = ArduinoMock.cpp ArduinoMockSim.cpp
OBJS_COMMON = ArduinoMock.o ArduinoMockSim.o

INO_FILES = $(wildcard *.ino)
C_FILES = $(wildcard *.c)

TEST_EXES = $(INO_FILES:.ino=.test) $(C_FILES:.c=.test)
CSV_FILES = $(TEST_EXES:.test=.csv)
PNG_FILES = $(TEST_EXES:.test=_results.png)

# Common flags for include files
INC_FLAGS = -include Arduino.h -include avr/io.h -include avr/wdt.h -include avr/interrupt.h -include shared_defs.h

all: $(TEST_EXES)

ArduinoMock.o: ArduinoMock.cpp Arduino.h simulator.h
	$(CXX) $(CXXFLAGS) -c ArduinoMock.cpp -o ArduinoMock.o

ArduinoMockSim.o: ArduinoMockSim.cpp Arduino.h simulator.h
	$(CXX) $(CXXFLAGS) -c ArduinoMockSim.cpp -o ArduinoMockSim.o

test_runner.o: test_runner.cpp Arduino.h simulator.h
	$(CXX) $(CXXFLAGS) -c test_runner.cpp -o test_runner.o

# For .ino files - use C++ mode
%.o: %.ino
	$(CXX) $(CXXFLAGS) -x c++ $(INC_FLAGS) -c $< -o $@

# For .c files
%.o: %.c
	$(CXX) $(CXXFLAGS) $(INC_FLAGS) -c $< -o $@

# Link test executables
%.test: %.o $(OBJS_COMMON) test_runner.o
	$(CXX) $(CXXFLAGS) $^ -o $@

%.csv: %.test
	./$< $@

%_results.png: %.csv
	python3 generate_graphs.py $< $(basename $<)

test: all
	@for test in $(TEST_EXES); do \
		echo "Running $$test..."; \
		./$$test > /dev/null && echo "  $$test passed" || echo "  $$test failed"; \
	done

report: $(PNG_FILES)
	@echo "Reports generated."

clean:
	rm -f *.o *.test *.csv *.png
