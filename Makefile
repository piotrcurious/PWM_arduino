
CXX = g++
CC = gcc
CXXFLAGS = -I. -Wall -Wno-unknown-pragmas
CFLAGS = -I. -Wall

SRCS_COMMON = ArduinoMock.cpp ArduinoMockSim.cpp
OBJS_COMMON = ArduinoMock.o ArduinoMockSim.o

INO_FILES = classic_PI.ino classic_voltage_current_limited.ino dumb_SR.ino inductance_estimator.ino very_simple_thermal_limited.ino very_simple_thermal_limited_with_WDT.ino weird_SR.ino
C_FILES = setup_pwm.c

TEST_EXES = $(INO_FILES:.ino=.test) $(C_FILES:.c=.test)

all: $(TEST_EXES)

ArduinoMock.o: ArduinoMock.cpp Arduino.h simulator.h
	$(CXX) $(CXXFLAGS) -c ArduinoMock.cpp -o ArduinoMock.o

ArduinoMockSim.o: ArduinoMockSim.cpp Arduino.h simulator.h
	$(CXX) $(CXXFLAGS) -c ArduinoMockSim.cpp -o ArduinoMockSim.o

test_runner.o: test_runner.cpp Arduino.h simulator.h
	$(CXX) $(CXXFLAGS) -c test_runner.cpp -o test_runner.o

# For .ino files: compile to .o first, then link
%.test: %.o $(OBJS_COMMON) test_runner.o
	$(CXX) $(CXXFLAGS) $^ -o $@

classic_PI.o: classic_PI.ino
	$(CXX) $(CXXFLAGS) -x c++ -include Arduino.h -include avr/io.h -include avr/wdt.h -include avr/interrupt.h -include shared_defs.h -c $< -o $@

classic_voltage_current_limited.o: classic_voltage_current_limited.ino
	$(CXX) $(CXXFLAGS) -x c++ -include Arduino.h -include avr/io.h -include avr/wdt.h -include avr/interrupt.h -include shared_defs.h -c $< -o $@

dumb_SR.o: dumb_SR.ino
	$(CXX) $(CXXFLAGS) -x c++ -include Arduino.h -include avr/io.h -include avr/wdt.h -include avr/interrupt.h -include shared_defs.h -c $< -o $@

inductance_estimator.o: inductance_estimator.ino
	$(CXX) $(CXXFLAGS) -x c++ -include Arduino.h -include avr/io.h -include avr/wdt.h -include avr/interrupt.h -include shared_defs.h -c $< -o $@

very_simple_thermal_limited.o: very_simple_thermal_limited.ino
	$(CXX) $(CXXFLAGS) -x c++ -include Arduino.h -include avr/io.h -include avr/wdt.h -include avr/interrupt.h -include shared_defs.h -c $< -o $@

very_simple_thermal_limited_with_WDT.o: very_simple_thermal_limited_with_WDT.ino
	$(CXX) $(CXXFLAGS) -x c++ -include Arduino.h -include avr/io.h -include avr/wdt.h -include avr/interrupt.h -include shared_defs.h -c $< -o $@

weird_SR.o: weird_SR.ino
	$(CXX) $(CXXFLAGS) -x c++ -include Arduino.h -include avr/io.h -include avr/wdt.h -include avr/interrupt.h -include shared_defs.h -c $< -o $@

setup_pwm.o: setup_pwm.c
	$(CXX) $(CXXFLAGS) -include Arduino.h -include avr/io.h -include avr/wdt.h -include avr/interrupt.h -include shared_defs.h -c $< -o $@

test: all
	@for test in $(TEST_EXES); do \
		echo "Running $$test..."; \
		./$$test > /dev/null && echo "  $$test passed" || echo "  $$test failed"; \
	done

clean:
	rm -f *.o *.test
