CXX = clang++
CXXFLAGS = -std=c++17 -O3 -Wall -Wextra -I/opt/homebrew/include -I.
LDFLAGS = -L/opt/homebrew/lib -lsfml-graphics -lsfml-window -lsfml-system

TARGET = smart_mars_rover
SOURCES = main.cpp

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(LDFLAGS) -o $(TARGET)

test: test_suite.cpp
	$(CXX) $(CXXFLAGS) test_suite.cpp -o test_suite
	./test_suite

clean:
	rm -f $(TARGET) test_suite *.o

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run test
