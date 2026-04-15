CXX= g++
CXXFLAGS = -g -Wall -Wextra -std=c++17 -O3

OBJS = puzzle.o

TARGET = puzzle

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LIBS)

all: $(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)

test: all
	./puzzle
