# Get compile/link flags from sdl2-config to assure portability
SDL2_CONFIG = sdl2-config
CFLAGS = $(shell $(SDL2_CONFIG) --cflags)
LDFLAGS = $(shell $(SDL2_CONFIG) --libs)
# Add other flags if necessary
LDFLAGS += -lm  
CFLAGS += -g

TARGET = project_testing_copy
OBJECTS = project_testing_copy.o ./fft/fft.o

# TARGET = simpleRecPlay
# OBJECTS = simpleRecPlay.o ./fft/fft.o

CC=gcc

all: $(TARGET)

$(TARGET): $(OBJECTS) 
	$(CC) -o $(TARGET) $(OBJECTS)  $(LDFLAGS) 
	
%.o:%.c
	$(CC) $(CFLAGS) -c $< -o $@ 

clean:
	rm -f *.o $(TARGET) $(OBJECTS)

run: $(TARGET)
	clear
	./$(TARGET) 0

# Some notes
# $@ represents the left side of the ":"
# $^ represents the right side of the ":"
# $< represents the first item in the dependency list   
