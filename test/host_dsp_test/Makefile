CC = gcc
CFLAGS = -Wall -Wextra -O2 -I../../components/dsp_pipeline/include -I../../components/system_config/include
LDFLAGS = -lm

all: test_dsp

test_dsp: test_dsp.c ../../components/dsp_pipeline/dsp_pipeline.c
	  -o $@  

run: test_dsp
	./test_dsp

clean:
	rm -f test_dsp *.o
