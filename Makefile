CXXFLAGS=-std=c++17 -O3 -m64 -pipe -w -pthread -ggdb -pg
#CXXFLAGS=-std=c++17 -Og -m64 -pipe -w -pthread -ggdb -pg
CXX=g++
SOLUTION_CODE_PATH=.
COMPILED_FILE_PATH=cpp1

SRCS = $(shell find ${SOLUTION_CODE_PATH} -type f -name '*.cpp')

all: ${SRCS}
	${CXX} ${CXXFLAGS} -o ${COMPILED_FILE_PATH} ${SRCS}
	./make_zip.sh

run: ${SRCS}
	${CXX} ${CXXFLAGS} -o ${COMPILED_FILE_PATH} ${SRCS}
	./${COMPILED_FILE_PATH} --test
